import time
from can_driver import CANDriver
from dji_motor import GM6020
from pid import PID

# === 关节封装类 ===
class GimbalJoint:
    def __init__(self, motor_id, target_angle=0.0):
        self.motor_id = motor_id
        self.target_angle = target_angle
        
        # 1. 硬件对象
        self.motor = GM6020(motor_id)
        
        # 2. 控制算法 (每个电机拥有独立的 PID 实例)
        # 这里使用您调好的参数
        self.speed_pid = PID(kp=30.0, ki=1.0, kd=0.0, i_max=300, out_max=10000, dead_zone=5)
        self.angle_pid = PID(kp=10.0, ki=1.0, kd=0.0, i_max=10, out_max=200, dead_zone=0.5)
        
        # 状态记录
        self.current_output = 0

    def update(self):
        """计算双环 PID，返回控制电流值"""
        # 转换角度
        current_angle_deg = (self.motor.angle / 8191.0) * 360.0
        current_speed = self.motor.rpm

        # 1. 角度环计算
        ref_speed = self.angle_pid.calc(self.target_angle, current_angle_deg)
        
        # 2. 速度环计算
        output_voltage = self.speed_pid.calc(ref_speed, current_speed)
        
        self.current_output = output_voltage
        return output_voltage, current_angle_deg, ref_speed

    def get_can_bytes(self):
        """获取该电机的控制字节(2 byte)"""
        return self.motor.get_voltage_bytes(self.current_output)

# === 全局配置 ===
# 在这里添加多个电机，例如: {1: joint1, 2: joint2}
joints = {}

# 添加 1 号电机 (Yaw)
joints[1] = GimbalJoint(motor_id=1, target_angle=180.0)

# 示例：添加 2 号电机 (Pitch) - 如果你有的话，取消注释即可
joints[2] = GimbalJoint(motor_id=2, target_angle=180.0)

# === 初始化 CAN ===
driver = CANDriver()

# === 接收回调 (支持多电机) ===
def rx_handler(can_id, data):
    # 遍历所有关节，找到匹配 ID 的电机进行解析
    for joint in joints.values():
        if can_id == joint.motor.feedback_id:
            joint.motor.parse_feedback(data)
            break

driver.set_rx_callback(rx_handler)

# === 主循环 ===
try:
    print(f"🚀 启动多电机控制系统")
    print(f"当前激活电机 ID: {list(joints.keys())}")

    # 等待所有电机上线
    print("等待电机反馈...")
    all_ready = False
    while not all_ready:
        all_ready = True
        for j in joints.values():
            if j.motor.angle == 0 and j.motor.rpm == 0:
                all_ready = False
        if not all_ready:
            time.sleep(0.1)
    
    print("✅ 所有电机就绪，开始闭环控制...")

    while True:
        # --- 1. 计算所有电机的 PID ---
        # 0x1FF 帧数据缓冲 (对应 ID 1-4)
        can_payload_1ff = [0] * 8
        # 0x200 帧数据缓冲 (对应 ID 5-8, 如果有的话)
        can_payload_200 = [0] * 8
        
        log_info = ""

        for joint_id, joint in joints.items():
            # 计算 PID
            out, real_ang, tgt_spd = joint.update()
            
            # 获取控制字节
            cmd_bytes = joint.get_can_bytes()
            
            # --- 2. 填充 CAN 数据包 ---
            # DJI 协议: 
            # 0x1FF 控制 ID 1-4 (索引 0-7)
            # 0x200 控制 ID 5-8 (索引 0-7)
            
            if 1 <= joint_id <= 4:
                idx = (joint_id - 1) * 2
                can_payload_1ff[idx] = cmd_bytes[0]
                can_payload_1ff[idx+1] = cmd_bytes[1]
            elif 5 <= joint_id <= 8:
                idx = (joint_id - 5) * 2
                can_payload_200[idx] = cmd_bytes[0]
                can_payload_200[idx+1] = cmd_bytes[1]

            # 拼接打印信息 (只显示 ID 和 角度)
            log_info += f"[ID{joint_id}] Tgt:{joint.target_angle:.1f} Real:{real_ang:.1f} Out:{int(out)} | "

        # --- 3. 发送 CAN 帧 ---
        # 发送 1-4 号组
        driver.send_can_frame(0x1FF, can_payload_1ff)
        
        # 如果有 5-8 号电机，发送 200 组
        # driver.send_can_frame(0x200, can_payload_200)

        # --- 4. 打印 ---
        print(log_info)
        
        time.sleep(0.005) # 200Hz

except KeyboardInterrupt:
    print("\n🛑 系统停止，释放所有电机...")
    driver.send_can_frame(0x1FF, [0]*8)
    driver.send_can_frame(0x200, [0]*8)
    driver.running = False