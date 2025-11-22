import time
from can_driver import CANDriver
from dji_motor import GM6020
from pid import PID

# === 配置 ===
MOTOR_ID = 1
# 目标角度 (单位：度)
TARGET_ANGLE = 90.0   

# === 初始化 ===
driver = CANDriver()
motor = GM6020(MOTOR_ID)

# === 初始化双环 PID (使用你提供的 C 代码参数) ===

# 1. 速度环 (内环)
speed_pid = PID(kp=30.0, ki=1.0, kd=0.0, i_max=300, out_max=10000,dead_zone=5)

# 2. 角度环 (外环)
angle_pid = PID(kp=10.0, ki=1.0, kd=0.0, i_max=10, out_max=200,dead_zone=0.5)

# === 接收回调 ===
def rx_handler(can_id, data):
    if can_id == motor.feedback_id:
        motor.parse_feedback(data)

driver.set_rx_callback(rx_handler)

# === 辅助函数：处理角度过零 ===
def get_motor_degree(raw_angle):
    """将 0-8191 映射到 0-360 度"""
    return (raw_angle / 8191.0) * 360.0

# === 主控制循环 ===
try:
    print(f"开始双环控制 (ID: {MOTOR_ID})")
    print(f"目标角度: {TARGET_ANGLE}°")
    
    # 等待电机初始数据返回，防止一开始 PID 疯转
    print("等待电机反馈...")
    while motor.angle == 0 and motor.rpm == 0:
        time.sleep(0.1)
    print("电机已就绪，开始闭环。")

    while True:
        # 1. 获取当前状态
        # 将 0-8191 转换为 0-360 度，方便 PID 计算
        current_angle = get_motor_degree(motor.angle)
        current_speed = motor.rpm

        # --- 串级 PID 计算 ---
        
        # 第一步：角度环计算 (外环)
        # 输入：目标角度, 实际角度
        # 输出：目标速度 (ref_speed)
        # 注意：这里是简单的线性 PID，没有做过零处理（比如从 359 到 1 度）。
        # 如果需要转多圈或过零，需要特殊的误差处理逻辑。这里严格遵照你的 C 代码逻辑。
        ref_speed = angle_pid.calc(TARGET_ANGLE, current_angle)
        
        # 第二步：速度环计算 (内环)
        # 输入：目标速度 (来自角度环), 实际速度
        # 输出：控制电压/电流
        output_voltage = speed_pid.calc(ref_speed, current_speed)
        
        # --------------------

        # 2. 发送控制命令
        can_payload = [0] * 8
        cmd_bytes = motor.get_voltage_bytes(output_voltage)
        
        idx = (MOTOR_ID - 1) * 2
        if idx < 7:
            can_payload[idx] = cmd_bytes[0]
            can_payload[idx+1] = cmd_bytes[1]
            
        driver.send_can_frame(motor.control_id, can_payload)
        
        # 3. 打印状态
        print(f"Angle Tgt:{TARGET_ANGLE:.1f} Real:{current_angle:.1f} | "
              f"Spd Tgt:{ref_speed:.1f} Real:{current_speed} | "
              f"Out: {int(output_voltage)}")
        
        time.sleep(0.005) # 200Hz

except KeyboardInterrupt:
    print("停止...")
    driver.send_can_frame(motor.control_id, [0]*8)
    driver.running = False