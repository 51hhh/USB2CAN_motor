from motor_controller import MotorController

class MultiMotorManager:
    """多电机管理器"""
    
    def __init__(self, can_driver):
        """
        初始化多电机管理器
        
        Args:
            can_driver: CANDriver实例
        """
        self.driver = can_driver
        self.motors = {}  # {motor_id: MotorController}
        
        # 设置CAN接收回调
        self.driver.set_rx_callback(self._rx_handler)
    
    def add_motor(self, motor_id, target_angle=0.0, 
                  speed_pid_params=None, angle_pid_params=None):
        """
        添加一个电机到管理器
        
        Args:
            motor_id: 电机ID (1-4)
            target_angle: 初始目标角度
            speed_pid_params: 速度环PID参数
            angle_pid_params: 角度环PID参数
        
        Returns:
            MotorController实例
        """
        if motor_id in self.motors:
            print(f"[警告] 电机ID {motor_id} 已存在，将被覆盖")
        
        controller = MotorController(
            motor_id, target_angle, 
            speed_pid_params, angle_pid_params
        )
        self.motors[motor_id] = controller
        print(f"[多电机管理器] 添加电机 ID={motor_id}, 目标角度={target_angle}°")
        return controller
    
    def remove_motor(self, motor_id):
        """移除电机"""
        if motor_id in self.motors:
            del self.motors[motor_id]
            print(f"[多电机管理器] 移除电机 ID={motor_id}")
    
    def set_target_angle(self, motor_id, angle):
        """设置指定电机的目标角度"""
        if motor_id in self.motors:
            self.motors[motor_id].set_target_angle(angle)
    
    def enable_motor(self, motor_id, enabled=True):
        """启用/禁用指定电机"""
        if motor_id in self.motors:
            self.motors[motor_id].enabled = enabled
    
    def _rx_handler(self, can_id, data):
        """CAN接收回调处理"""
        for controller in self.motors.values():
            if can_id == controller.motor.feedback_id:
                controller.motor.parse_feedback(data)
                break
    
    def update_all(self):
        """
        更新所有电机的控制输出
        
        Returns:
            dict: {control_id: [voltage_list]}
        """
        # 按控制ID分组 (GM6020的1-4号都用0x1FF)
        control_groups = {}
        
        for controller in self.motors.values():
            control_id = controller.motor.control_id
            
            if control_id not in control_groups:
                control_groups[control_id] = [0] * 8
            
            # 计算输出
            output = controller.calculate_output()
            voltage_bytes = controller.motor.get_voltage_bytes(output)
            
            # 填入对应位置 (ID 1-4 -> 索引 0,2,4,6)
            idx = (controller.motor.id - 1) * 2
            if idx < 7:
                control_groups[control_id][idx] = voltage_bytes[0]
                control_groups[control_id][idx + 1] = voltage_bytes[1]
        
        return control_groups
    
    def send_commands(self):
        """发送所有电机的控制命令"""
        control_groups = self.update_all()
        
        for control_id, payload in control_groups.items():
            self.driver.send_can_frame(control_id, payload)
    
    def stop_all(self):
        """停止所有电机"""
        # 获取所有使用的控制ID
        control_ids = set(m.motor.control_id for m in self.motors.values())
        
        for control_id in control_ids:
            self.driver.send_can_frame(control_id, [0] * 8)
    
    def get_all_status(self):
        """获取所有电机状态"""
        return {motor_id: controller.get_status() 
                for motor_id, controller in self.motors.items()}
    
    def print_status(self):
        """打印所有电机状态"""
        for motor_id in sorted(self.motors.keys()):
            status = self.motors[motor_id].get_status()
            ref_speed = self.motors[motor_id].angle_pid.output
            output = self.motors[motor_id].speed_pid.output
            
            print(f"[ID{motor_id}] "
                  f"Ang: {status['current_angle']:6.1f}°→{status['target_angle']:6.1f}° | "
                  f"Spd: {status['current_speed']:5d}→{ref_speed:6.1f} | "
                  f"Out: {int(output):6d} | "
                  f"{'EN' if status['enabled'] else 'DIS'}")
