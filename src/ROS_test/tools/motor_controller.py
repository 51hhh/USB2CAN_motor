from dji_motor import GM6020
from pid import PID

class MotorController:
    """单个电机的双环PID控制器"""
    
    def __init__(self, motor_id, target_angle=0.0, 
                 speed_pid_params=None, angle_pid_params=None):
        """
        初始化单个电机控制器
        
        Args:
            motor_id: 电机ID (1-4)
            target_angle: 目标角度 (度)
            speed_pid_params: 速度环PID参数字典 {kp, ki, kd, i_max, out_max, dead_zone}
            angle_pid_params: 角度环PID参数字典 {kp, ki, kd, i_max, out_max, dead_zone}
        """
        self.motor = GM6020(motor_id)
        self.target_angle = target_angle
        self.enabled = True
        
        # 默认速度环参数
        if speed_pid_params is None:
            speed_pid_params = {
                'kp': 30.0, 'ki': 1.0, 'kd': 0.0,
                'i_max': 300, 'out_max': 10000, 'dead_zone': 5
            }
        
        # 默认角度环参数
        if angle_pid_params is None:
            angle_pid_params = {
                'kp': 10.0, 'ki': 1.0, 'kd': 0.0,
                'i_max': 10, 'out_max': 200, 'dead_zone': 0.5
            }
        
        # 初始化双环PID
        self.speed_pid = PID(**speed_pid_params)
        self.angle_pid = PID(**angle_pid_params)
    
    def get_motor_degree(self, raw_angle):
        """将 0-8191 映射到 0-360 度"""
        return (raw_angle / 8191.0) * 360.0
    
    def set_target_angle(self, angle):
        """设置目标角度"""
        self.target_angle = angle
    
    def calculate_output(self):
        """
        执行双环PID计算
        
        Returns:
            output_voltage: 控制电压输出
        """
        if not self.enabled:
            return 0
        
        # 获取当前状态
        current_angle = self.get_motor_degree(self.motor.angle)
        current_speed = self.motor.rpm
        
        # 串级PID计算
        # 1. 角度环 (外环) -> 输出目标速度
        ref_speed = self.angle_pid.calc(self.target_angle, current_angle)
        
        # 2. 速度环 (内环) -> 输出控制电压
        output_voltage = self.speed_pid.calc(ref_speed, current_speed)
        
        return output_voltage
    
    def get_status(self):
        """获取电机状态信息"""
        current_angle = self.get_motor_degree(self.motor.angle)
        return {
            'id': self.motor.id,
            'target_angle': self.target_angle,
            'current_angle': current_angle,
            'current_speed': self.motor.rpm,
            'current': self.motor.current,
            'temp': self.motor.temp,
            'enabled': self.enabled
        }
