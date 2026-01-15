import struct

class GM6020:
    def __init__(self, motor_id):
        """
        motor_id: 拨码开关设定的 ID (1-4)
        """
        self.id = motor_id
        
        # GM6020 ID映射逻辑
        # 反馈 ID: 1->0x205, 2->0x206 ...
        self.feedback_id = 0x205 + (motor_id - 1)
        
        # 控制 ID: 1-4号电机统一用 0x1FF
        self.control_id = 0x1FF
        
        # 状态存储
        self.angle = 0     # 0-8191
        self.rpm = 0       # 转速
        self.current = 0   # 实际转矩电流
        self.temp = 0      # 温度

    def parse_feedback(self, payload):
        """解析8字节反馈数据"""
        if len(payload) < 8: return
        # DJI 格式 (Big Endian): Angle(2) RPM(2) Current(2) Temp(1) Null(1)
        (self.angle, self.rpm, self.current, self.temp) = struct.unpack('>hhhB', payload[:7])

    def get_voltage_bytes(self, voltage):
        """
        GM6020 电压控制模式
        范围: -30000 ~ 30000
        """
        limit = 30000
        if voltage > limit: voltage = limit
        if voltage < -limit: voltage = -limit
        
        val = int(voltage)
        # Big Endian 转换
        return [(val >> 8) & 0xFF, val & 0xFF]