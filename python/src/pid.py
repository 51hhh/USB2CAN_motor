class PID:
    def __init__(self, kp, ki, kd, i_max, out_max, dead_zone=0.0):
        """
        增加 dead_zone 参数:
        如果 abs(error) < dead_zone，则强制 error = 0
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max
        self.out_max = out_max
        self.dead_zone = dead_zone  # 新增死区阈值
        
        # 状态变量
        self.ref = 0.0
        self.fdb = 0.0
        self.err = [0.0, 0.0]
        self.p_out = 0.0
        self.i_out = 0.0
        self.d_out = 0.0
        self.output = 0.0

    def calc(self, ref, fdb):
        self.ref = ref
        self.fdb = fdb
        
        # 1. 计算原始误差
        raw_error = self.ref - self.fdb
        
        # 2. 死区处理 (Dead Zone Logic)
        # 如果误差在死区范围内，强制归零，避免微小震荡
        if abs(raw_error) < self.dead_zone:
            self.err[0] = 0.0
        else:
            self.err[0] = raw_error

        # 3. 移位保存上次误差 (注意：这里保存的是处理过死区后的误差，以保证微分平滑)
        # 如果想让微分更敏感，也可以把 raw_error 存入，但通常对死区归零后的平滑更重要
        
        # 4. 计算 P
        self.p_out = self.kp * self.err[0]
        
        # 5. 计算 I
        self.i_out += self.ki * self.err[0]
        
        # 6. 计算 D
        self.d_out = self.kd * (self.err[0] - self.err[1])
        
        # 移位
        self.err[1] = self.err[0]
        
        # 积分限幅
        if self.i_out > self.i_max:
            self.i_out = self.i_max
        elif self.i_out < -self.i_max:
            self.i_out = -self.i_max
            
        # 总输出
        self.output = self.p_out + self.i_out + self.d_out
        
        # 输出限幅
        if self.output > self.out_max:
            self.output = self.out_max
        elif self.output < -self.out_max:
            self.output = -self.out_max
            
        return self.output