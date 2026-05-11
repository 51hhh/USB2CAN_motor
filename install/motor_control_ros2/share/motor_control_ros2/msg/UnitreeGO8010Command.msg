std_msgs/Header header

uint8 id                       # 电机 ID (0-15)
string device                  # 串口设备路径（如 /dev/ttyUSB0），为空时广播到所有同 ID 电机

# 控制模式
uint8 MODE_BRAKE = 0       # 刹车模式
uint8 MODE_FOC = 1         # FOC闭环控制
uint8 MODE_CALIBRATE = 2   # 校准模式
uint8 mode

# 目标值（FOC模式使用）
float64 position_target    # 期望位置（弧度），FOC模式时使用
float64 velocity_target    # 期望速度（弧度/秒），FOC模式时使用
float32 torque_ff          # 期望力矩（Nm），FOC模式时使用
float32 kp                 # 关节刚度系数，FOC模式时使用
float32 kd                 # 关节阻尼系数，FOC模式时使用
