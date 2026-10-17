# 单个舵轮单元状态

# 转向电机 (GM6020)
string steer_motor_name
float64 steer_angle         # 当前转向角度 (度, 0-360)
float64 steer_angle_target  # 目标转向角度 (度)

# 驱动电机 (GM3508)
string drive_motor_name
float64 drive_velocity      # 当前驱动速度 (m/s)
float64 drive_velocity_target  # 目标驱动速度 (m/s)

# 安全与录制状态
bool steer_online
bool drive_online
bool steer_saturated
bool drive_saturated
bool steer_fault
bool drive_fault
int16 steer_current
int16 drive_current
uint8 steer_temperature
uint8 drive_temperature
