# 单个舵轮单元状态

# 转向电机 (GM6020)
string steer_motor_name
float64 steer_angle         # 当前转向角度 (度, 0-360)
float64 steer_angle_target  # 目标转向角度 (度)

# 驱动电机 (GM3508)
string drive_motor_name
float64 drive_velocity      # 当前驱动速度 (m/s)
float64 drive_velocity_target  # 目标驱动速度 (m/s)
