std_msgs/Header header

# 底盘当前速度（里程计）
float64 linear_x
float64 linear_y
float64 angular_z

# 最近收到的原始底盘命令（限幅前）
float64 requested_linear_x
float64 requested_linear_y
float64 requested_angular_z

# 实际下发给舵轮分配器的命令（限幅/安全门禁后）
float64 command_linear_x
float64 command_linear_y
float64 command_angular_z

# 控制链安全状态
bool all_motors_online
bool all_motors_healthy
bool command_timed_out
bool estop_active
bool safety_stop

# 4 个舵轮单元状态
SteerWheelState fl_wheel
SteerWheelState fr_wheel
SteerWheelState rl_wheel
SteerWheelState rr_wheel
