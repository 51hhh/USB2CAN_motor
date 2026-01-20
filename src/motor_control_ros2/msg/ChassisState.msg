std_msgs/Header header

# 底盘当前速度（里程计）
float64 linear_x
float64 linear_y
float64 angular_z

# 4 个舵轮单元状态
SteerWheelState fl_wheel
SteerWheelState fr_wheel
SteerWheelState rl_wheel
SteerWheelState rr_wheel
