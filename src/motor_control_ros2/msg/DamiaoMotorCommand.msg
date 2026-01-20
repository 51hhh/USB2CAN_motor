std_msgs/Header header
string joint_name
float32 pos_des                # 期望位置（弧度）
float32 vel_des                # 期望速度（弧度/秒）
float32 kp                     # 位置增益
float32 kd                     # 速度增益
float32 torque_ff              # 前馈力矩（Nm）
