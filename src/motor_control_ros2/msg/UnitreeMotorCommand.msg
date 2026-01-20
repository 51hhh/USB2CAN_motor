std_msgs/Header header
string joint_name
uint8 leg_id
uint8 motor_id
uint8 mode                     # 0:空闲, 5:开环, 10:闭环FOC
float32 kp                     # 关节刚度系数
float32 kd                     # 关节速度系数
float32 pos_des                # 期望位置（弧度）
float32 vel_des                # 期望速度（弧度/秒）
float32 torque_ff              # 期望力矩（Nm）
