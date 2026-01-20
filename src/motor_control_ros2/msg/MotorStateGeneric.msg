std_msgs/Header header
string joint_name              # 关节名称
string motor_type              # 电机类型: "dji", "damiao", "unitree"
bool online                    # 是否在线
float64 position               # 位置（弧度）
float64 velocity               # 速度（弧度/秒）
float64 torque                 # 力矩（Nm）
float32 temperature            # 温度（摄氏度）
