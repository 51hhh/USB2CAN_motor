std_msgs/Header header
string joint_name
uint8 motor_id                 # 电机 ID
uint8 mode                     # 模式
bool online
float32 position               # 位置（弧度）
float32 velocity               # 速度（弧度/秒）
float32 torque                 # 力矩（Nm）
int16 acceleration             # 加速度
int8 temperature               # 温度
int8 error                     # 错误码
