std_msgs/Header header
string joint_name
string model                   # "DM4310", "DM4340" 等
bool online
float32 position               # 位置（弧度）
float32 velocity               # 速度（弧度/秒）
float32 torque                 # 力矩（Nm）
uint8 temp_mos                 # MOS 温度
uint8 temp_rotor               # 转子温度
uint8 error_code               # 错误码
