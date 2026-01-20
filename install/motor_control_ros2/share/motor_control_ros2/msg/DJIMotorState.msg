std_msgs/Header header
string joint_name
string model                   # "GM3508" 或 "GM6020"
bool online
float64 angle                  # 角度（度，0-360）
int16 rpm                      # 转速
int16 current                  # 实际电流
uint8 temperature              # 温度
float64 control_frequency      # 控制频率 (Hz)
