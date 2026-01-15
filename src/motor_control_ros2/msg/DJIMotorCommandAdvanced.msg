std_msgs/Header header
string joint_name

# 控制模式
uint8 MODE_DIRECT = 0      # 直接输出（电流/电压）
uint8 MODE_VELOCITY = 1    # 速度控制
uint8 MODE_POSITION = 2    # 位置控制
uint8 mode

# 目标值（根据模式使用对应字段）
float64 position_target    # 位置目标（弧度），mode=2 时使用
float64 velocity_target    # 速度目标（弧度/秒），mode=1 或 mode=2 时使用
int16 direct_output        # 直接输出（GM3508: 电流, GM6020: 电压），mode=0 时使用
