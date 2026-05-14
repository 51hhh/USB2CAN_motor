std_msgs/Header header

# 控制模式
uint8 MODE_IDLE = 0          # 空闲（刹车）
uint8 MODE_TASK_SPACE = 1    # 任务空间：给定末端 (x, y)(米)，IK 解算
uint8 MODE_JOINT_SPACE = 2   # 关节空间：直接给定 θ1, θ2_rel（弧度）
uint8 mode

# 任务空间目标（MODE_TASK_SPACE 时使用）
float64 x                    # 末端 x 坐标（米），以肩关节为原点
float64 y                    # 末端 y 坐标（米），以肩关节为原点

# 击球管理器目标（volleyball_strike_manager 使用，基座中心参考系）
float64 x_target             # 击球点 X 坐标 (m)
float64 y_target             # 击球点 Y 坐标 (m)

# 关节空间目标（MODE_JOINT_SPACE 时使用，左右对称）
float64 theta1               # 大臂绝对角度（弧度），从+X轴逆时针为正
float64 theta2_rel           # 小臂相对角度（弧度），相对大臂

# 速度前馈（可选，任务空间）
float64 dx                   # 末端 x 速度（m/s）
float64 dy                   # 末端 y 速度（m/s）

# PD 增益覆盖（0 表示使用默认值）
float32 kp
float32 kd
