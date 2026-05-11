# VolleyballUpwardStrikeManager 开发计划

> 2026-04-01 专家委员会（架构师 + 开发 + 审查官）一致通过方案

## 一、现有框架问题检查列表

| # | 问题 | 严重度 | 位置 | 修复方案 |
|---|------|--------|------|---------|
| 1 | 传动拓扑不匹配：代码为"基座集中驱动 50-200-50-200"，实际为"大臂上平行四边形 14-200-14-200" | 🔴 致命 | hpp#L34-L42, cpp#L326-L440 | 删除 decoupleToMotor/decoupleToMath/getLinkageMotorOffset，IK 直接返回 q_rel |
| 2 | q2 角度语义错误：代码将 q2 视为"小臂绝对角"，实际 14-200-14-200 映射 q_motor = θ_relative | 🔴 致命 | cpp#L254-L268 | solveIK 中删除 forearm_abs 计算，直接 q_L2 = math_q2_L_rel |
| 3 | ArmGeometry 含冗余连杆参数 | 🟠 严重 | hpp#L34-L42 | 删除冗余字段，仅保留 D, L1, L2, W |
| 4 | 缺少重力补偿前馈 | 🟠 严重 | 全局 | 新增 computeGravityCompensation()，各状态叠加 τ_g |
| 5 | IDLE 状态无反馈时 p_des=0 暴走风险 | 🟡 中等 | cpp#L600 | 无反馈时发零力矩命令 |
| 6 | 缺少 L1=R1, L2=R2 镜像强制 | 🟡 中等 | solveIK | IK 后添加对称平均 |
| 7 | E_STOP 瞬间全 Kd 冲击 | 🟡 中等 | cpp#L822 | E_STOP Kd ramp |
| 8 | 缺少配置文件 | 🟡 中等 | 缺失 | 新建 yaml |

## 二、技术选型

### 运动学
- IK: 标准 2R 解析解（保留）
- 传动: 14-200-14-200 精确平行四边形 → q_motor = q_relative（直通）
- q1 = 大臂绝对角（直驱），q2 = 肘关节折叠角（相对角）

### 状态机（保留6态）
IDLE → WIND_UP → STRIKE_ACCEL → IMPACT_MOMENT → FOLLOW_THROUGH → IDLE + E_STOP

### 重力补偿
τ_g1 = (m1·g·L1/2 + m2·g·L1)·cos(q1) + m2·g·L2/2·cos(q1+q2)
τ_g2 = m2·g·L2/2·cos(q1+q2)

## 三、开发 TODO
- [x] 探索现有项目结构
- [x] 召开专家委员会讨论
- [x] 生成 plan.md
- [x] 修改 hpp + cpp（传动映射/重力补偿/镜像/安全）
- [x] 创建 volleyball_strike_params.yaml
- [x] colcon build 编译验证（0 errors, 0 warnings — 2026-04-01）

## 四、开机归零（单圈绝对编码器）

### 归零背景
- GO-8010 为单圈绝对编码器，开机初始读数受停机姿态影响，每次可能不同。
- 因此不能把开机读数直接当逻辑零点，必须先“撞限位”建立统一参考。

### 归零顺序（已按此实现）
1. **先小臂归零（L2/R2）**：小臂向“最小角”方向运动，撞到机械限位后将该位置设为 `q2_min = 0`。
2. **再大臂归零（L1/R1）**：大臂向下运动（你的实测中角度实际增大方向），撞到机械限位后将该位置设为 `q1_min = 0`。
3. 完成后系统进入 `IDLE`，后续所有控制在“解耦后的逻辑坐标系”下进行：
	 - `q_logical = (q_raw - zero_offset) / direction`
	 - `q_raw_cmd = q_logical_cmd * direction + zero_offset`

### 你提供的两组实测参考值

#### A) 大臂最低点 + 小臂最小夹角（建议作为最小限位参考）
| 电机 | 读数(rad) |
|---|---:|
| arm_motor_l_1 | 1.118 |
| arm_motor_l_2 | -0.368 |
| arm_motor_r_1 | 0.028 |
| arm_motor_r_2 | 1.698 |

#### B) 大臂最高点 + 小臂最大夹角
| 电机 | 读数(rad) |
|---|---:|
| arm_motor_l_1 | -0.710 |
| arm_motor_l_2 | 1.067 |
| arm_motor_r_1 | 1.944 |
| arm_motor_r_2 | 0.211 |

### 调参提示（特别是右臂）
- 若启动后右臂朝反方向走，优先检查：
	1. `motors[*].direction`
	2. `homing.torque_to_min` 中 `R1/R2` 的符号
- 原则：**“向最小限位运动”的力矩方向必须正确**，方向不对就把对应项改成负值。
