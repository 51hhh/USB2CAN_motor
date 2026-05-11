# 关节轨迹同步算法讨论

> 2026-04-09 讨论文档  
> 背景：排球击球系统 `volleyball_strike_manager` 的 `updateTrajectory` 优化

---

## 一、当前算法（独立规划器）

### 原理

每个关节**独立**跑到 `max_joint_velocity` / `max_joint_acceleration`，不考虑其他关节状态。

```
for each joint i:
    err = target[i] - current[i]
    if far from target: accelerate to max_velocity
    if near target: decelerate (v = sqrt(2·a·|err|))
    clamp velocity and acceleration
    integrate position
```

### 行为特征

```
假设：大臂需走 1.0 rad，小臂需走 0.1 rad
max_v = 30 rad/s, max_a = 120 rad/s²

大臂：加速区(0.25s) → 匀速区 → 减速区 → 总时间 ≈ 0.08s
小臂：加速区(0.02s) → 直接减速 → 总时间 ≈ 0.03s

结果：小臂 0.03s 先到位，大臂 0.08s 后到位
      中间 0.05s 内只有大臂在动，小臂保持目标位
```

### 优点

- 简单、鲁棒、计算量极小
- 每个关节都以最大能力运动 → 总时间由最慢关节决定 = 最快到达
- 关节独立 → 任何一个关节卡住不影响其他关节

### 缺点

- **笛卡尔路径不可预测**：大小臂到达时间不同 → 末端轨迹是弯曲/扭曲的
- **对闭链有应力**：左右臂不同步到达 → 瞬态内力
- 但在当前对称构型下（L1=R1, L2=R2），左右两侧总是同步的，内力风险仅存在于大小臂时间差

---

## 二、Time-Scaling 算法（已尝试并回退）

### 原理

每帧动态计算各关节的**剩余误差比例**，按比例缩放速度/加速度上限：

```
max_err = max(|err[0..3]|)
for each joint i:
    scale = |err[i]| / max_err
    local_max_v = max_velocity * scale
    local_max_a = max_acceleration * scale
    // 再跑和独立规划器相同的加减速逻辑
```

### 失败原因分析

**核心问题**：每帧重新计算 `scale` 导致比例系数随时间变化。

```
帧0:    err_大臂=1.0  err_小臂=0.1  → scale_小臂=0.10 → v_max=3.0
帧50:   err_大臂=0.5  err_小臂=0.05 → scale_小臂=0.10 → v_max=3.0 ← 看似OK

但当一个关节接近目标时:
帧99:   err_大臂=0.01 err_小臂=0.003 → scale_小臂=0.30 → v_max=9.0 ← 突然加速
帧100:  err_大臂=0.001 err_小臂=0.002 → max_err=0.002, scale_小臂=1.0 → v_max=30！← 全速！
```

**结果**：行程短的关节全程被压制，只在最后几帧突然加速 → 视觉上"先动大臂，再动小臂"。

---

## 三、正确的 Time-Scaling 方案（待实现）

### 方案 A：初始比例缓存

在每次设置新 `traj_target_` 时，**一次性**计算并存储各关节的速度比例：

```cpp
// 在 traj_target_ 被设置的地方（strikeCommandCallback、handleWindUp 等）
double max_initial_err = 1e-6;
for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double err = std::abs(traj_target_[i] - traj_state_[i].position);
    if (err > max_initial_err) max_initial_err = err;
}
for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double err = std::abs(traj_target_[i] - traj_state_[i].position);
    traj_sync_scale_[i] = std::max(err / max_initial_err, 0.01);
}
```

`updateTrajectory` 中使用**固定的** `traj_sync_scale_[i]`：

```cpp
void updateTrajectory(double dt) {
    for (size_t i = 0; i < NUM_MOTORS; ++i) {
        double local_max_v = max_joint_velocity_ * traj_sync_scale_[i];
        double local_max_a = max_joint_acceleration_ * traj_sync_scale_[i];
        local_max_v = std::max(local_max_v, 0.05);
        local_max_a = std::max(local_max_a, 0.5);
        // ... 同样的加减速逻辑 ...
    }
}
```

**优点**：
- 比例恒定，全程协调运动
- 所有关节同时起步、同时到达
- 笛卡尔路径平滑

**缺点**：
- 需要在 6+ 处设置 `traj_target_` 的地方同步计算 `traj_sync_scale_`
- 如果行程极端不对称（一个关节不动、另一个走1 rad），不动的关节被限速到 0.01 × max → 数值抖动

### 方案 B：最长时间归一化

计算每个关节独立规划需要的时间，取最大值，然后反推每个关节的速度上限：

```cpp
// 每个关节需要的最短时间（梯形速度曲线）
double max_time = 0.0;
for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double dist = std::abs(traj_target_[i] - traj_state_[i].position);
    // 梯形曲线: T = v/a + dist/v（简化估算）
    double t_accel = max_joint_velocity_ / max_joint_acceleration_;
    double d_accel = 0.5 * max_joint_acceleration_ * t_accel * t_accel;
    double t;
    if (dist <= 2 * d_accel) {
        // 三角形曲线（来不及达到 max_v）
        t = 2.0 * std::sqrt(dist / max_joint_acceleration_);
    } else {
        // 梯形曲线
        t = 2.0 * t_accel + (dist - 2 * d_accel) / max_joint_velocity_;
    }
    if (t > max_time) max_time = t;
}

// 反推每个关节的 v_max 和 a_max
for (size_t i = 0; i < NUM_MOTORS; ++i) {
    double dist = std::abs(traj_target_[i] - traj_state_[i].position);
    // 按照 max_time 倒推这个关节的 v_max
    // 简化：假设梯形，a 不变，缩放 v
    traj_sync_v_max_[i] = std::max(dist / max_time * 1.5, 0.05);  // 1.5 = 补偿加减速
    traj_sync_a_max_[i] = std::max(traj_sync_v_max_[i] / (max_time * 0.3), 0.5);
}
```

**优点**：物理上最正确，精确保证同时到达

**缺点**：计算复杂，梯形/三角形曲线判断需要仔细

### 方案 C：不改轨迹规划，改笛卡尔空间插值

完全不在关节空间做同步，而是在**笛卡尔空间**插值：

```
1. 计算起始末端位置 FK(q_start) → (x0, y0)
2. 计算目标末端位置 (x_target, y_target)
3. 在笛卡尔空间做直线/曲线插值: (x(t), y(t))
4. 每帧对 (x(t), y(t)) 做 IK → 得到关节角
5. 关节角直接作为 p_des 发送（无需关节空间轨迹规划）
```

**优点**：
- 末端轨迹完美可控（直线 / 弧线 / S-curve）
- 笛卡尔速度/加速度可控
- 对人类观察最自然

**缺点**：
- 每帧调用 IK → 计算量增加（但 2R 解析解很快，不是问题）
- IK 可能在中间点无解（需要规划安全路径）
- 笛卡尔空间速度限制不直接对应关节空间限制

---

## 四、我的建议

### 对当前排球击球系统

| 阶段 | 推荐算法 | 理由 |
|------|---------|------|
| **WIND_UP** | 方案A（初始比例缓存） | 引拍需要协调运动，避免手臂乱甩 |
| **STRIKE_ACCEL** | 独立规划器（当前） | 追求极限速度，大小臂不同步问题不大 |
| **SETTLING** | 方案A 或 独立 | 回位时协调性锦上添花但非必须 |

### 优先级

1. **先验证当前版本**（独立规划 + max_position_error=0.5 + SETTLING→READY）
2. 如果 WIND_UP 阶段的路径不够优雅，再加入方案A
3. 方案C（笛卡尔插值）是终极方案但目前过早优化

### 需要新增的成员变量（方案A）

```cpp
// hpp 中添加:
std::array<double, NUM_MOTORS> traj_sync_scale_;  // 关节同步缩放系数（目标切换时计算）

// 新增辅助函数:
void computeTrajSyncScale();  // 在设定 traj_target_ 后调用
```

---

## 五、待验证清单

- [x] `max_position_error=0.5` 后击球速度是否恢复正常 → 已修改，部分改善
- [x] SETTLING → READY 后是否不再发生猛撞限位 → ✅ 已修复
- [ ] ~~独立规划器下大小臂运动是否可接受~~ → ❌ 不可接受
- [x] 如果不可接受，实施方案A → ✅ 已实施但效果有限（见第六章分析）

---

## 六、实机测试分析：方案A + 参数调优 (2026-04-10)

### 6.1 测试环境

- 方案A（初始比例缓存）已实施
- 参数调整：`virtual_overshoot_y=0.30`, `impact_y_threshold=0.02`, `impact_window_after_ms=20`
- 测试目标：`(0.380, -0.110)` 和 `(0.400, -0.110)`

### 6.2 日志关键数据

#### Strike #1: (0.380, -0.110)

```
击球IK: [L1=-0.672 L2=0.737] | 引拍IK: [L1=-0.579 L2=0.258]
当前位: [L1=-1.558 L2=1.218] | FK_Y=-0.275 m

WIND_UP (243ms): FK_Y → -0.183 m
STRIKE_ACCEL (110ms): 关节速度 [0.4, 4.7, 0.4, 4.7] rad/s | FK_Y → -0.127
IMPACT (25ms): FK_Y → -0.098 | overshoot = +0.012 ✅ 穿过目标！
```

#### Strike #2/#3: (0.400, -0.110)

```
[WARN] 虚拟目标 IK 无解，缩减过冲距离重试       ← ❌ 0.30 被截断
[WARN] 引拍回退自适应缩减至 30%: y=-0.140        ← ❌ backoff 也被截断

击球IK: [L1=-0.489 L2=0.417] | 引拍IK: [L1=-0.370 L2=0.063]
当前位: [L1=-1.546 L2=1.217]

WIND_UP (257ms): FK_Y → -0.153 m
STRIKE_ACCEL (60ms): 关节速度 [0.5, 6.9, 0.5, 6.9] rad/s | FK_Y → -0.124
IMPACT (25ms): FK_Y → -0.123 | overshoot = -0.013 ❌ 未穿过
```

### 6.3 根因分析：IK 工作空间硬约束

`virtual_overshoot_y=0.30` 在 x=0.400 时导致 IK 无解：

```
虚拟目标: (0.400, -0.110 + 0.30) = (0.400, 0.190)
到原点距离: d = √(0.16 + 0.0361) = 0.443
臂展极限:   L1 + L2 = 0.424
0.443 > 0.424 → IK 无解！
```

代码自动缩减到 50%：有效 overshoot = 0.15m（旧值 0.10m → 仅提升 50%）

引拍 backoff 被缩减到 30%：有效 backoff = 0.03m（旧值 0.10m → 反而缩短了）

**结论：在 x=0.400 附近，y 方向可用空间极其有限，参数调优收益递减。**

### 6.4 STRIKE_ACCEL 关节行程分析

| 测试 | L1 行程 | L2 行程 | 比例 | 说明 |
|------|---------|---------|------|------|
| x=0.380 | 0.093 rad (5°) | 0.479 rad (27°) | 1:5.2 | 小臂主导 |
| x=0.400 | 0.036 rad (2°) | 0.615 rad (35°) | 1:17 | 几乎纯小臂 |

**x 越接近臂展极限，肩关节对 Y 越不敏感。** 在 x=0.400 时，STRIKE_ACCEL 阶段的肩角变化仅 2°，
无论使用何种同步算法（方案A/B/C），视觉效果都是"只有小臂在动"。

**方案A 在 STRIKE_ACCEL 确实让两关节同时到达**，但因为肩关节仅需移动 2°，
人眼根本感知不到肩关节在动。同步问题的本质不是时间调度，而是 **运动量分配不均**。

---

## 七、新方案：对角线挥拍 (Diagonal Sweep)

### 7.1 核心思路

**将引拍位的 x 坐标内收**，让手臂从"缩回弯曲"到"伸展击球"，形成对角线挥拍轨迹。
这样 STRIKE_ACCEL 阶段大臂和小臂都有大幅度运动。

```
旧方案（纯垂直）：
  引拍 (0.400, -0.210) → 虚拟 (0.400, 0.040)     ← 同一 x，纯肘运动
  L1 行程: 0.036 rad (2°)   L2 行程: 0.615 rad (35°)

新方案（对角线）：
  引拍 (0.300, -0.200) → 虚拟 (0.400, 0.040)     ← x 也变化，肩肘协同
  L1 行程: 0.935 rad (54°)  L2 行程: 0.475 rad (27°)
```

### 7.2 IK 验证

```
引拍 (0.300, -0.200):
  d = √(0.09 + 0.04) = 0.361 < 0.424 ✓ (有余量)
  IK: q1 = -1.178 rad, q2 = 1.111 rad

虚拟穿透 (0.400, 0.040):
  d = √(0.16 + 0.0016) = 0.402 < 0.424 ✓
  IK: q1 = -0.243 rad, q2 = 0.636 rad

Home → 引拍 (WIND_UP 行程):
  L1: -1.571 → -1.178 = 0.393 rad (23°) ← 比旧的 1.188 rad 短很多！
  L2:  1.222 →  1.111 = 0.111 rad (6°)  ← 非常短

引拍 → 虚拟 (STRIKE_ACCEL 行程):
  L1: -1.178 → -0.243 = 0.935 rad (54°) ← 大行程！肩关节大幅挥动！
  L2:  1.111 →  0.636 = 0.475 rad (27°) ← 同时展开！
  比例 = 1.97 : 1 → 肩关节运动量甚至更大
```

### 7.3 笛卡尔轨迹估算

关节空间线性插值下，末端轨迹近似为：

```
t=0.0: (0.300, -0.200)      引拍位（缩回弯曲）
t=0.3: (0.346, -0.155)      斜向上伸展
t=0.5: (0.380, -0.115)      接近击球点
t=0.7: (0.400, -0.070)      穿过击球面
t=1.0: (0.400,  0.040)      虚拟穿透点
```

末端轨迹是一条**从左下到右上的弧线**，自然穿过击球点 (0.400, -0.110)。

### 7.4 方案优势

| 维度 | 旧方案（纯垂直） | 新方案（对角线） |
|------|----------------|----------------|
| WIND_UP 行程 | L1=1.188, L2=1.155 rad | L1=0.393, L2=0.111 rad |
| WIND_UP 时间 | ~260ms | **~80ms** (×3 加速) |
| STRIKE_ACCEL L1行程 | 0.036 rad (2°) | **0.935 rad (54°)** |
| STRIKE_ACCEL L2行程 | 0.615 rad (35°) | 0.475 rad (27°) |
| 两关节视觉协同 | ❌ 仅小臂动 | ✅ 大小臂同时大幅挥动 |
| 引拍位 IK 安全裕度 | d/Lmax = 0.94 | d/Lmax = **0.85** (更安全) |
| 虚拟穿透 IK 裕度 | d/Lmax = 0.95 | d/Lmax = 0.95 (相同) |

### 7.5 实现方案

**新增参数** (`volleyball_strike_params.yaml`):

```yaml
spatial_trigger:
  wind_up_x_offset: 0.10    # 引拍 x 回退距离 (m)，引拍x = impact_x - offset
```

**代码修改** (`strikeCommandCallback`):

```cpp
// Step 3：引拍位 x 内收
double wind_up_x = impact_x_ - wind_up_x_offset_;
double wind_up_y = impact_y_ - wind_up_backoff_y_;
ik_wind_up_ = solveIK(wind_up_x, wind_up_y);
```

仅需 1 个新参数 + 1 行代码修改。

### 7.6 风险评估

| 风险 | 严重性 | 缓解 |
|------|--------|------|
| 笛卡尔轨迹是弧线不是直线 | 低 | 排球拍面大，±2cm偏差可接受 |
| 引拍位 x 不同可能不经过击球 x | 中 | IMPACT 触发条件是 FK_Y 阈值，不检查 x |
| 引拍位离 home 更近 → backoff 不够 | 低 | 可用 wind_up_backoff_y 补偿 |

### 7.7 待确认

- [x] 用户是否接受"对角线挥拍"的运动风格 → 物理限位修复后测试通过，击球正常
- [x] `wind_up_x_offset` 的最优值 → 自适应降到 50%（x=0.350）后可用
- [x] IMPACT 的 FK_Y 触发是否仍然可靠 → ✅ 在 threshold=0.02m 下稳定触发

---

## 八、实机测试第二轮分析 (2026-04-10 16:34)

### 8.1 修复效果

| 修复项 | 效果 |
|--------|------|
| 引拍 IK 物理限位校验 | ✅ 100% 消除 WIND_UP 超时（L2 不再超过 1.222） |
| STRIKE_ACCEL 超时保护 | ✅ 加入 2s 保底，日志中未触发（说明正常流程够快） |
| x_offset 自适应降级 | ✅ 自动从 100% 降到 50%（x=0.350），所有目标可达 |

### 8.2 数据统计（target 0.400, -0.050）

| 指标 | 值 | 说明 |
|------|-----|------|
| WIND_UP 耗时 | **200ms** | 从 home 到引拍(0.350, -0.150) |
| STRIKE_ACCEL 耗时 | **130ms** | 从引拍到 IMPACT 触发 |
| IMPACT 耗时 | **25ms** | 力矩爆破窗口 |
| FOLLOW_THROUGH 耗时 | **285ms** | Kd ramp 刹车 |
| SETTLING 耗时 | **570ms** | 缓冲回 home |
| **击球总周期** | **~1210ms** | 330ms 攻击 + 880ms 恢复 |
| 到达 IMPACT 时速度 | [5.1, -2.6] rad/s | L1=5.1(肩), L2=-2.6(肘) |
| overshoot | **-0.033 ~ -0.071** | ❌ 始终为负，手臂在 IMPACT 后下落 |

### 8.3 新发现的问题

#### 问题 1：WIND_UP → STRIKE_ACCEL 转换处"停顿"

```
时间线：
  t=0ms:    手臂从 home 开始运动（WIND_UP 开始）
  t=150ms:  接近引拍位，轨迹规划器开始减速
  t=200ms:  到达引拍位，速度降至 ≈0 → 切入 STRIKE_ACCEL
  t=201ms:  轨迹目标切为虚拟穿透点，从零重新加速
  t=330ms:  到达 IMPACT 触发阈值

问题：t=150ms~200ms 的减速 + t=200ms~250ms 的重新加速
      → 50ms 的 "刹车-起步" 窗口，视觉上明显卡顿
```

#### 问题 2：STRIKE_ACCEL → IMPACT 转换处速度骤降

```
STRIKE_ACCEL 阶段：阻抗 = 高Kp (40 Nm/rad) + 低Kd (2 Nm·s/rad)
IMPACT_MOMENT 阶段：阻抗 = 极低Kp (2 Nm/rad) + 极低Kd (0.5 Nm·s/rad)

转换瞬间：Kp 从 40 骤降到 2 → 位置跟踪力消失
         重力 > tau_ff → 手臂开始自由落体
         25ms 后 FOLLOW_THROUGH → Kd ramp 开始刹车

结果：overshoot 始终为负（手臂下落 3-7cm）
```

#### 问题 3：总周期 1.2s，其中 73% 是恢复时间

```
攻击: 330ms (27%)
恢复: 880ms (73%)  ← 大部分时间在"回家"
```

---

## 九、方案：合并 WIND_UP + STRIKE_ACCEL 为单阶段连续挥拍

### 9.1 核心思路

**取消引拍目标，手臂从 home 位直接全速冲向虚拟穿透目标。**

```
旧状态机（6 个活跃状态）：
  READY → WIND_UP → STRIKE_ACCEL → IMPACT_MOMENT → FOLLOW_THROUGH → SETTLING → READY
  （引拍减速停→从零加速→力矩爆破→刹车→回位）

新状态机（5 个活跃状态）：
  READY → STRIKE → IMPACT_MOMENT → FOLLOW_THROUGH → SETTLING → READY
  （从home全速冲击→力矩爆破→刹车→回位）
```

### 9.2 运动学分析

#### 从 home 到虚拟穿透目标的行程

以 target (0.400, -0.050) 为例，virtual = (0.400, 0.100)（取50%过冲后）：

```
Home:    [L1=-1.571, L2=1.222]  → FK(x,y) = (0.059, -0.277)
Virtual: [L1≈-0.10,  L2≈0.52]  → FK(x,y) = (0.400, 0.100)

关节行程：
  L1: -1.571 → -0.10 = 1.471 rad (84°) ← 大臂大幅挥动
  L2:  1.222 →  0.52 = 0.702 rad (40°) ← 小臂同时展开
  比例: 2.10 : 1

对比旧方案（引拍 → 虚拟）：
  L1: -0.889 → -0.10 = 0.789 rad (45°)
  L2:  0.912 →  0.52 = 0.392 rad (22°)
  比例: 2.01 : 1
```

**新方案行程是旧方案的 1.86 倍**，但全程不停顿 → 到达击球点时速度更高。

#### 笛卡尔轨迹估算（关节空间线性插值）

```
t=0.0: ( 0.059, -0.277)  home（手臂朝下）
t=0.2: ( 0.166, -0.248)  手臂开始展开
t=0.4: ( 0.290, -0.186)  手臂向右前伸
t=0.5: ( 0.338, -0.142)  进入击球区域
t=0.6: ( 0.377, -0.088)  ← 穿过 target_y=-0.050 附近
t=0.7: ( 0.400, -0.023)  继续上升
t=1.0: ( 0.400,  0.100)  虚拟穿透点
```

末端从左下方画一条弧线扫向右上方，自然穿过击球区域。

#### 速度估算

使用同步轨迹规划器（scale: L1=1.0, L2=0.477）：

```
L1: max_v = 30 rad/s, max_a = 120 rad/s²
    1.471 rad 行程 → 梯形曲线 T ≈ 0.30s
    通过击球区时速度 ≈ 25 rad/s（接近峰值）

L2: max_v = 14.3 rad/s (30 × 0.477), max_a = 57.2 rad/s²
    0.702 rad 行程 → 梯形曲线 T ≈ 0.30s（与L1同步）
    通过击球区时速度 ≈ 12 rad/s

末端 vy ≈ 0.200×cos(-0.5)×25 + 0.224×cos(0.0)×(25-12)
        ≈ 4.39 + 2.91
        ≈ 7.3 m/s
```

**比旧方案的 4-6 rad/s 击球速度提升 ×3-5。**

### 9.3 实现细节

#### 代码修改清单

| 位置 | 修改 |
|------|------|
| `strikeCommandCallback` | 删除 Step 3（引拍IK）, traj_target 直接设为 virtual |
| `strikeCommandCallback` | 初始转移到 STRIKE_ACCEL（不经过 WIND_UP） |
| `getImpedanceParams` | WIND_UP case 改为 STRIKE_ACCEL 参数（或直接复用） |
| `controlLoop` | 保留 WIND_UP case 分发但不会被触发 |
| `handleWindUp` | 不再被调用，保留代码备用 |

#### 不需要修改的部分

- `handleStrikeAccel` — 已有 FK_Y 触发 + 2s 超时
- `handleImpactMoment` — 不变
- `handleFollowThrough` — 不变
- `handleSettling` — 不变
- `updateTrajectory` — 不变
- `computeTrajSyncScale` — 不变
- YAML 参数 — 引拍相关参数变为废弃（不影响运行）

#### 状态机修改伪代码

```cpp
void strikeCommandCallback(msg) {
    // Step 1: impact IK — 不变
    // Step 2: virtual IK — 不变
    // Step 3: [删除] 不再计算引拍IK

    // Step 4: 直接从 home 冲向虚拟穿透目标
    for (i : NUM_MOTORS) {
        traj_state_[i].position = current_positions_[i];
        traj_state_[i].velocity = 0.0;
    }
    traj_target_ = {ik_virtual_.q_L1, ik_virtual_.q_L2,
                     ik_virtual_.q_R1, ik_virtual_.q_R2};
    computeTrajSyncScale();  // home → virtual 全程同步

    transitionTo(StrikeState::STRIKE_ACCEL);  // 直接进入击球加速
    RCLCPP_INFO(logger, "READY → STRIKE | 目标 (%.3f, %.3f) | 虚拟穿透IK: [...]");
}
```

### 9.4 方案对比

| 维度 | 旧方案（WIND_UP + STRIKE_ACCEL） | 新方案（直接 STRIKE） |
|------|------|------|
| 状态数 | 8 | **7**（砍掉 WIND_UP） |
| 攻击耗时 | 330ms (200+130) | **~250ms** (全程不停) |
| 中途停顿 | ❌ 减速→加速 | ✅ **无停顿** |
| 击球速度 | ~5 rad/s (L1), ~3 rad/s (L2) | **~25 rad/s (L1), ~12 rad/s (L2)** |
| 末端速度 | ~2-3 m/s | **~7 m/s** |
| 加速距离 | 从引拍到虚拟 (0.79 + 0.39 rad) | 从home到虚拟 (**1.47 + 0.70 rad**) |
| 代码量 | strikeCallback 中3个IK + 双维搜索 | strikeCallback 中**2个IK** |
| IK 限位风险 | ❌ 引拍常超限 | ✅ home 是已知安全位 |

### 9.5 风险评估

| 风险 | 严重性 | 缓解 |
|------|--------|------|
| 末端轨迹不经过精确击球点 | 低 | IMPACT 由 FK_Y 阈值触发，不要求精确路径 |
| 击球时 FK_X 可能偏离目标 x | 中 | 拍面宽 35cm，±2cm 可接受；可后续加 FK_X 检查 |
| 长加速距离导致速度太高撞到东西 | 低 | velocity_spike_threshold 安全保护 + IMPACT 后刹车 |
| 没有"蓄力"视觉效果 | 低 | 打排球本来就是连续挥拍，不需要"蓄力-停-打" |
| IMPACT_MOMENT 的 tau_ff 方向问题 | 中 | 从home上来的运动方向与 tau_ff 一致（都是向上） |

### 9.6 待讨论

- [ ] 是否保留 WIND_UP 代码（注释保留 vs 彻底删除）
- [ ] STRIKE 阶段用 STRIKE_ACCEL 的阻抗参数还是单独设一套
- [ ] virtual_overshoot_y 是否需要调整（home→virtual 行程更长，可能不需要那么大的过冲）
- [ ] IMPACT_MOMENT 的 overshoot 始终为负 → 是否需要同步调整 IMPACT 参数
