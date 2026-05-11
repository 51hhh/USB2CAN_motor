# VolleyballStrikeManager WIND_UP 删除总结

> 2026-04-11

## 一、修改动机

实测验证：从 Home 位直接起步全速冲刺是最高效的击球路径，WIND_UP 引拍阶段是多余的中间环节：
1. **从 Home 直冲**有 ~0.4m 加速距离，足以在梯形速度曲线最高平顶区通过排球
2. WIND_UP 增加了一次"先移到引拍位→再加速到虚拟目标"的两段式延迟
3. 引拍 IK 自适应搜索逻辑复杂（双维遍历+物理限位校验），删除后代码大幅精简

## 二、状态机精简

### Before（8态）
```
HOMING → READY → WIND_UP → STRIKE_ACCEL → IMPACT_MOMENT → FOLLOW_THROUGH → SETTLING → READY
```

### After（7态）
```
HOMING → READY → STRIKE → IMPACT_MOMENT → FOLLOW_THROUGH → SETTLING → READY
```

## 三、修改清单

### volleyball_strike_manager.hpp
| 修改项 | 具体内容 |
|--------|---------|
| 枚举 | 删除 `WIND_UP`，重命名 `STRIKE_ACCEL` → `STRIKE` |
| strikeStateToString | 对应更新 |
| 函数声明 | 删除 `handleWindUp()`，重命名 `handleStrikeAccel()` → `handleStrike()` |
| 成员变量 | 删除 `ik_wind_up_`、`wind_up_kp/kd_shoulder/elbow_`、`wind_up_reach_tol_`、`wind_up_backoff_y_`、`wind_up_x_offset_` |

### volleyball_strike_manager.cpp
| 修改项 | 具体内容 |
|--------|---------|
| 构造函数 | 删除 wind_up 阻抗默认值和空间参数默认值 |
| loadConfig() | 删除 `imp["wind_up"]` 加载段；`imp["strike_accel"]` → `imp["strike"]`；删除 `sp["wind_up_*"]` |
| getImpedanceParams() | READY/SETTLING 统一使用 strike 参数（原来用 wind_up 参数）；删除 `WIND_UP` case |
| controlLoop() | 删除 `WIND_UP`/`STRIKE_ACCEL` 分支，改为 `STRIKE` |
| handleWindUp() | **整个函数删除**（~50行） |
| handleStrikeAccel() | 重命名为 `handleStrike()`，日志更新 |
| handleImpactMoment() | 退出条件从 `elapsed > timeout || overshoot > threshold` 改为纯空间穿透 `overshoot > threshold`，超时作为硬兜底 |
| strikeCommandCallback() | 删除 Step 3 引拍 IK 搜索（~40行），Step 4 改为直接设 virtual 为轨迹目标，状态切换到 `STRIKE` |

### volleyball_strike_params.yaml
| 修改项 | 具体内容 |
|--------|---------|
| impedance.wind_up | **整段删除** |
| impedance.strike_accel | 重命名为 `impedance.strike` |
| spatial_trigger | 删除 `wind_up_reach_tol`、`wind_up_backoff_y`、`wind_up_x_offset` |
| virtual_overshoot_y | 0.30 → 0.10（保持 10cm 过冲距离） |

### volleyball_strike_usage.md
- 状态机图更新为 5 步
- 参数速查表与实际 YAML 同步
- 删除 WIND_UP 和 IDLE 段，新增 SETTLING 段

## 四、IMPACT 退出逻辑变更

### Before
```
if (elapsed > timeout || overshoot > threshold) → FOLLOW_THROUGH
```
两个条件 OR 组合，超时和空间穿透平级。

### After
```
if (overshoot > threshold) → FOLLOW_THROUGH           // 主退出：纯空间穿透
if (elapsed > timeout)     → FOLLOW_THROUGH (WARN)    // 兜底：硬超时保护
```
空间穿透为主判定条件，超时仅作为安全兜底防止永远卡在 IMPACT。

## 五、代码行数变化

| 文件 | 修改前 | 修改后 | 差值 |
|------|--------|--------|------|
| .cpp | 1363 行 | ~1267 行 | **-96 行** |
| .hpp | 366 行 | 356 行 | **-10 行** |
| .yaml | ~160 行 | ~130 行 | **-30 行** |

## 六、编译结果

```
colcon build --packages-select motor_control_ros2
0 errors, 0 warnings ✅
```
