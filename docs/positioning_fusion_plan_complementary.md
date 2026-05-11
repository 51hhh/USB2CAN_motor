# 定位融合·方案 1：互补滤波（Complementary Filter）

> 状态：**实施中**，对应 [docs/positioning_fusion_plan_ekf.md](positioning_fusion_plan_ekf.md) 是延后里程碑。
> 目标：消除"被踹一脚回不去"，减小往返累积误差，保持直行平滑。

## 1. 设计原则

```
真值 = 码盘 pose (来自 STM32 ODOM_STATE: AS5048 从动轮 + IMU)
预测 = pose_fused_prev + motor_twist × dt (来自 omni_chassis_control)

融合 = α × 预测 + (1-α) × 真值
       ↑ 小                 ↑ 大权重
       (作平滑插值器)        (锚定不漂)
```

实测依据（用户经验）：
- 码盘的 x, y 比电机正运动学准（从动轮无打滑）
- 码盘的 yaw 是绝对真值，电机 wz 提供高质量短期变化率

## 2. 系统拓扑

```
                  /cmd_vel ──► omni_chassis_control_node ──► 4×GM3508
                                       │
                                       └──► /odom_wheels  (twist 主, 100Hz)
                                                  │
   STM32码盘 ──► positioning_bridge_node ◄────────┘
                       │ (内部互补滤波)
                       ├──► /odom (融合后, 100Hz)
                       └──► /tf odom→base_link

                      /odom ──► vision_catch_controller_node 等下游
```

## 3. 算法流程

每次收到 ODOM_STATE 帧触发：

```cpp
// Step 1: 用 motor_twist 从 pose_fused_prev 预测到当前帧时间
double dt = (t_now - t_fused_prev).seconds();
if (dt > 0 && dt < 0.5 && motor_twist 新鲜) {
    double cs = cos(pose_fused.yaw), sn = sin(pose_fused.yaw);
    double vx_w = twist.vx * cs - twist.vy * sn;
    double vy_w = twist.vx * sn + twist.vy * cs;
    pred.x   = pose_fused.x   + vx_w * dt;
    pred.y   = pose_fused.y   + vy_w * dt;
    pred.yaw = wrap(pose_fused.yaw + twist.wz * dt);
} else {
    pred = pose_fused;  // 退化: 跳过预测
}

// Step 2: 残差
e_xy  = hypot(pred.x - meas.x, pred.y - meas.y);
e_yaw = abs(angleDiff(pred.yaw, meas.yaw));

// Step 3: 自适应 alpha
if      (e_xy < TH_POS_NORMAL && e_yaw < TH_YAW_NORMAL) α = α_normal;
else if (e_xy < TH_POS_ALERT  && e_yaw < TH_YAW_ALERT)  α = α_alert;
else                                                     α = α_disturbed;

// Step 4: 加权融合 (yaw 走差值法避开 ±π wrap)
pose_fused.x   = α * pred.x + (1-α) * meas.x;
pose_fused.y   = α * pred.y + (1-α) * meas.y;
pose_fused.yaw = wrap(meas.yaw + α * angleDiff(pred.yaw, meas.yaw));
```

## 4. 参数

```yaml
positioning_bridge_node:
  ros__parameters:
    # ── 融合开关与源 ──
    fusion_mode: "complementary"      # none | complementary
    motor_twist_topic: "/odom_wheels"
    motor_twist_timeout_sec: 0.3      # 超时退化 α=0 (纯码盘)

    # ── 残差阈值 (xy: m, yaw: rad) ──
    fusion_threshold_pos_normal:  0.05    # 5cm 内视为正常
    fusion_threshold_pos_alert:   0.20    # 20cm 内视为中度偏差
    fusion_threshold_yaw_normal:  0.05    # 2.9°
    fusion_threshold_yaw_alert:   0.30    # 17°

    # ── 自适应权重 ──
    fusion_alpha_normal:    0.30    # 30% 电机预测 + 70% 码盘 (平时)
    fusion_alpha_alert:     0.15    # 偏差中等, 多信码盘
    fusion_alpha_disturbed: 0.02    # 几乎全码盘 (被踹时)
```

## 5. 边界条件

| 场景 | 处理 |
|---|---|
| `fusion_mode == none` | 跳过融合，直接发 pose_meas（向下兼容） |
| 启动后未收到 motor_twist | 自动降级 α=0（纯码盘），故障保护 |
| motor_twist 超时 | 同上 |
| dt ≤ 0 或 > 0.5s | 跳过预测，pose_fused = pose_meas |
| 初次预测（pose_fused 未初始化） | pose_fused = pose_meas |
| reset_local_origin 触发 | pose_fused 也归零 |

## 6. 诊断输出（/diagnostics）

```
fusion_mode               complementary
fusion_state              NORMAL | ALERT | DISTURBED
fusion_alpha              0.30
fusion_residual_xy_m      0.012
fusion_residual_yaw_rad   0.003
motor_twist_age_sec       0.012
fusion_state_changes      3
```

## 7. 测试用例

| 场景 | α 应稳定在 | pose_fused 行为 |
|---|---|---|
| 静止 | NORMAL (0.30) | 与 pose_meas 几乎相等 |
| 直行 1m | NORMAL | 平滑积分，与电机/码盘均一致 |
| 横推 0.3m | DISTURBED (0.02) | 立刻贴近 pose_meas，闭环自动反向回 goal |
| 慢推 0.05m | ALERT (0.15) | 平稳过渡，无突跳 |
| 原地转 360° | NORMAL | 跟 pose_meas 一致（前提 IMU 不漂） |

## 8. 改动文件

| 文件 | 改动 |
|---|---|
| `motor_control_ros2/src/omni_chassis_control_node.cpp` | `/odom` → `/odom_wheels`，移除 publish_odom |
| `motor_control_ros2/config/omni_chassis_params.yaml` | 删 publish_odom |
| `positioning_bridge_ros2/include/.../positioning_bridge_node.hpp` | 加 fusion 成员 |
| `positioning_bridge_ros2/src/positioning_bridge_node.cpp` | 实现融合 + motor_twist 订阅 |
| `positioning_bridge_ros2/config/positioning_bridge_params.yaml` | 加 fusion 参数 |

## 9. 调参指引

1. 跑 `fusion_mode: none` 一遍做 baseline
2. 切 `complementary`，默认参数跑
3. 看 `/diagnostics` 实时数据：
   - 静止 / 直行时 `fusion_state` 应 99% 在 NORMAL
   - 故意推车，观察 fusion_state 切换日志（WARN）
   - residual_xy 平时分布应 < 0.01m，否则 TH_POS_NORMAL 调大
4. 现场推车测试：
   - 推 5cm → 应在 ALERT
   - 推 30cm → 应在 DISTURBED 并秒切到码盘
5. 如果 vision_catch 回归 goal 仍偏，提高 α_disturbed 上限到 0 (即纯码盘)

## 10. 与方案 3 (EKF) 的对接

- `/odom` topic 名保持不变（融合输出占位）
- 升级 EKF 时：
  - 把 positioning_bridge 的 fusion_mode 设回 none
  - `/odom` 改名 `/odom_encoder`
  - 启 ekf_node 输出 `/odometry/filtered`
  - 详见 [positioning_fusion_plan_ekf.md](positioning_fusion_plan_ekf.md)
