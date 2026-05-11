# 定位融合·方案 3：robot_localization EKF（归档·延后）

> 状态：**未实施**，等方案 1（互补滤波）跑通并积累数据后再决定是否升级。
> 目的：留作里程碑 B 的实施蓝本，避免后续重复推导。

## 1. 何时启用本方案

满足下列任一条件后再考虑升级到 EKF：

- 互补滤波运行 ≥ 1 周，掌握各源残差的实际统计分布
- 即将接入 nav2 / SLAM，需要标准 `/odometry/filtered` 接口
- 多传感器（含独立 IMU 主题）需要统一融合
- 互补滤波的阈值切换造成可见跳变，需要数学连续解

## 2. 适用场景与不适用场景

✅ 适用：长期稳定运行、多源（≥3 路）传感器、需要协方差作为下游输入
❌ 不适用：调试期方案频繁迭代、协方差完全无经验、源 ≤ 2 路

## 3. 依赖

```bash
sudo apt install ros-humble-robot-localization
```

工程仓库 `package.xml` 增加：
```xml
<exec_depend>robot_localization</exec_depend>
```

## 4. 系统架构

```
                    /cmd_vel ──► omni_chassis_control_node ──► /dji_motor_command_advanced
                                          │
                                          └──► /odom_wheels      (twist 高质量, pose 辅助)

   STM32码盘 ──► positioning_bridge_node ──► /odom_encoder        (pose 高质量, yaw 高质量)
                                          └──► /imu/data_raw      (可选, MCU 加发 IMU 主题)

                    /odom_wheels ──┐
                    /odom_encoder ─┼──► ekf_node (robot_localization) ──► /odometry/filtered
                    /imu/data_raw ─┘                                    └──► /tf odom→base_link

                    /odometry/filtered ──► vision_catch / nav2 / etc.
```

> 注：positioning_bridge 不再发 odom→base_link TF（移交给 ekf_node）。

## 5. EKF 状态向量（15 维）

```
[ x,  y,  z,        位置  (m, world frame)
  roll, pitch, yaw, 姿态  (rad)
  vx, vy, vz,       线速度 (m/s, body frame)
  vroll, vpitch, vyaw, 角速度 (rad/s, body frame)
  ax, ay, az ]      加速度 (m/s², body frame)
```

平面机器人只关心 `x, y, yaw, vx, vy, vyaw`，其它维度可在 yaml 中关闭/降权。

## 6. 配置 yaml 范本

文件：`src/positioning_bridge_ros2/config/ekf_fusion_params.yaml`

```yaml
ekf_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true                  # 平面 2D 模式 (z, roll, pitch 锁 0)
    transform_time_offset: 0.0
    transform_timeout: 0.0
    print_diagnostics: true
    debug: false

    # 帧定义
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # ────────── 源 1: 电机 odom (twist) ──────────
    odom0: /odom_wheels
    odom0_config: [false, false, false,    # x, y, z       ← 不用电机 pose
                   false, false, false,    # roll, pitch, yaw
                   true,  true,  false,    # vx, vy, vz    ← 用电机线速度
                   false, false, true,     # vroll, vpitch, vyaw ← 用电机角速度
                   false, false, false]    # ax, ay, az
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10
    odom0_nodelay: true

    # ────────── 源 2: 码盘 odom (pose + yaw) ──────────
    odom1: /odom_encoder
    odom1_config: [true,  true,  false,    # x, y, z       ← 用码盘位置
                   false, false, true,     # roll, pitch, yaw ← 用码盘 yaw
                   false, false, false,
                   false, false, false,
                   false, false, false]
    odom1_differential: false
    odom1_relative: false
    odom1_queue_size: 10
    odom1_nodelay: true

    # ────────── 源 3: IMU (可选) ──────────
    # imu0: /imu/data_raw
    # imu0_config: [false, false, false,
    #               false, false, false,
    #               false, false, false,
    #               false, false, true,    # vyaw ← 陀螺仪 z
    #               true,  true,  false]   # ax, ay
    # imu0_differential: false
    # imu0_remove_gravitational_acceleration: true

    # ────────── 过程噪声协方差 (15x15, 行主) ──────────
    # 越小表示越信任运动模型预测
    # 调参起点: 参考 robot_localization 默认值, 先用 1e-3 ~ 1e-2 试
    process_noise_covariance: [
      0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0.04, 0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.02, 0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.015]

    # ────────── 初始估计协方差 ──────────
    initial_estimate_covariance: [
      1e-9, ...]
```

实际协方差值由方案 1 阶段采集的残差统计反推：
```
σ²_meas ≈ Var(residual)   (各源各分量分别算)
```

## 7. 上位机改动

1. `omni_chassis_control_node`：把 `/odom` 改名 `/odom_wheels`
2. `positioning_bridge_node`：
   - 把 `/odom` 改名 `/odom_encoder`
   - **关闭 `publish_tf`**（移交 ekf_node）
3. 新增 launch 文件 `positioning_fusion.launch.py`：启动 ekf_node + 加载 yaml
4. `vision_catch_controller_node`：参数 `odom_topic` 改 `/odometry/filtered`

## 8. 调参步骤

```
[1] 先跑方案 1 互补滤波 ≥ 3 天, 收集 rosbag (静止/直行/横推/旋转)
[2] 用 rosbag 离线计算各源残差统计:
       python3 -c "import rosbag2_py; ..." 
       得到 σ_x, σ_y, σ_yaw, σ_vx, σ_vy, σ_vyaw
[3] 把 σ² 填入对应源的 measurement_covariance (R 矩阵)
[4] process_noise_covariance (Q 矩阵) 从默认值 ±一个数量级试
[5] 启动 ekf_node + 真实场景跑, 看 /diagnostics 是否报"测量被拒绝"
[6] 若拒绝率 > 5% → R 矩阵该源调大
[7] 若融合输出抖 → Q 矩阵调小
[8] 用 plotjuggler 画 (融合输出 vs 各源) 三轴对比图
```

## 9. 失败回退

ekf_node yaml 改 `frequency: 0.0` 即可关闭，下游切回 `/odom_encoder`。

## 10. 风险点

| 风险 | 缓解 |
|---|---|
| 协方差盲调越调越差 | 必须先跑方案 1 拿数据 |
| 源时间戳不对齐 → 融合错乱 | 确保 time_sync_locked, 各源 stamp 用 ROS time |
| odom→base_link TF 双发布 | 关掉 positioning_bridge 的 publish_tf |
| 启动顺序敏感 (ekf 先于源启动会报 "no measurement") | launch 文件加 `Node(..., respawn=True)` |
| ekf 频率 50Hz 高于源频率 → 重复预测放大不确定度 | frequency 不超过最快源的频率 |

## 11. 与 nav2 集成

```
ekf_node → /odometry/filtered + /tf odom→base_link
                                 ↓
                  nav2_amcl (融合激光) → /tf map→odom
                                 ↓
                          nav2 controller / planner
```

切换到 nav2 时只需：
- 加 amcl 提供 map→odom
- nav2 controller 订阅 `/odometry/filtered`
- 不需要重新设计 EKF

## 12. 与方案 1 的接口兼容

为了无缝切换：
- 方案 1 的 `/odom` topic 名最终也要改成 `/odom_encoder`（融合输出占用 `/odom`）
- vision_catch 改用 `/odometry/filtered`（先在方案 1 阶段把这个改动准备好）

---

**下一步**：先实施 [docs/positioning_fusion_plan_complementary.md](positioning_fusion_plan_complementary.md)（方案 1），跑通后再回看本文档。
