# rviz_bag_tools

`rviz_bag_tools` 是 Robot_1A 的 rosbag/RViz 调试工具包。它不参与底盘控制，只负责：

- 按 YAML 配置录制多个 ROS2 话题到 rosbag。
- 在本机交互式回放 rosbag，并把话题发布到 `/replay/*`。
- 启动 RViz 查看实时话题或回放话题。

默认回放不会发布 `/odom`、`/cmd_vel`，避免和真实底盘节点冲突。

## 构建

```bash
colcon build --packages-select rviz_bag_tools
source install/setup.bash
```

如果和底盘一起构建：

```bash
colcon build --packages-select motor_control_ros2 wheel_imu_ekf rviz_bag_tools
source install/setup.bash
```

## 录制

默认配置文件：

```text
src/rviz_bag_tools/config/chassis_bag.yaml
```

默认录制：

```bash
ros2 run rviz_bag_tools bag_record.py
```

指定输出目录：

```bash
ros2 run rviz_bag_tools bag_record.py \
  --config src/rviz_bag_tools/config/chassis_bag.yaml \
  --output bags/chassis_test_001
```

限定录制时间：

```bash
ros2 run rviz_bag_tools bag_record.py --duration 30 --name chassis_30s
```

录制话题在 `record.topics` 下配置。rosbag2 会保存不同频率话题的消息时间序列，回放时按 bag 时间恢复相对时序。

## 离线回放 + RViz

```bash
ros2 run rviz_bag_tools start_replay_rviz.sh bags/chassis_test_001
```

或者直接运行播放器：

```bash
ros2 run rviz_bag_tools interactive_bag_player.py \
  --bag bags/chassis_test_001 \
  --config src/rviz_bag_tools/config/chassis_bag.yaml
```

键盘控制：

```text
Space  暂停/继续
d/→    前进 1 步
a/←    后退 1 步
w/.    前进 10 步
s/,    后退 10 步
+/-    调整速度
r      回到起点
q      退出
```

步进模式由 `replay.step_mode` 控制：

```yaml
replay:
  step_mode: "topic"        # message | topic | time
  step_topic: "/odom"
  step_time_sec: 1.0
```

默认按 `/odom` 步进，比按混合消息条数更适合看底盘轨迹。播放器启动时只建立轻量时间索引，消息内容在播放/跳转时按需读取。

默认回放映射：

```text
/odom              -> /replay/odom
/cmd_vel           -> /replay/cmd_vel
/dji_motor_states  -> /replay/dji_motor_states
/control_frequency -> /replay/control_frequency
```

`/replay/path` 和 `replay_odom -> replay_base_link` TF 由 `/replay/odom` 自动生成。

## 实时 RViz

实时模式默认只启动 RViz，不发布任何话题：

```bash
ros2 run rviz_bag_tools start_live_rviz.sh
```

DDS/跨主机环境变量在这里配置：

```text
src/rviz_bag_tools/config/live_remote.yaml
```

如果远程机器和本机处于同一 `ROS_DOMAIN_ID` 且 DDS 网络可达，RViz 可以直接看远程 `/odom` 和 `/tf`。如果远程和本机是不同 `ROS_DOMAIN_ID`，需要额外使用 domain bridge 或把远程 bag 下载到本机回放。

如果桥接后的实时话题需要避免 frame/topic 冲突，可以在 `live_remote.yaml` 中启用 adapter：

```yaml
live:
  rviz_config: "rviz/live_remote_view.rviz"
  adapter:
    enabled: true
    input_odom: "/odom"
    output_odom: "/remote_view/odom"
    path_topic: "/remote_view/path"
    frame_id: "remote_odom"
    child_frame_id: "remote_base_link"
    qos_reliability: "reliable"  # reliable | best_effort
```

adapter 只订阅 odom 并发布 RViz 专用话题，不向 `/cmd_vel` 或底盘控制链路发布数据。
`start_live_rviz.sh` 退出时会清理它启动的 adapter 进程。

## 安全约定

- 回放默认只发布 `/replay/*` 和 `replay_*` TF frame。
- 实时 adapter 默认只发布 `/remote_view/*` 和 `remote_*` TF frame。
- 不要在底盘真实运行时把回放目标改成 `/odom` 或 `/cmd_vel`。
- 如果需要完全模拟原始话题，先停止真实发布者，再修改 `replay.topics[].target`。
