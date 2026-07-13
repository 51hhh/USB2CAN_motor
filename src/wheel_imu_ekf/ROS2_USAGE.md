# MCU ODOM Bridge

这个包只保留一个 ROS2 节点：

- `mcu_odom_bridge_node`: 串口接收 MCU `ODOM_STATE`，发布 `/odom` 和 `odom -> base_link` TF

MCU 已完成码盘和 IMU yaw 的坐标转换，上位机只做协议解析和 ROS2 桥接。

## Build

```bash
colcon build --packages-select wheel_imu_ekf
source install/setup.bash
```

## Run

```bash
ros2 launch wheel_imu_ekf mcu_bridge.launch.py serial_port:=/dev/ttyUSB0
```

等价的直接运行方式：

```bash
ros2 run wheel_imu_ekf mcu_odom_bridge_node --ros-args \
  --params-file src/wheel_imu_ekf/config/mcu_bridge.yaml
```

## Topics

- `/odom` (`nav_msgs/Odometry`)
- `/tf` (`odom -> base_link`)

## Parameters

- `serial_port`: 默认 `/dev/ttyUSB0`
- `baud_rate`: 默认 `115200`
- `odom_topic`: 默认 `/odom`
- `odom_frame`: 默认 `odom`
- `base_frame`: 默认 `base_link`
- `publish_tf`: 默认 `true`

## Check

```bash
ros2 topic echo /odom
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
```

## RViz

```bash
rviz2 -d src/wheel_imu_ekf/rviz/mcu_odom.rviz
```
