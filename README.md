# Robot_1A 底盘控制仓库

这个分支只保留底盘最小闭环：

- `motor_control_ros2`: DJI 电机控制层 + 底盘控制层 + USB 遥控器控制层
- `wheel_imu_ekf`: MCU `ODOM_STATE` 串口桥接层，发布 `/odom` 和 TF
- `rviz_bag_tools`: rosbag 录制、交互回放和 RViz 查看工具，不参与底盘控制

## 当前控制结构

```text
/dev/robocon_rc
  -> rc_usb_control_node
  -> /cmd_vel
  -> omni_chassis_control_node
     - 速度模式: /cmd_vel + /odom.twist 闭环
     - 位置模式: /cmd_pose + /odom.pose 闭环
  -> /dji_motor_command_advanced
  -> motor_control_node
  -> /dev/robocon_usb2can
```

`/odom` 只由 `mcu_odom_bridge_node` 发布。

## 主要节点

- `motor_control_ros2/motor_control_node`
- `motor_control_ros2/omni_chassis_control_node`
- `motor_control_ros2/rc_usb_control_node`
- `wheel_imu_ekf/mcu_odom_bridge_node`

## 启动

编译：

```bash
colcon build --packages-select motor_control_ros2 wheel_imu_ekf
source install/setup.bash
```

需要 rosbag/RViz 工具时额外构建：

```bash
colcon build --packages-select rviz_bag_tools
source install/setup.bash
```

底盘启动脚本：

```bash
./src/ROS_test/launch/start_chassis.sh
```

默认只启动 4 个核心节点：`motor_control_node`、`omni_chassis_control_node`、`mcu_odom_bridge_node`、`rc_usb_control_node`。

遥控器控制：

- 左拨杆上档: 手动遥控，发布 `/cmd_vel` 并解除 `/chassis/estop`
- 左拨杆中档: 硬急停，发布 `/chassis/estop=true`，底盘电机直接给 0 速度
- 左拨杆下档: 视觉速度档，转发 `/vision/cmd_vel`，无新视觉速度时零速
- 左摇杆左右 `lx`: `/cmd_vel.linear.x`，当前开环底盘右/左平移
- 左摇杆前后 `ly`: `/cmd_vel.linear.y`，当前开环底盘前/后平移
- 右摇杆 X: 旋转速度

当前 USB 转发帧只包含左右摇杆和左右三档拨杆，不包含旧手柄的 A/B/X/Y/Start 独立按钮状态，因此遥控节点按 DJI 左拨杆三档完成手动/急停/视觉切换。

USB2CAN、遥控器 CDC、MCU 里程计串口都通过 udev 规则绑定到固定设备名，避免 USB 枚举顺序变化。设备对应关系见 [docs/serial_devices.md](docs/serial_devices.md)。

| 固定设备名 | USB ID | 用途 |
| --- | --- | --- |
| `/dev/robocon_usb2can` | `2e88:4603` | HDSC USB2CAN，控制 DJI 3508 |
| `/dev/robocon_rc` | `0483:5740` | STM32 USB CDC，转发 DJI 遥控器数据 |
| `/dev/robocon_odom` | `1a86:7522` | QinHeng USB 串口，码盘 / 里程计数据 |

RViz 默认不启动。需要查看 `/odom` 和 `odom -> base_link` TF 时可额外开启：

```bash
START_RVIZ=1 ./src/ROS_test/launch/start_chassis.sh
```

## 关键话题

- `/cmd_vel`: 底盘速度目标
- `/cmd_pose`: 底盘位置目标，`geometry_msgs/Pose2D`
- `/odom`: 码盘位姿与速度反馈
- `/dji_motor_command_advanced`: 底层电机速度命令
- `/dji_motor_states`: 电机状态
- `/control_frequency`: 电机控制频率统计

## Rosbag/RViz 调试工具

`rviz_bag_tools` 支持两种模式：

- 实时查看：只启动 RViz，读取当前 DDS 网络中的 `/odom` 和 TF。
- 离线回放：读取下载到本机的 rosbag，默认发布 `/replay/odom`、`/replay/path` 和 `replay_odom -> replay_base_link`，不与真实底盘话题冲突。
- 远程安全查看：可启用 live adapter，把远程 odom 转成 `/remote_view/odom` 和 `remote_odom -> remote_base_link`。

配置文件位于 `src/rviz_bag_tools/config/chassis_bag.yaml`。
