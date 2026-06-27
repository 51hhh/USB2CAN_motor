# Robot_1A 底盘控制仓库

这个分支只保留底盘最小闭环：

- `motor_control_ros2`: DJI 电机控制层 + 底盘控制层 + 手柄遥控层
- `wheel_imu_ekf`: MCU `ODOM_STATE` 串口桥接层，发布 `/odom` 和 TF

## 当前控制结构

```text
/joy
  -> joystick_control_node
  -> /cmd_vel
  -> omni_chassis_control_node
     - 速度模式: /cmd_vel + /odom.twist 闭环
     - 位置模式: /cmd_pose + /odom.pose 闭环
  -> /dji_motor_command_advanced
  -> motor_control_node
  -> USB2CAN
```

`/odom` 只由 `mcu_odom_bridge_node` 发布。

## 主要节点

- `motor_control_ros2/motor_control_node`
- `motor_control_ros2/omni_chassis_control_node`
- `motor_control_ros2/joystick_control_node`
- `wheel_imu_ekf/mcu_odom_bridge_node`

## 启动

编译：

```bash
colcon build --packages-select motor_control_ros2 wheel_imu_ekf
source install/setup.bash
```

底盘启动脚本：

```bash
./src/ROS_test/launch/start_chassis.sh
```

默认会启动 `motor_control_node`、`omni_chassis_control_node`、`mcu_odom_bridge_node`、`joy_node`、`joystick_control_node` 和 RViz。

手柄控制：

- Start: 使能/禁用底盘控制
- B: 急停/解除急停，解除后需要再次按 Start 才能重新使能
- 左摇杆: 前后/左右速度
- 右摇杆 X: 旋转速度

RViz 默认读取 `src/wheel_imu_ekf/rviz/mcu_odom.rviz`，显示 `/odom` 和 `odom -> base_link` TF。无桌面环境时可关闭 RViz：

```bash
START_RVIZ=0 ./src/ROS_test/launch/start_chassis.sh
```

## 关键话题

- `/cmd_vel`: 底盘速度目标
- `/cmd_pose`: 底盘位置目标，`geometry_msgs/Pose2D`
- `/odom`: 码盘位姿与速度反馈
- `/dji_motor_command_advanced`: 底层电机速度命令
- `/dji_motor_states`: 电机状态
- `/control_frequency`: 电机控制频率统计
