# motor_control_ros2

只保留底盘相关控制链路。

## 范围

- DJI GM3508 / GM6020
- USB2CAN 通信
- 手柄速度指令
- 全向轮底盘速度闭环
- 全向轮底盘位置闭环

不再包含：

- 达妙电机
- 宇树电机
- 机械臂
- 击球/发球
- 视觉跟踪
- 多源 `cmd_vel` 复用

## 节点

### `motor_control_node`

- 读取 `motors.yaml`
- 建立 USB2CAN 通道
- 管理 DJI 电机
- 执行电机位置/速度 PID
- 发布 `/dji_motor_states`、`/control_frequency`

### `omni_chassis_control_node`

- 速度模式：订阅 `/cmd_vel`，结合 `/odom.twist` 做底盘速度闭环
- 位置模式：订阅 `/cmd_pose`，结合 `/odom.pose` 做底盘位置闭环
- 输出 `/dji_motor_command_advanced`

### `joystick_control_node`

- 订阅 `/joy`
- 输出 `/cmd_vel`

## 配置

- `config/motors.yaml`: CAN 电机定义
- `config/pid_params.yaml`: 电机 PID
- `config/joystick_params.yaml`: 手柄参数
- `config/omni_chassis_params.yaml`: 底盘参数、速度环 PID、位置环 PID
