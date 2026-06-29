# motor_control_ros2

只保留底盘相关控制链路。

## 范围

- DJI GM3508 / GM6020
- USB2CAN 通信
- USB 遥控器速度指令
- 全向轮底盘速度闭环
- 全向轮底盘位置闭环

不再包含：

- 达妙电机
- 宇树电机
- 机械臂
- 击球/发球
- 独立视觉跟踪节点
- 独立多源 `cmd_vel` 复用节点

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

### `rc_usb_control_node`

- 读取 RC USB CDC 二进制帧，默认设备使用 `/dev/robocon_rc` 稳定路径
- 校验 `0xA55A` 帧头、协议版本、固定 24 字节长度和 CRC16/Modbus
- 遥控帧轴定义：`lx` 为左摇杆左右，`ly` 为左摇杆前后，`rx` 为右摇杆左右
- 当前开环底盘映射：`lx -> linear.x` 右/左平移，`ly -> linear.y` 前/后平移，`rx -> angular.z` 旋转
- 左拨杆上档：手动遥控，输出 `/cmd_vel` 并解除 `/chassis/estop`
- 左拨杆中档：硬急停，发布 `/chassis/estop=true`，底盘电机直接给 0 速度
- 左拨杆下档：视觉速度档，转发 `/vision/cmd_vel`，视觉速度超时时零速

当前固件转发帧只包含 `lx/ly/rx/ry/sw_left/sw_right`，没有旧手柄 A/B/X/Y/Start 独立按钮状态，因此不再保留旧 joystick 按键逻辑。

遥控字段当前定义：

| 字段 | 物理输入 | 当前用途 |
| --- | --- | --- |
| `lx` | 左摇杆左右，右推为正 | `/cmd_vel.linear.x`，右/左平移 |
| `ly` | 左摇杆前后，前推为正 | `/cmd_vel.linear.y`，前/后平移 |
| `rx` | 右摇杆左右，右推为正 | `/cmd_vel.angular.z`，旋转 |
| `ry` | 右摇杆前后 | 当前未使用 |
| `sw_left` | 左三档拨杆，`UP=1 MID=3 DOWN=2` | 手动 / 硬急停 / 视觉模式切换 |
| `sw_right` | 右三档拨杆，`UP=1 MID=3 DOWN=2` | 当前未使用 |

USB2CAN、遥控 CDC、MCU 里程计串口不应共用同一设备。当前设备绑定为：USB2CAN 使用 `/dev/robocon_usb2can` (`2e88:4603`)，遥控器 STM32 CDC 使用 `/dev/robocon_rc` (`0483:5740`)，码盘串口使用 `/dev/robocon_odom` (`1a86:7522`)。拨杆值按 DJI DBUS 宏定义处理：`UP=1`、`MID=3`、`DOWN=2`。当前使用物理左拨杆，对应帧内 `sw_left`，因此默认 `mode_switch_field: "left"`。

## 配置

- `config/motors.yaml`: CAN 电机定义
- `config/pid_params.yaml`: 电机 PID
- `config/rc_usb_control_params.yaml`: USB 遥控参数
- `config/omni_chassis_params.yaml`: 底盘参数、速度环 PID、位置环 PID
