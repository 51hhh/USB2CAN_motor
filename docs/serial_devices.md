# 底盘串口设备绑定

底盘运行时有 3 个 USB 串口设备。不要在节点配置中直接使用 `/dev/ttyACM*`、`/dev/ttyUSB*` 或只凭厂商字符串判断用途；这些名字会随插拔顺序变化。

本项目按 USB 设备描述符里的 `idVendor:idProduct` 绑定。对当前这三类硬件来说，USB ID 是固件 / USB 芯片枚举出来的固定身份，不会因为插拔顺序、USB 口变化、重启后枚举顺序变化而改变。因此节点配置只使用 udev 生成的 `/dev/robocon_*` 固定链接。

## 设备表

| 固定设备名 | USB ID | `lsusb` 设备名 | 用途 | 使用方 |
| --- | --- | --- | --- | --- |
| `/dev/robocon_usb2can` | `2e88:4603` | `HDSC CDC Device` | USB 转 CAN，控制 DJI 3508 电机 | `motor_control_node` |
| `/dev/robocon_rc` | `0483:5740` | `STMicroelectronics Virtual COM Port` | STM32 转发 DJI 遥控器数据的 USB CDC 虚拟串口 | `rc_usb_control_node` |
| `/dev/robocon_odom` | `1a86:7522` | `QinHeng Electronics USB Serial` | 码盘 / 里程计串口数据 | `mcu_odom_bridge_node` |

当前实测正确对应关系：

- `Bus 001 Device 021: ID 2e88:4603 HDSC CDC Device` 是 USB2CAN，用于 DJI 3508 电机控制。
- `Bus 001 Device 010: ID 0483:5740 STMicroelectronics Virtual COM Port` 是 STM32 转发遥控器数据的虚拟串口。
- `Bus 001 Device 023: ID 1a86:7522 QinHeng Electronics USB Serial` 是码盘 / 里程计串口数据。

其中 `Bus 001 Device 021/010/023` 是本次枚举出来的临时编号，拔插或重启后可能变化；后面的 `ID 2e88:4603`、`ID 0483:5740`、`ID 1a86:7522` 是当前硬件的 USB VID/PID，作为 udev 绑定依据不会随枚举顺序变化。

当前配置文件已经使用上述固定设备名：

- `src/motor_control_ros2/config/motors.yaml`: `/dev/robocon_usb2can`
- `src/motor_control_ros2/config/rc_usb_control_params.yaml`: `/dev/robocon_rc`
- `src/wheel_imu_ekf/config/mcu_bridge.yaml`: `/dev/robocon_odom`

## 安装 udev 规则

在机器人上执行：

```bash
cd /home/nvidia/robocon/USB2CAN_motor
sudo cp config/udev/99-robocon-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后重新插拔三个 USB 设备，或重启机器人。

如果 `nvidia` 用户还不在 `dialout` 组，执行：

```bash
sudo usermod -aG dialout nvidia
```

执行后需要重新登录，或者重启机器人让用户组生效。

## 绑定方式说明

规则文件位于 `config/udev/99-robocon-serial.rules`：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e88", ATTRS{idProduct}=="4603", SYMLINK+="robocon_usb2can", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="robocon_rc", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", SYMLINK+="robocon_odom", MODE="0666", GROUP="dialout"
```

字段含义：

- `SUBSYSTEM=="tty"`: 只匹配 Linux 串口 tty 设备。
- `ATTRS{idVendor}` / `ATTRS{idProduct}`: 匹配 USB 设备 VID/PID。
- `SYMLINK+="robocon_*"`: 在 `/dev` 下生成稳定链接。
- `MODE="0666"`: 允许普通用户读写串口。
- `GROUP="dialout"`: 记录串口所属组，便于后续收紧权限。

udev 规则生成的是稳定符号链接，不是强行固定内核的 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。例如 USB2CAN 实际可能枚举为 `/dev/ttyACM0` 或 `/dev/ttyACM1`，但节点始终打开 `/dev/robocon_usb2can`。

## 检查命令

查看当前 USB ID：

```bash
lsusb
```

应能看到：

```text
ID 2e88:4603 HDSC CDC Device
ID 0483:5740 STMicroelectronics Virtual COM Port
ID 1a86:7522 QinHeng Electronics USB Serial
```

查看固定链接：

```bash
ls -l /dev/robocon_usb2can /dev/robocon_rc /dev/robocon_odom
```

输出示例：

```text
lrwxrwxrwx 1 root root ... /dev/robocon_usb2can -> ttyACM1
lrwxrwxrwx 1 root root ... /dev/robocon_rc -> ttyACM0
lrwxrwxrwx 1 root root ... /dev/robocon_odom -> ttyUSB0
```

确认每个固定链接对应的 USB ID：

```bash
udevadm info -q property -n /dev/robocon_usb2can | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_MODEL=|ID_VENDOR='
udevadm info -q property -n /dev/robocon_rc | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_MODEL=|ID_VENDOR='
udevadm info -q property -n /dev/robocon_odom | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_MODEL=|ID_VENDOR='
```

期望结果：

```text
/dev/robocon_usb2can: ID_VENDOR_ID=2e88, ID_MODEL_ID=4603
/dev/robocon_rc:     ID_VENDOR_ID=0483, ID_MODEL_ID=5740
/dev/robocon_odom:   ID_VENDOR_ID=1a86, ID_MODEL_ID=7522
```

查看底盘启动脚本识别状态：

```bash
./src/ROS_test/launch/start_chassis.sh
```

脚本启动前会打印：

```text
设备状态:
/dev/robocon_usb2can ...
/dev/robocon_rc ...
/dev/robocon_odom ...
```

## 故障排查

如果 `/dev/robocon_*` 不存在：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
lsusb
ls -l /dev/robocon_usb2can /dev/robocon_rc /dev/robocon_odom
```

如果 `lsusb` 能看到设备但链接仍不存在，检查对应设备的 udev 属性：

```bash
udevadm info -a -n /dev/ttyACM0 | grep -m1 -E 'idVendor|idProduct'
udevadm info -a -n /dev/ttyACM1 | grep -m1 -E 'idVendor|idProduct'
udevadm info -a -n /dev/ttyUSB0 | grep -m1 -E 'idVendor|idProduct'
```

如果以后接入第二个同 VID/PID 的设备，单靠 USB ID 会无法区分两者，需要在规则里额外加入序列号或物理 USB 口路径。例如先查询：

```bash
udevadm info -q property -n /dev/ttyACM0 | grep -E 'ID_SERIAL=|ID_PATH='
```

然后把规则收紧为 `ENV{ID_SERIAL}=="..."` 或 `ENV{ID_PATH}=="..."`。当前底盘这三类设备 VID/PID 互不相同，按 USB ID 绑定已经足够稳定。
