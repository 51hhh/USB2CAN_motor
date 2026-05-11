# positioning_bridge 启动自动归零方案

> 创建时间: 2026-05-08
> 涉及包: `positioning_bridge_ros2`
> 涉及节点: `positioning_bridge_node`

## 1. 背景

STM32 码盘板上电后，会以自身上电时刻的位姿作为零点持续累计位姿。为了让 ROS 端 `/odom` 在每次桥接节点启动时拥有一致的初始位姿（`x=0, y=0, yaw=0`），需要由桥接节点向 MCU 发送 `SET_LOCAL_ORIGIN(0,0,0)` 指令，把"当前位置"重新标记为新原点。

## 2. 协议层（已存在）

| 项 | 值 |
|---|---|
| 帧头 | `0xAA 0x55` |
| msg_type | `0x30` (SET_LOCAL_ORIGIN) |
| ACK msg_type | `0x31` (SET_LOCAL_ORIGIN_ACK) |
| payload | `float32 x, float32 y, float32 yaw, uint8 flags, uint8[3] reserved` (16 字节) |
| CRC | CRC16-CCITT，覆盖 `version..payload` |

构造函数: [`buildSetLocalOriginFrame`](../src/positioning_bridge_ros2/src/protocol_decoder.cpp)

## 3. ROS2 接口

### 3.1 已有 service（手动触发）

```bash
ros2 service call /positioning/reset_local_origin std_srvs/srv/Trigger
```

返回 `success: bool, message: string`，适合调试与比赛中按键复位。

### 3.2 新增：启动自动归零（B 策略）

节点启动后，**等待时间同步锁定**再发一次 `SetLocalOrigin(0,0,0)`，一次性完成，不再重发。

#### 触发条件（**任一满足即发送**）

1. `time_sync_estimator_.locked() == true` （正常路径）
2. 距节点启动已超过 `auto_reset_origin_timeout_sec`（兜底，防止时间同步永不锁定时阻塞）

#### 前置条件

- `protocol_mode == binary_v1`（legacy_ascii 不支持归零指令）
- `serial_.isOpen() == true`

#### 失败处理

- 串口写失败：日志 WARN，下次 `diagnosticsTick` 仍会重试，直到成功或节点关闭
- MCU 不回 ACK：当前不做强校验（service 也不校验 ACK），只看是否成功写到串口；如未来需要，可在 `handleFrame` 接收 `SET_LOCAL_ORIGIN_ACK` 后置位 `origin_reset_acked_` 标志

## 4. 配置参数

[positioning_bridge_params.yaml](../src/positioning_bridge_ros2/config/positioning_bridge_params.yaml)

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `auto_reset_origin_on_start` | bool | `true` | 启动时是否自动归零 |
| `auto_reset_origin_timeout_sec` | double | `3.0` | 等时间同步锁定的最大秒数；超时仍未锁定则强制发送 |

关闭自动归零：将 `auto_reset_origin_on_start` 设为 `false`，仅靠手动 service 触发。

## 5. 实现要点

- 复用 `diagnosticsTick`（默认 1 Hz），避免新增 timer
- 新增私有状态：
  - `origin_reset_done_ : bool` — 已完成自动归零
  - `node_start_time_ : rclcpp::Time` — 节点启动时刻
- 新增私有方法：
  - `bool sendSetLocalOrigin(float x, float y, float yaw, std::string & err)` — 抽出公共发送逻辑，被 service handler 与自动归零共用
  - `void maybeAutoResetOrigin()` — 在 `diagnosticsTick` 中调用

## 6. 使用流程

```
开机 → 启动 positioning_bridge_node
        │
        ├── 串口连接成功
        ├── 等待 time_sync 锁定 (典型 0.5~2s)
        ├── 自动发送 SET_LOCAL_ORIGIN(0,0,0)
        │   → 此时机器人当前真实位置在 ROS 坐标系中变为 (0, 0, 0)
        └── 后续 /odom 从 (0,0,0) 开始累计
            
比赛中按需手动归零:
        ros2 service call /positioning/reset_local_origin std_srvs/srv/Trigger
```

## 7. 注意事项

- **物理位置不变**，只是 ROS 中坐标重置。归零瞬间，依赖 `/odom` 做闭环的下游节点（如导航、`omni_chassis_control_node` 的位置闭环）会感知到位置跳变，应在归零前停止运动指令
- 节点重启 = 重新归零；若希望保留累计位姿，关闭 `auto_reset_origin_on_start` 即可
- 时间同步未锁定就强制归零时，发送时刻可能略有偏差，但对原点设置本身无影响
