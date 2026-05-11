# ODOM 下位机数据使用说明

## 1. 硬件连接

| 参数 | 值 |
|------|---|
| 接口 | UART (TTL 3.3V) |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 输出频率 | **100 Hz**（每 10ms 一帧） |

接线：下位机 TX → 上位机 RX，共地。

---

## 2. 帧格式

每帧固定 **45 字节**，结构如下：

```
偏移  长度  字段          说明
─────────────────────────────────────────
0     2     帧头          固定 0xAA 0x55
2     1     版本号        当前 = 0x01
3     1     消息类型      0x02 = ODOM_STATE
4     1     帧序号        0~255 循环递增
5     2     Payload长度   固定 0x24 0x00 (=36, 小端)
7     36    Payload       数据区（见下）
43    2     CRC16         校验码（小端）
─────────────────────────────────────────
总长: 45 字节
```

---

## 3. Payload 数据区（36 字节）

| 偏移 | 类型 | 字段 | 单位 | 说明 |
|------|------|------|------|------|
| 0 | uint64 | t_sample_us | μs | 采样时间戳（上电开始计时） |
| 8 | float32 | x | m | 位置 X（前进方向为正） |
| 12 | float32 | y | m | 位置 Y（左方为正） |
| 16 | float32 | yaw | rad | 航向角（连续值，逆时针为正） |
| 20 | float32 | vx | m/s | 体坐标系前进速度 |
| 24 | float32 | vy | m/s | 体坐标系侧向速度（左为正） |
| 28 | float32 | wz | rad/s | 角速度（逆时针为正） |
| 32 | uint16 | status_bits | — | 状态标志位（见下） |
| 34 | uint8 | quality | — | 数据质量等级 |
| 35 | uint8 | reserved | — | 保留（=0） |

### 坐标系约定（ROS2 标准）

```
        X (前进 +)
        ↑
        |
 Y ←────⊕ (底盘中心)
(左 +)
        yaw: 逆时针为正
```

---

## 4. 状态标志位 (status_bits)

| Bit | 名称 | 含义 |
|-----|------|------|
| 0 | ENC_VALID | 编码器数据有效 |
| 1 | IMU_VALID | IMU 数据有效 |
| 2 | YAW_VALID | 航向角有效 |
| 3 | POS_VALID | 位置有效 |
| 4 | VEL_VALID | 速度有效 |
| 5 | RELOCATE | 重定位中（预留） |
| 6 | TIME_SYNC | 时间已同步（预留） |
| 7 | DEGRADED | 降级运行 |

正常工作时 status_bits = 0x001F（低5位全1）。

### 质量等级 (quality)

| 值 | 含义 |
|----|------|
| 0 | 不可用 |
| 1 | 降级 |
| 2 | 正常 |
| 3 | 高精度 |

---

## 5. CRC16 校验

- 算法：**CRC-16/CCITT**
- 初始值：0xFFFF
- 多项式：0x1021
- 计算范围：**字节 [2] 到 [42]**（从版本号到 Payload 末尾，共 41 字节）
- **不包含**帧头 AA 55
- CRC 存储：**小端**（低字节在前）

### Python 实现

```python
def odom_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# 验证:
# crc_calc = odom_crc16(frame[2:43])
# crc_recv = frame[43] | (frame[44] << 8)
# assert crc_calc == crc_recv
```

---

## 6. 解析示例（Python）

```python
import struct

FRAME_LEN = 45

def parse_odom_state(frame: bytes):
    """解析一帧 ODOM_STATE，返回字典"""
    if len(frame) < FRAME_LEN:
        return None
    if frame[0] != 0xAA or frame[1] != 0x55:
        return None
    if frame[3] != 0x02:  # 非 ODOM_STATE
        return None

    # CRC 校验
    crc_calc = odom_crc16(frame[2:43])
    crc_recv = frame[43] | (frame[44] << 8)
    if crc_calc != crc_recv:
        return None

    # 解包 payload
    payload = frame[7:43]
    (t_us, x, y, yaw, vx, vy, wz,
     status, quality, _) = struct.unpack("<QffffffHBB", payload)

    return {
        "t_us": t_us,
        "x": x, "y": y, "yaw": yaw,
        "vx": vx, "vy": vy, "wz": wz,
        "status": status, "quality": quality
    }
```

完整解析器参见 `tools/odom_parser.py`。

---

## 7. 接收流程

```
1. 打开串口 (115200, 8N1)
2. 循环读取字节到缓冲区
3. 在缓冲区中搜索帧头 AA 55
4. 找到后检查是否有足够 45 字节
5. 验证 CRC16
6. CRC 通过 → 解析 payload → 使用数据
7. CRC 失败 → 跳过 1 字节，继续搜索
```

---

## 8. ROS2 集成建议

将解析后的数据发布为 `nav_msgs/Odometry`：

```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

msg = Odometry()
msg.header.stamp = <ROS时间>
msg.header.frame_id = "odom"
msg.child_frame_id = "base_link"

# 位置
msg.pose.pose.position.x = data["x"]
msg.pose.pose.position.y = data["y"]
msg.pose.pose.position.z = 0.0

# 航向 → 四元数
q = Quaternion()
q.z = math.sin(data["yaw"] / 2.0)
q.w = math.cos(data["yaw"] / 2.0)
msg.pose.pose.orientation = q

# 速度（体坐标系）
msg.twist.twist.linear.x = data["vx"]
msg.twist.twist.linear.y = data["vy"]
msg.twist.twist.angular.z = data["wz"]
```

---

## 9. 注意事项

1. **时间戳**：t_sample_us 是 MCU 上电后的时间，不是 UTC。需要上位机自行对齐。
2. **yaw 连续性**：航向角已做 ±180° 跳变消除（unwrap），可直接积分使用。
3. **第一帧**：上电后前几帧数据可能不稳定，建议丢弃前 10 帧。
4. **帧丢失**：通过 seq 字段检测。如果 seq 跳跃说明有帧丢失。
5. **速度尖峰**：快速启停时可能出现瞬时大值，可在上位机做滑动平均滤波。
6. **舵轮底盘**：x/y/vx/vy 是用全向轮运动学计算的，舵轮底盘应忽略这些字段，仅使用 yaw 和 wz，自行根据转向角计算里程计。

---

## 10. 通信带宽

```
45 字节 × 100 Hz = 4500 字节/秒
115200 baud / 10 = 11520 字节/秒 (容量)
占用率: 4500/11520 = 39%
```

余量充足，不存在丢帧风险（纯 DMA 发送，不占 CPU）。
