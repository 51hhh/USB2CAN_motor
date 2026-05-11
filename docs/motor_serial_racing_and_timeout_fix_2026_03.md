# 宇树电机串口通信频繁掉线（在线/离线跳变）BUG 修复分析记录

**日期：** 2026-03-31  
**影响组件：** `motor_control_ros2` 节点中的串口轮询机制，宇树原生电机驱动 (`GO8010`)  
**相关文件：**
- `src/motor_control_ros2/src/hardware/serial_interface.cpp`
- `src/motor_control_ros2/src/motor_control_node.cpp`
- `src/motor_control_ros2/config/motors.yaml`

---

## 1. 故障现象

在 ROS 2 控制节点中，日志显示 `[motor_control_node]` 中所有手臂关节的宇树电机（如 `arm_motor_l_1`, `arm_motor_l_2`, `arm_motor_r_1`, `arm_motor_r_2`）均出现极高频率的掉线现象。
- **具体表现**为 `motor_monitor_node` 中不断打印 `[arm_motor_l_1 上线]` 和 `[arm_motor_l_1 离线]` 的交替日志，甚至处于几毫秒内反复横跳的极端状态。
- **底层报错**为：`[Serial Thread] arm_motor_xxx: 通信失败 (recv=16 bytes, head=[FD EE ...]) iface=serial_X`。有时收不到字节，有时收到错误 ID 的返回帧（例如配置为 ID `0` 的电机收到了 ID `1` 的帧），或者 CRC 校验失败。
- 最终导致实际控制频率骤降至 `1~2 Hz`。

---

## 2. 根因分析

经过系统地梳理配置文件与 C++ 源码，发现导致掉线的核心原因由三个并发叠加的维度构成：

### 维度 A：配置文件的跨线程数据竞争（Data Race 核心祸首）
在原有的 `config/motors.yaml` 配置文件中：
```yaml
serial_interfaces:
  - device: /dev/ttyUSB0
    motors: [ arm_motor_l_1 ]
  - device: /dev/ttyUSB0
    motors: [ arm_motor_l_2 ]
```
这段配置让框架针对**同一个 Linux 硬件串口 (`/dev/ttyUSB0`) 实例化了多个独立的读写线程**（创建了 `serial_3`、`serial_4` 等不同逻辑对象，但底层指向同一个 fd）。这两个线程在并发抢占同一个 RS485 轮询总线，导致：
1. **错峰截流**：左臂 1 的线程可能顺手把左臂 2 的回包数据读走了。
2. **总线冲突**：由于无锁发送，多个指令会互相插入在物理总线上，彻底扰乱时序，导致帧头混乱及 CRC 解析崩溃。

### 维度 B：`sendRecvAccumulate` 的无意义轮询阻塞
原 `motor_control_node.cpp` 中定义 `BUF_SIZE` 为 48 字节，但在发送接收时直接请求读取 `BUF_SIZE` 的全量：
```cpp
ssize_t n = serial->sendRecvAccumulate(cmd, 17, buf, BUF_SIZE, kSerialWaitMs, kSerialTimeoutMs);
```
由于宇树电机仅仅返回 `16` 字节（也就是 `FRAME_LEN`）的数据包，由于始终填不满 `48` 字节，导致该函数即使已经完美接收了一整帧，依然会**死等**剩余的 32 字节，直到超过 `kSerialTimeoutMs` （原配置为 20ms）后才被迫弹出。从而将每轮原本只需 ~2ms 的串口单次交互直接拖成了硬性 >20ms。

### 维度 C：软等待系统休眠计时误差 (sleep vs 墙钟)
原有的 `serial_interface.cpp` 的 `receive` 函数中：
```cpp
// 以前底层依赖睡眠轮询计数
tv.tv_usec = (timeout_ms % 1000) * 1000;  // 动态等待过长
```
当 `select` 超时并且外部累计使用软中断方式计算时间差时，`select` 本身因系统调度导致的堵塞没有被精确归入控制环中，导致大量有效时间被空置并积压成更大的超时。

---

## 3. 修复方案实施

为了将单机通讯频率极限压迫到百赫兹以上（单电机 `<2ms`），我们实施了以下重整：

### 修复 1：规整 `motors.yaml` 消除数据竞争 (Data Race)
将同一个 `device` 硬件绑定的挂载对象**汇集**进同意层级的 `motors` 从数组：
```yaml
  - device: /dev/ttyUSB0
    baudrate: 4000000
    protocol: native
    motors:
      - name: arm_motor_l_1
        type: GO8010
        id: 0
        gear_ratio: 6.33        
      - name: arm_motor_l_2
        type: GO8010
        id: 1
        gear_ratio: 6.33
```
- **机制改变**：系统将只拉起单个统一的读写线程。对挂载的电机按严格的时间片排队逐个通信（时分复用），彻底消除串线争抢，根治 ID 跳跃和数据交叉。

### 修复 2：将 `sendRecvAccumulate` 预期接收量收严为唯一匹配帧宽
将 `motor_control_node.cpp` 里的请求配置更正：
```cpp
constexpr int kSerialWaitMs = 2; // 下压等待间隔
constexpr int kSerialTimeoutMs = 8;
auto try_recv_parse = [&](uint8_t (&buf)[BUF_SIZE]) -> bool {
    // 将 max_len 降维定义为精确的 FRAME_LEN (16)
    ssize_t n = serial->sendRecvAccumulate(cmd, 17, buf, FRAME_LEN, ...);
```
- **机制改变**：只要串口返回了一整帧宽度的 16 字节，直接提前结束读取并交付校验。将无意义空转浪费的 20ms 延时直接置零。

### 修复 3：底层引入硬时间线（`steady_clock`）与微秒级 `select` 锁定
修改了 `serial_interface.cpp` 中用于调取 `select` 队列的处理时长极限：
```cpp
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 2000; // 强制缩紧到极限 2ms
```
- **机制改变**：`select` 单次只被允许挂起 2 毫秒来探活。只要外层 `steady_clock::now()` `deadline` 仍有时间便连续高效探测，一旦抓取数据即刻进行内存拼接，保障 C++ 端对于实时系统的贴合响应。

---

## 4. 修复结果

- `motor_monitor_node` 日志下不再出现断电重签式（Online -> Offline -> Online）状态跳跃。
- 对左右两臂的从控串口（`ttyUSB0`, `ttyUSB1`）的各2枚电机的调度不再出现重叠竞争。
- 节点成功地将多个宇树端原生接口频率压稳，通信循环由原来的 >500ms 重登到了预设的稳定频率。