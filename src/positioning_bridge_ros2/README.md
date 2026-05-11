# positioning_bridge_ros2

用于将 STM32 本地定位链路桥接到 ROS2。

完整启动、回零、`/odom` 查看与跳变排查流程见：

- [`docs/positioning_bridge_整体使用说明.md`](../../docs/positioning_bridge_整体使用说明.md)

协议模式：

- 当前支持 `legacy_ascii` 模式：兼容 `bc x y yaw roll`
- 当前推荐 `binary_v1` 模式：对接 STM32 ODOM 二进制定位协议

节点职责：

- 读取串口定位链路
- 解析定位数据
- 进行时间同步（binary_v1）
- 发布 `/odom`
- 广播 `odom -> base_link`
- 发布 `/diagnostics`

正式系统中，该节点应成为唯一合法的 `odom -> base_link` 发布者。
