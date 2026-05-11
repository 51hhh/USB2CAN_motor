# vision_catch_controller_node 说明

## 作用

第一版接球控制节点：

- 输入视觉给出的落点目标（仅 `x, y`）
- 订阅当前 `/odom`
- 输出 `/cmd_vel_remote`
- 供 `cmd_vel_mux_node` 作为自动控制源使用

控制链路如下：

`goal_pose -> vision_catch_controller_node -> /cmd_vel_remote -> cmd_vel_mux_node -> /cmd_vel -> chassis_control_node`

## 第一版约束

- 目标坐标系：`vision_world`
- 只处理落点位置，不处理姿态
- 控制方式：距离模长 + 夹角
- 到达目标点后停车等待
- 目标超时或 odom 超时后停车等待
- 预留 TF 扩展接口，但当前测试版先直接按目标点运行

## 默认参数文件

- `src/motor_control_ros2/config/vision_catch_controller_params.yaml`

节点会采用与现有 `motor_control_node` / `remote_control_node` 类似的方式：

- 启动时先读取 `config_file` 参数
- 若未显式指定，则自动从包内加载默认 YAML
- 默认路径：`share/motor_control_ros2/config/vision_catch_controller_params.yaml`

默认参数：

- `goal_topic`: `/auto/goal_pose`
- `odom_topic`: `/odom`
- `cmd_vel_topic`: `/cmd_vel_remote`
- `goal_frame`: `vision_world`
- `control_frequency`: `50.0`
- `approach_gain`: `1.0`
- `max_linear_speed`: `1.0`
- `position_tolerance`: `0.05`
- `goal_timeout_sec`: `1.0`
- `odom_timeout_sec`: `0.5`

## 当前节点文件

- `src/motor_control_ros2/src/vision_catch_controller_node.cpp`
