# cmd_vel 多源切换使用说明

## 目标

为避免手柄与自动控制同时写入 `/cmd_vel` 产生冲突，控制链路拆分为：

- 手动源：`/cmd_vel_joy`
- 自动源：`/cmd_vel_remote`
- 复用节点：`cmd_vel_mux_node`
- 底盘输入：`/cmd_vel`

## 节点参数

`cmd_vel_mux_node` 支持以下参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `joy_input_topic` | `/cmd_vel_joy` | 手动源输入话题 |
| `remote_input_topic` | `/cmd_vel_remote` | 自动源输入话题 |
| `output_topic` | `/cmd_vel` | 最终输出话题 |
| `active_source` | `remote` | 当前生效源（`joy` 或 `remote`） |
| `source_timeout_sec` | `0.5` | 活动源超时阈值（秒）；`<=0` 表示关闭超时保护 |
| `timeout_mode` | `brake` | 超时策略：`brake` / `fallback` |
| `fallback_source` | `joy` | 当 `timeout_mode=fallback` 时的备用源 |
| `lock_active_source` | `false` | `true` 时禁止运行时切源（自动控制锁定） |

参数默认配置见：

- `src/motor_control_ros2/config/cmd_vel_mux_params.yaml`
- `src/motor_control_ros2/config/cmd_vel_mux_auto_params.yaml`

## 运行时切换

`active_source` 支持运行时动态切换：

- 切到 `remote`：由自动控制接管
- 切到 `joy`：由手动控制接管

说明：

- 非法值会被拒绝（仅允许 `joy`、`remote`）
- 当 `lock_active_source=true` 时，切源请求会被拒绝
- 切换到目标源后，若目标源存在“新鲜缓存命令”，会立即转发该命令

## 超时保护

当当前活动源在 `source_timeout_sec` 内未收到新命令时：

1. `timeout_mode = brake`
   - 立即发布零速度（刹停）

2. `timeout_mode = fallback`
   - 若 `fallback_source` 命令新鲜：自动切源并继续输出
   - 否则：发布零速度（刹停）


自动控制锁定建议：

- 使用 `cmd_vel_mux_auto_params.yaml`（`active_source=remote` + `lock_active_source=true`）
- 这样可避免运行中误切到手动源

## 推荐联调顺序

1. 检查 `/cmd_vel_joy`、`/cmd_vel_remote` 均有输入
2. 设置 `active_source`，确认 `/cmd_vel` 跟随当前源
3. 人为停止当前源输入，观察是否按策略刹停/回退
4. 关注 `cmd_vel_mux_node` 日志，确认切源和超时提示符合预期

## 变更文件参考

- `src/motor_control_ros2/src/cmd_vel_mux_node.cpp`
- `src/motor_control_ros2/config/cmd_vel_mux_params.yaml`
- `src/motor_control_ros2/config/cmd_vel_mux_auto_params.yaml`
- `src/motor_control_ros2/README.md`
