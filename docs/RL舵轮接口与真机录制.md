# RL 舵轮接口与真机录制

本分支把学习控制固定在底盘速度层。RL 只输出三个归一化残差
`delta_vx/delta_vy/delta_wz`，禁止直接发布 8 个电机命令。

## 控制链

```
经典控制 /cmd_vel_classic
             +  RL残差 /rl/swerve_residual_normalized ([-1,1]^3)
             -> rl_velocity_residual_node
             -> /cmd_vel_remote
             -> cmd_vel_mux_node -> /cmd_vel
             -> chassis_control_node -> 4转向位置环 + 4驱动速度环
```

RL 残差默认只有 G0 实测速度上限的 5%。RL 超时会退回经典控制；经典
控制、底盘健康或急停心跳超时会立即归零。桥接节点所有速度/加速度上限
默认为 0，未填入 G0 实测值时拒绝启动。

## 本分支修复

- 四个模块都参与反馈和正运动学，RPM 按 `rpm*2*pi/60*radius` 转换。
- 转向角正确应用零位和方向；静止/安全停车保持当前舵角。
- 四轮最小二乘正运动学、整体轮速缩放、舵角最短路径。
- 平移/旋转加速度、舵角速率、余弦补偿和转向未对准禁驱。
- 8 电机反馈新鲜度、在线、温度和电流门禁。
- 命令、电机反馈、急停心跳三重看门狗，任何一项失败立即零驱动。
- `/chassis/limited_cmd` 和扩展 `/chassis_state` 暴露限幅后命令、四轮状态、
  饱和、故障、电流和安全停车原因。

`fault` 是控制器派生安全位：反馈丢失/离线、温度超过阈值或电流绝对值
超过阈值均置位；它不是电调原生故障寄存器。真机记录必须保留此语义。

## 上车前门禁

1. 复制 `motors_swerve.pending.yaml`，把两个 USB-CAN 设备替换成稳定的
   `/dev/serial/by-id/...` 路径，核对总线和 ID，禁止直接使用 pending 文件。
2. 实测并冻结轮径、模块位置、速度/加速度/舵速、电机阈值、控制频率、
   零位和方向。当前 `chassis_params.yaml` 数字仍是候选值。
3. 急停节点必须持续发布 `/emergency_stop=false` 心跳；断流等价于急停。
4. 架空车轮并断开驱动动力完成话题/HIL检查，再进行 10% G0 限制的首轮。
5. 同时确认 `/odom` 的 `odom -> base_link` 坐标、时间同步和速度单位。

## 真实数据录制

每个调度任务单独输出一个不可覆盖的 CSV。示例：

```bash
ros2 run motor_control_ros2 record_swerve_rl_data.py --ros-args \
  -p confirm_real_robot:=true \
  -p session_id:=train-vx-01 \
  -p operator:=OPERATOR_NAME \
  -p controller_version:=GIT_COMMIT \
  -p firmware_version:=FIRMWARE_VERSION \
  -p chassis_manifest_sha256:=64_CHAR_LOWERCASE_SHA256 \
  -p output_path:=/data/v5-real/train-vx-01.csv \
  -p duration_s:=30.0 \
  -p wheel_radius_m:=MEASURED_RADIUS
```

录制器按 `/chassis_state` 时间戳采样，与最近 `/odom` 的时间差必须不超过
30 ms。CSV 同时包含限幅前/后命令、位姿/速度、4舵角、4轮速、饱和、故障
和电流；轮速保存为 rad/s。成功达到时长后才把 `.partial` 原子改名为目标
CSV，并生成含 SHA256、操作员、单位和同步条件的 metadata JSON。中断的
`.partial` 文件不得进入 G1。

电压不是现有 DJI 反馈字段，因此不伪造电压列。将 CSV 导入 RSI 时仍需显式
确认 `source=real_robot`，并通过冻结的 G0/G1 门禁；本录制器不会自动授权
RL 上车。
