# 排球车视觉追球与底盘调试总结

时间：2026-07-04  
设备：NVIDIA `192.168.153.54`  
底盘项目：`~/USB2CAN_motor_new`  
视觉项目：`~/frontvehicle`  
ROS_DOMAIN_ID：`88`

## 当前目标

让底盘在视觉模式下稳定追球/追落点，重点处理：

- auto-goal 超距过滤，realtime 可追远点。
- 没有 auto-goal 时先追 realtime，有 pre-goal/auto-goal 时优先追落点。
- 追点速度足够快，但接近目标时不过冲。
- 减少右前方追点时的横向甩动和末端前偏。
- 用码盘闭环修正底盘 vx/vy/yaw，同时继续排查左后角落地负载异常。

## 视觉追点节点

新增节点：

```text
vision_goal_tracker_node
```

目标优先级：

```text
auto_goal > pre_goal > realtime
```

订阅：

```text
/frontvehicle/auto/goal_pose
/frontvehicle/auto/pre_goal_pose
/frontvehicle/ball/realtime
/frontvehicle/vision/zed_odom
```

发布：

```text
/vision/cmd_vel
```

关键规则：

- `auto_goal` 和 `pre_goal` 最大距离限制为 `7.0m`。
- `realtime` 最大距离限制为 `14.0m`，允许超过 5m。
- 需要 `/frontvehicle/vision/zed_odom`，否则输出零速。
- 目标或 odom 短暂丢失时放宽超时，避免一卡一卡。

当前关键参数：

```yaml
target_source: "auto_or_realtime"
goal_timeout: 1.5
odom_timeout: 0.8
require_odom: true
max_goal_distance: 7.0
max_realtime_distance: 14.0
auto_goal_hold_time: 6.0
pre_goal_hold_time: 2.0
```

## 追点速度与停靠

当前追点速度：

```yaml
kp_forward: 3.6
kp_lateral: 2.0
max_forward_velocity: 4.5
max_lateral_velocity: 1.3
max_far_lateral_velocity: 1.0
far_lateral_distance: 2.5
max_backward_velocity: 0.7
max_linear_accel: 16.0
```

接近目标时限速：

```yaml
stop_forward_distance: 0.70
approach_slow_distance: 1.2
max_near_forward_velocity: 1.1
reverse_deadband: 0.35
```

解释：

- `stop_forward_distance=0.70`：追点时停在落点前约 70cm，减少前偏。
- 横向最大速度从早期的 `2.2` 降到 `1.3`，避免末端突然横扫。
- 远距离横向速度从 `0.8` 提到 `1.0`，让轨迹更像连续斜线，而不是先前冲再横向修。

真实 ZED odom 测试中，发布相对当前点：

```text
前方 3m，右方 2m
```

调参后结果：

```text
END_ODOM x=2.442 y=-2.026
target_error=0.559
err_x=0.558
err_y=0.026
```

这与 `stop_forward_distance=0.70` 的提前停靠意图一致，横向误差约 2.6cm。

## 底盘闭环

底盘速度控制模式：

```yaml
velocity_control_mode: "closed_loop"
max_linear_velocity: 4.5
max_wheel_linear_velocity: 5.3
yaw_feedback_sign: -1.0
```

当前闭环反馈映射：

```cpp
feedback_right = -odom.twist.linear.y;
feedback_forward = odom.twist.linear.x;
feedback_yaw = yaw_feedback_sign * odom.twist.angular.z;
```

速度闭环为前馈加 PID 修正：

```text
cmd = target + velocity_pid(target, odom_feedback)
```

当前 PID：

```yaml
velocity_pid_x:
  kp: 0.45
  ki: 0.0
  out_max: 1.0
  dead_zone: 0.03

velocity_pid_y:
  kp: 0.45
  ki: 0.0
  out_max: 1.0
  dead_zone: 0.03

velocity_pid_yaw:
  kp: 0.7
  ki: 0.0
  out_max: 0.35
  dead_zone: 0.01
```

实测 `CMD_WZ=+0.2` 时 odom `wz` 为负，因此当前：

```yaml
yaw_feedback_sign: -1.0
```

不要随意改回 `+1.0`。

## 左后轮/左后角问题

已增加按电机名覆盖 PID 的能力：

```yaml
motor_overrides:
  DJI3508_3:
    velocity_pid:
      kp: 13.0
      ki: 2.0
      kd: 0.5
      i_max: 4000.0
      out_max: 16384.0
      dead_zone: 5.0
```

这个覆盖是为了补偿左后落地负载偏大。后续如果机械问题修好，需要重新评估是否降回默认。

### 当前测试结论

换电机后，悬空测试正常：

```text
前进 0.4m/s:
DJI3508_1  701.3 rpm
DJI3508_2 -700.3 rpm
DJI3508_3  700.1 rpm
DJI3508_4 -700.7 rpm

右移 0.4m/s:
DJI3508_1  700.3 rpm
DJI3508_2  700.5 rpm
DJI3508_3 -700.1 rpm
DJI3508_4 -700.9 rpm

原地转 0.4rad/s:
四轮约 722-723 rpm
```

但落地后左后仍异常：

```text
0.4m/s 前进落地:
DJI3508_1 左前: 501.5 rpm, current 36.5
DJI3508_3 左后: 442.9 rpm, current 3781.4
DJI3508_4 右后: -495.6 rpm, current -2428.4
```

手轻抬左后角做 `0.2m/s` 低速测试，左后电流仍未明显下降：

```text
DJI3508_1 左前: 252.1 rpm, current 59.2
DJI3508_3 左后: 217.0 rpm, current 3642.3
DJI3508_4 右后: -256.9 rpm, current -1973.9
```

因此当前判断：

- 不像单纯压地力过大。
- 更像左后轮座/联轴器/轴承/安装板在落地受力后卡滞或不同心。
- 悬空正常、落地异常，说明电控方向和编码基本正确，结构受力优先排查。

建议下一步机械检查：

1. 拆左后轮，手转输出轴和联轴器。
2. 装回轮子但不落地，确认是否变涩。
3. 落地断电手推 1-2m，摸左后是否周期性拖滞。
4. 松左后电机/轮座螺丝，释放装配应力后重新找正。
5. 检查左后轮轴是否水平、轮面是否垂直、联轴器是否顶死。

## 启动方式

底盘：

```bash
cd ~/USB2CAN_motor_new
./src/ROS_test/launch/start_chassis.sh
```

视觉：

```bash
cd ~/frontvehicle
export ROS_DOMAIN_ID=88
./run.sh --no-gui --ros --auto-goal
```

测试前确认：

```bash
export ROS_DOMAIN_ID=88
ros2 topic info /frontvehicle/vision/zed_odom
ros2 topic info /vision/cmd_vel
ros2 topic echo /chassis/estop --once
```

要求：

```text
/frontvehicle/vision/zed_odom Publisher count: 1
/vision/cmd_vel Publisher count: 1
/chassis/estop: false
```

## 注意事项

- 左拨杆上档：手动遥控。
- 左拨杆中档：硬急停零速。
- 左拨杆下档：视觉速度 `/vision/cmd_vel`。
- 视觉追点必须有 `/frontvehicle/vision/zed_odom`，只发 `auto_goal` 不会动。
- 假发固定 odom 只能验证方向，不能验证能否刹停；真实测试必须使用真实 ZED odom。
