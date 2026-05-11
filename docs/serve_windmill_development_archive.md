# ServeWindmillManager 开发归档

> 合并自 2026-04 ~ 2026-04-09 期间的所有 round_summary 文档

---

## 一、发球流程概述

```
IDLE → GRAVITY_HOMING → WIND_UP → (trigger) → FIRE → FREE_WHEEL → CATCH → GRAVITY_HOMING → ...
```

- 2× DJI M3508（C620 ESC），主从镜像：`current_m1 = -current_m2`
- `motor_direction_ = -1` 同时翻转反馈和输出
- PD 控制引臂/接臂，固定电流甩臂

---

## 二、修改历程

### 2.1 删除锁存 + CATCH 回 0°（2026-04-04）

**锁存删除**：移除 `execute_request_pending_`，`requestExecute()` 改为 switch-case 直接触发。WIND_UP 只做 PD 保持位置，收到 trigger 才开火。

**CATCH 回归零位**：CATCH 完成后转入 GRAVITY_HOMING 重新建立 0° 基准，不再 `home_offset += 360`。

### 2.2 参数机制迁移（2026-04-08）

从手动 yaml-cpp 解析迁移到 ROS2 `declare_parameter` / `get_parameter`。迁移中曾误把代码插入订阅器参数列表导致连锁编译错误，已修复。

### 2.3 自动读取 YAML（2026-04-08）

仿照 `motor_control_node`，启动时自动 `ament_index_cpp` 定位配置文件 + `YAML::LoadFile` 加载参数。

**根因发现**：YAML 首行多了一个空格（`  serve_windmill_manager:` 应为 `serve_windmill_manager:`），导致结构解析失败。修正后实测确认 `wind_up=190.0` 正确生效。

### 2.4 代码精简（2026-04-09）

删掉 `declare_parameter` / `set_parameter` / `get_parameter` 三层中间机制（~180 行），改为与 `motor_control_node` 一致的写法：默认值 → `YAML::LoadFile` → 直接赋值成员变量（~80 行）。

### 2.5 PD 参数调优（2026-04-09）

| 问题 | 根因 | 修复 |
|------|------|------|
| WIND_UP 剧烈震荡（±5000 翻转） | kp=150 太大，PD 超调不收敛 | kp 降低 |
| WIND_UP 暴走 | kd=30 × 电机轴 RPM(~1000) 碾压 kp 项 | kd 降到 3 |
| WIND_UP 力不够到位 | kp=20 稳态误差 12° | kp 提到 80，max_current 提到 8000 |
| CATCH 卡死不切换 | catch kp=30 精度不够，误差>3° | catch kp 提到 60，加 5s 超时 |

**最终参数**：wind_up kp=80 kd=3 max=8000 | catch kp=60 kd=4 max=8000

### 2.6 飞车保护调整（2026-04-09）

- 检测范围：从"排除 FIRE/FREE_WHEEL"改为**所有状态**（仅排除 IDLE/E_STOP）
- 阈值：720°(2圈) → 1080°(3圈)

### 2.7 fire 电流修正（2026-04-09）

YAML `fire: 120000` → `14000`（原值远超 int16_t 范围，属笔误）

---

## 三、关键设计要点

1. **PD 公式**：`output = kp * error_deg - kd * feedback_rpm_`，其中 rpm 是电机轴 RPM（M3508 约 ±几千）
2. **kd 不能太大**：kd × RPM 量级必须与 kp × error 在同一数量级，否则阻尼项碾压位置项导致震荡
3. **motor_direction_**：同时翻转 feedback（angle, rpm）和 output（sendCurrent 内乘 direction），PD 环内部看到的是翻转后的坐标系
4. **home_offset 累积**：每次归零后 `home_offset_ = feedback_angle_deg_`（连续累积值），不影响 PD 差值计算
5. **CATCH 超时**：5 秒兜底，防止 PD 精度不够时永远无法退出

---

## 四、当前文件状态

| 文件 | 行数 | 状态 |
|------|------|------|
| serve_windmill_manager.cpp | ~418 | ✅ 稳定 |
| serve_windmill_manager.hpp | ~110 | ✅ 稳定 |
| serve_windmill_params.yaml | ~41 | ✅ 参数正确 |
