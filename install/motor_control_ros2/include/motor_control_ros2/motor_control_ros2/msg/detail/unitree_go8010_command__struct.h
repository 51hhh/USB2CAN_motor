// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/UnitreeGO8010Command.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_BRAKE'.
/**
  * 控制模式
  * 刹车模式
 */
enum
{
  motor_control_ros2__msg__UnitreeGO8010Command__MODE_BRAKE = 0
};

/// Constant 'MODE_FOC'.
/**
  * FOC闭环控制
 */
enum
{
  motor_control_ros2__msg__UnitreeGO8010Command__MODE_FOC = 1
};

/// Constant 'MODE_CALIBRATE'.
/**
  * 校准模式
 */
enum
{
  motor_control_ros2__msg__UnitreeGO8010Command__MODE_CALIBRATE = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'device'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/UnitreeGO8010Command in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__UnitreeGO8010Command
{
  std_msgs__msg__Header header;
  /// 电机 ID (0-15)
  uint8_t id;
  /// 串口设备路径（如 /dev/ttyUSB0），为空时广播到所有同 ID 电机
  rosidl_runtime_c__String device;
  uint8_t mode;
  /// 目标值（FOC模式使用）
  /// 期望位置（弧度），FOC模式时使用
  double position_target;
  /// 期望速度（弧度/秒），FOC模式时使用
  double velocity_target;
  /// 期望力矩（Nm），FOC模式时使用
  float torque_ff;
  /// 关节刚度系数，FOC模式时使用
  float kp;
  /// 关节阻尼系数，FOC模式时使用
  float kd;
} motor_control_ros2__msg__UnitreeGO8010Command;

// Struct for a sequence of motor_control_ros2__msg__UnitreeGO8010Command.
typedef struct motor_control_ros2__msg__UnitreeGO8010Command__Sequence
{
  motor_control_ros2__msg__UnitreeGO8010Command * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__UnitreeGO8010Command__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__STRUCT_H_
