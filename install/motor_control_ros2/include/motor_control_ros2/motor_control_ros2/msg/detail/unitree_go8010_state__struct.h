// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/UnitreeGO8010State.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_STATE__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'joint_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/UnitreeGO8010State in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__UnitreeGO8010State
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String joint_name;
  /// 电机 ID
  uint8_t motor_id;
  /// 模式
  uint8_t mode;
  bool online;
  /// 位置（弧度）
  float position;
  /// 速度（弧度/秒）
  float velocity;
  /// 力矩（Nm）
  float torque;
  /// 加速度
  int16_t acceleration;
  /// 温度
  int8_t temperature;
  /// 错误码
  int8_t error;
} motor_control_ros2__msg__UnitreeGO8010State;

// Struct for a sequence of motor_control_ros2__msg__UnitreeGO8010State.
typedef struct motor_control_ros2__msg__UnitreeGO8010State__Sequence
{
  motor_control_ros2__msg__UnitreeGO8010State * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__UnitreeGO8010State__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_STATE__STRUCT_H_
