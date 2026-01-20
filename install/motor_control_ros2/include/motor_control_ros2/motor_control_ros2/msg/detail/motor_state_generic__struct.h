// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/MotorStateGeneric.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__STRUCT_H_

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
// Member 'motor_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorStateGeneric in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__MotorStateGeneric
{
  std_msgs__msg__Header header;
  /// 关节名称
  rosidl_runtime_c__String joint_name;
  /// 电机类型: "dji", "damiao", "unitree"
  rosidl_runtime_c__String motor_type;
  /// 是否在线
  bool online;
  /// 位置（弧度）
  double position;
  /// 速度（弧度/秒）
  double velocity;
  /// 力矩（Nm）
  double torque;
  /// 温度（摄氏度）
  float temperature;
} motor_control_ros2__msg__MotorStateGeneric;

// Struct for a sequence of motor_control_ros2__msg__MotorStateGeneric.
typedef struct motor_control_ros2__msg__MotorStateGeneric__Sequence
{
  motor_control_ros2__msg__MotorStateGeneric * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__MotorStateGeneric__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__STRUCT_H_
