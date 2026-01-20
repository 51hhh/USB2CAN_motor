// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/UnitreeMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__STRUCT_H_

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

/// Struct defined in msg/UnitreeMotorState in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__UnitreeMotorState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String joint_name;
  /// 腿 ID (0-3)
  uint8_t leg_id;
  /// 电机 ID (0-2)
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
} motor_control_ros2__msg__UnitreeMotorState;

// Struct for a sequence of motor_control_ros2__msg__UnitreeMotorState.
typedef struct motor_control_ros2__msg__UnitreeMotorState__Sequence
{
  motor_control_ros2__msg__UnitreeMotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__UnitreeMotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__STRUCT_H_
