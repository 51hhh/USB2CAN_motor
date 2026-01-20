// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/DJIMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__STRUCT_H_

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
// Member 'model'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DJIMotorState in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__DJIMotorState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String joint_name;
  /// "GM3508" 或 "GM6020"
  rosidl_runtime_c__String model;
  bool online;
  /// 角度（度，0-360）
  double angle;
  /// 转速
  int16_t rpm;
  /// 实际电流
  int16_t current;
  /// 温度
  uint8_t temperature;
  /// 控制频率 (Hz)
  double control_frequency;
} motor_control_ros2__msg__DJIMotorState;

// Struct for a sequence of motor_control_ros2__msg__DJIMotorState.
typedef struct motor_control_ros2__msg__DJIMotorState__Sequence
{
  motor_control_ros2__msg__DJIMotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__DJIMotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__STRUCT_H_
