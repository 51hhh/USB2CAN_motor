// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/ControlFrequency.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__STRUCT_H_

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

/// Struct defined in msg/ControlFrequency in the package motor_control_ros2.
/**
  * 控制系统频率统计消息
 */
typedef struct motor_control_ros2__msg__ControlFrequency
{
  std_msgs__msg__Header header;
  /// 实际控制循环频率 (Hz)
  double control_frequency;
  /// CAN 发送频率 (Hz)
  double can_tx_frequency;
  /// 目标控制频率 (Hz)
  double target_frequency;
} motor_control_ros2__msg__ControlFrequency;

// Struct for a sequence of motor_control_ros2__msg__ControlFrequency.
typedef struct motor_control_ros2__msg__ControlFrequency__Sequence
{
  motor_control_ros2__msg__ControlFrequency * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__ControlFrequency__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__STRUCT_H_
