// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/DamiaoMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__STRUCT_H_

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

/// Struct defined in msg/DamiaoMotorState in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__DamiaoMotorState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String joint_name;
  /// "DM4310", "DM4340" 等
  rosidl_runtime_c__String model;
  bool online;
  /// 位置（弧度）
  float position;
  /// 速度（弧度/秒）
  float velocity;
  /// 力矩（Nm）
  float torque;
  /// MOS 温度
  uint8_t temp_mos;
  /// 转子温度
  uint8_t temp_rotor;
  /// 错误码
  uint8_t error_code;
} motor_control_ros2__msg__DamiaoMotorState;

// Struct for a sequence of motor_control_ros2__msg__DamiaoMotorState.
typedef struct motor_control_ros2__msg__DamiaoMotorState__Sequence
{
  motor_control_ros2__msg__DamiaoMotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__DamiaoMotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__STRUCT_H_
