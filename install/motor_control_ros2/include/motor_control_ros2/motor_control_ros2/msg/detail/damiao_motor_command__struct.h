// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/DamiaoMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__STRUCT_H_

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

/// Struct defined in msg/DamiaoMotorCommand in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__DamiaoMotorCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String joint_name;
  /// 期望位置（弧度）
  float pos_des;
  /// 期望速度（弧度/秒）
  float vel_des;
  /// 位置增益
  float kp;
  /// 速度增益
  float kd;
  /// 前馈力矩（Nm）
  float torque_ff;
} motor_control_ros2__msg__DamiaoMotorCommand;

// Struct for a sequence of motor_control_ros2__msg__DamiaoMotorCommand.
typedef struct motor_control_ros2__msg__DamiaoMotorCommand__Sequence
{
  motor_control_ros2__msg__DamiaoMotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__DamiaoMotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__STRUCT_H_
