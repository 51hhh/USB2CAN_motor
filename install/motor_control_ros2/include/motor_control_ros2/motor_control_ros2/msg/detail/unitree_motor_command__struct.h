// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_ros2:msg/UnitreeMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__STRUCT_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__STRUCT_H_

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

/// Struct defined in msg/UnitreeMotorCommand in the package motor_control_ros2.
typedef struct motor_control_ros2__msg__UnitreeMotorCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String joint_name;
  uint8_t leg_id;
  uint8_t motor_id;
  /// 0:空闲, 5:开环, 10:闭环FOC
  uint8_t mode;
  /// 关节刚度系数
  float kp;
  /// 关节速度系数
  float kd;
  /// 期望位置（弧度）
  float pos_des;
  /// 期望速度（弧度/秒）
  float vel_des;
  /// 期望力矩（Nm）
  float torque_ff;
} motor_control_ros2__msg__UnitreeMotorCommand;

// Struct for a sequence of motor_control_ros2__msg__UnitreeMotorCommand.
typedef struct motor_control_ros2__msg__UnitreeMotorCommand__Sequence
{
  motor_control_ros2__msg__UnitreeMotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_ros2__msg__UnitreeMotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__STRUCT_H_
