// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/DJIMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_COMMAND__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/dji_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_DJIMotorCommand_output
{
public:
  explicit Init_DJIMotorCommand_output(::motor_control_ros2::msg::DJIMotorCommand & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::DJIMotorCommand output(::motor_control_ros2::msg::DJIMotorCommand::_output_type arg)
  {
    msg_.output = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorCommand msg_;
};

class Init_DJIMotorCommand_joint_name
{
public:
  explicit Init_DJIMotorCommand_joint_name(::motor_control_ros2::msg::DJIMotorCommand & msg)
  : msg_(msg)
  {}
  Init_DJIMotorCommand_output joint_name(::motor_control_ros2::msg::DJIMotorCommand::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_DJIMotorCommand_output(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorCommand msg_;
};

class Init_DJIMotorCommand_header
{
public:
  Init_DJIMotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DJIMotorCommand_joint_name header(::motor_control_ros2::msg::DJIMotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DJIMotorCommand_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::DJIMotorCommand>()
{
  return motor_control_ros2::msg::builder::Init_DJIMotorCommand_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_COMMAND__BUILDER_HPP_
