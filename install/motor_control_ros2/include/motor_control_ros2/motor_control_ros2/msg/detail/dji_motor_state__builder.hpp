// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/DJIMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/dji_motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_DJIMotorState_control_frequency
{
public:
  explicit Init_DJIMotorState_control_frequency(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::DJIMotorState control_frequency(::motor_control_ros2::msg::DJIMotorState::_control_frequency_type arg)
  {
    msg_.control_frequency = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_temperature
{
public:
  explicit Init_DJIMotorState_temperature(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  Init_DJIMotorState_control_frequency temperature(::motor_control_ros2::msg::DJIMotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_DJIMotorState_control_frequency(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_current
{
public:
  explicit Init_DJIMotorState_current(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  Init_DJIMotorState_temperature current(::motor_control_ros2::msg::DJIMotorState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_DJIMotorState_temperature(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_rpm
{
public:
  explicit Init_DJIMotorState_rpm(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  Init_DJIMotorState_current rpm(::motor_control_ros2::msg::DJIMotorState::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_DJIMotorState_current(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_angle
{
public:
  explicit Init_DJIMotorState_angle(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  Init_DJIMotorState_rpm angle(::motor_control_ros2::msg::DJIMotorState::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_DJIMotorState_rpm(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_online
{
public:
  explicit Init_DJIMotorState_online(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  Init_DJIMotorState_angle online(::motor_control_ros2::msg::DJIMotorState::_online_type arg)
  {
    msg_.online = std::move(arg);
    return Init_DJIMotorState_angle(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_model
{
public:
  explicit Init_DJIMotorState_model(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  Init_DJIMotorState_online model(::motor_control_ros2::msg::DJIMotorState::_model_type arg)
  {
    msg_.model = std::move(arg);
    return Init_DJIMotorState_online(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_joint_name
{
public:
  explicit Init_DJIMotorState_joint_name(::motor_control_ros2::msg::DJIMotorState & msg)
  : msg_(msg)
  {}
  Init_DJIMotorState_model joint_name(::motor_control_ros2::msg::DJIMotorState::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_DJIMotorState_model(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

class Init_DJIMotorState_header
{
public:
  Init_DJIMotorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DJIMotorState_joint_name header(::motor_control_ros2::msg::DJIMotorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DJIMotorState_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::DJIMotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::DJIMotorState>()
{
  return motor_control_ros2::msg::builder::Init_DJIMotorState_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__BUILDER_HPP_
