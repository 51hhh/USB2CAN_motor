// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/ControlFrequency.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/control_frequency__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_ControlFrequency_target_frequency
{
public:
  explicit Init_ControlFrequency_target_frequency(::motor_control_ros2::msg::ControlFrequency & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::ControlFrequency target_frequency(::motor_control_ros2::msg::ControlFrequency::_target_frequency_type arg)
  {
    msg_.target_frequency = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::ControlFrequency msg_;
};

class Init_ControlFrequency_can_tx_frequency
{
public:
  explicit Init_ControlFrequency_can_tx_frequency(::motor_control_ros2::msg::ControlFrequency & msg)
  : msg_(msg)
  {}
  Init_ControlFrequency_target_frequency can_tx_frequency(::motor_control_ros2::msg::ControlFrequency::_can_tx_frequency_type arg)
  {
    msg_.can_tx_frequency = std::move(arg);
    return Init_ControlFrequency_target_frequency(msg_);
  }

private:
  ::motor_control_ros2::msg::ControlFrequency msg_;
};

class Init_ControlFrequency_control_frequency
{
public:
  explicit Init_ControlFrequency_control_frequency(::motor_control_ros2::msg::ControlFrequency & msg)
  : msg_(msg)
  {}
  Init_ControlFrequency_can_tx_frequency control_frequency(::motor_control_ros2::msg::ControlFrequency::_control_frequency_type arg)
  {
    msg_.control_frequency = std::move(arg);
    return Init_ControlFrequency_can_tx_frequency(msg_);
  }

private:
  ::motor_control_ros2::msg::ControlFrequency msg_;
};

class Init_ControlFrequency_header
{
public:
  Init_ControlFrequency_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlFrequency_control_frequency header(::motor_control_ros2::msg::ControlFrequency::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ControlFrequency_control_frequency(msg_);
  }

private:
  ::motor_control_ros2::msg::ControlFrequency msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::ControlFrequency>()
{
  return motor_control_ros2::msg::builder::Init_ControlFrequency_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__BUILDER_HPP_
