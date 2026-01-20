// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/MotorStateGeneric.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/motor_state_generic__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_MotorStateGeneric_temperature
{
public:
  explicit Init_MotorStateGeneric_temperature(::motor_control_ros2::msg::MotorStateGeneric & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::MotorStateGeneric temperature(::motor_control_ros2::msg::MotorStateGeneric::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

class Init_MotorStateGeneric_torque
{
public:
  explicit Init_MotorStateGeneric_torque(::motor_control_ros2::msg::MotorStateGeneric & msg)
  : msg_(msg)
  {}
  Init_MotorStateGeneric_temperature torque(::motor_control_ros2::msg::MotorStateGeneric::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_MotorStateGeneric_temperature(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

class Init_MotorStateGeneric_velocity
{
public:
  explicit Init_MotorStateGeneric_velocity(::motor_control_ros2::msg::MotorStateGeneric & msg)
  : msg_(msg)
  {}
  Init_MotorStateGeneric_torque velocity(::motor_control_ros2::msg::MotorStateGeneric::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorStateGeneric_torque(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

class Init_MotorStateGeneric_position
{
public:
  explicit Init_MotorStateGeneric_position(::motor_control_ros2::msg::MotorStateGeneric & msg)
  : msg_(msg)
  {}
  Init_MotorStateGeneric_velocity position(::motor_control_ros2::msg::MotorStateGeneric::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorStateGeneric_velocity(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

class Init_MotorStateGeneric_online
{
public:
  explicit Init_MotorStateGeneric_online(::motor_control_ros2::msg::MotorStateGeneric & msg)
  : msg_(msg)
  {}
  Init_MotorStateGeneric_position online(::motor_control_ros2::msg::MotorStateGeneric::_online_type arg)
  {
    msg_.online = std::move(arg);
    return Init_MotorStateGeneric_position(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

class Init_MotorStateGeneric_motor_type
{
public:
  explicit Init_MotorStateGeneric_motor_type(::motor_control_ros2::msg::MotorStateGeneric & msg)
  : msg_(msg)
  {}
  Init_MotorStateGeneric_online motor_type(::motor_control_ros2::msg::MotorStateGeneric::_motor_type_type arg)
  {
    msg_.motor_type = std::move(arg);
    return Init_MotorStateGeneric_online(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

class Init_MotorStateGeneric_joint_name
{
public:
  explicit Init_MotorStateGeneric_joint_name(::motor_control_ros2::msg::MotorStateGeneric & msg)
  : msg_(msg)
  {}
  Init_MotorStateGeneric_motor_type joint_name(::motor_control_ros2::msg::MotorStateGeneric::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_MotorStateGeneric_motor_type(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

class Init_MotorStateGeneric_header
{
public:
  Init_MotorStateGeneric_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorStateGeneric_joint_name header(::motor_control_ros2::msg::MotorStateGeneric::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorStateGeneric_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::MotorStateGeneric msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::MotorStateGeneric>()
{
  return motor_control_ros2::msg::builder::Init_MotorStateGeneric_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__BUILDER_HPP_
