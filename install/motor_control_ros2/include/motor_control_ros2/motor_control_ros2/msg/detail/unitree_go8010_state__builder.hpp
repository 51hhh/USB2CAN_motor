// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/UnitreeGO8010State.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_STATE__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/unitree_go8010_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_UnitreeGO8010State_error
{
public:
  explicit Init_UnitreeGO8010State_error(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::UnitreeGO8010State error(::motor_control_ros2::msg::UnitreeGO8010State::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_temperature
{
public:
  explicit Init_UnitreeGO8010State_temperature(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_error temperature(::motor_control_ros2::msg::UnitreeGO8010State::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_UnitreeGO8010State_error(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_acceleration
{
public:
  explicit Init_UnitreeGO8010State_acceleration(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_temperature acceleration(::motor_control_ros2::msg::UnitreeGO8010State::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_UnitreeGO8010State_temperature(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_torque
{
public:
  explicit Init_UnitreeGO8010State_torque(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_acceleration torque(::motor_control_ros2::msg::UnitreeGO8010State::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_UnitreeGO8010State_acceleration(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_velocity
{
public:
  explicit Init_UnitreeGO8010State_velocity(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_torque velocity(::motor_control_ros2::msg::UnitreeGO8010State::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_UnitreeGO8010State_torque(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_position
{
public:
  explicit Init_UnitreeGO8010State_position(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_velocity position(::motor_control_ros2::msg::UnitreeGO8010State::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_UnitreeGO8010State_velocity(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_online
{
public:
  explicit Init_UnitreeGO8010State_online(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_position online(::motor_control_ros2::msg::UnitreeGO8010State::_online_type arg)
  {
    msg_.online = std::move(arg);
    return Init_UnitreeGO8010State_position(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_mode
{
public:
  explicit Init_UnitreeGO8010State_mode(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_online mode(::motor_control_ros2::msg::UnitreeGO8010State::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_UnitreeGO8010State_online(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_motor_id
{
public:
  explicit Init_UnitreeGO8010State_motor_id(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_mode motor_id(::motor_control_ros2::msg::UnitreeGO8010State::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_UnitreeGO8010State_mode(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_joint_name
{
public:
  explicit Init_UnitreeGO8010State_joint_name(::motor_control_ros2::msg::UnitreeGO8010State & msg)
  : msg_(msg)
  {}
  Init_UnitreeGO8010State_motor_id joint_name(::motor_control_ros2::msg::UnitreeGO8010State::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_UnitreeGO8010State_motor_id(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

class Init_UnitreeGO8010State_header
{
public:
  Init_UnitreeGO8010State_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UnitreeGO8010State_joint_name header(::motor_control_ros2::msg::UnitreeGO8010State::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UnitreeGO8010State_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeGO8010State msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::UnitreeGO8010State>()
{
  return motor_control_ros2::msg::builder::Init_UnitreeGO8010State_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_STATE__BUILDER_HPP_
