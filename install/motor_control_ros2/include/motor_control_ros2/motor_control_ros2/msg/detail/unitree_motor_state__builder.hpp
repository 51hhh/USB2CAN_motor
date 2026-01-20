// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/UnitreeMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/unitree_motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_UnitreeMotorState_error
{
public:
  explicit Init_UnitreeMotorState_error(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::UnitreeMotorState error(::motor_control_ros2::msg::UnitreeMotorState::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_temperature
{
public:
  explicit Init_UnitreeMotorState_temperature(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_error temperature(::motor_control_ros2::msg::UnitreeMotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_UnitreeMotorState_error(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_acceleration
{
public:
  explicit Init_UnitreeMotorState_acceleration(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_temperature acceleration(::motor_control_ros2::msg::UnitreeMotorState::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_UnitreeMotorState_temperature(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_torque
{
public:
  explicit Init_UnitreeMotorState_torque(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_acceleration torque(::motor_control_ros2::msg::UnitreeMotorState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_UnitreeMotorState_acceleration(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_velocity
{
public:
  explicit Init_UnitreeMotorState_velocity(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_torque velocity(::motor_control_ros2::msg::UnitreeMotorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_UnitreeMotorState_torque(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_position
{
public:
  explicit Init_UnitreeMotorState_position(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_velocity position(::motor_control_ros2::msg::UnitreeMotorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_UnitreeMotorState_velocity(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_online
{
public:
  explicit Init_UnitreeMotorState_online(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_position online(::motor_control_ros2::msg::UnitreeMotorState::_online_type arg)
  {
    msg_.online = std::move(arg);
    return Init_UnitreeMotorState_position(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_mode
{
public:
  explicit Init_UnitreeMotorState_mode(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_online mode(::motor_control_ros2::msg::UnitreeMotorState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_UnitreeMotorState_online(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_motor_id
{
public:
  explicit Init_UnitreeMotorState_motor_id(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_mode motor_id(::motor_control_ros2::msg::UnitreeMotorState::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_UnitreeMotorState_mode(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_leg_id
{
public:
  explicit Init_UnitreeMotorState_leg_id(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_motor_id leg_id(::motor_control_ros2::msg::UnitreeMotorState::_leg_id_type arg)
  {
    msg_.leg_id = std::move(arg);
    return Init_UnitreeMotorState_motor_id(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_joint_name
{
public:
  explicit Init_UnitreeMotorState_joint_name(::motor_control_ros2::msg::UnitreeMotorState & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorState_leg_id joint_name(::motor_control_ros2::msg::UnitreeMotorState::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_UnitreeMotorState_leg_id(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

class Init_UnitreeMotorState_header
{
public:
  Init_UnitreeMotorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UnitreeMotorState_joint_name header(::motor_control_ros2::msg::UnitreeMotorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UnitreeMotorState_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::UnitreeMotorState>()
{
  return motor_control_ros2::msg::builder::Init_UnitreeMotorState_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__BUILDER_HPP_
