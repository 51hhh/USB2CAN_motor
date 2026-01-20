// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/DamiaoMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/damiao_motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_DamiaoMotorState_error_code
{
public:
  explicit Init_DamiaoMotorState_error_code(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::DamiaoMotorState error_code(::motor_control_ros2::msg::DamiaoMotorState::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_temp_rotor
{
public:
  explicit Init_DamiaoMotorState_temp_rotor(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_error_code temp_rotor(::motor_control_ros2::msg::DamiaoMotorState::_temp_rotor_type arg)
  {
    msg_.temp_rotor = std::move(arg);
    return Init_DamiaoMotorState_error_code(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_temp_mos
{
public:
  explicit Init_DamiaoMotorState_temp_mos(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_temp_rotor temp_mos(::motor_control_ros2::msg::DamiaoMotorState::_temp_mos_type arg)
  {
    msg_.temp_mos = std::move(arg);
    return Init_DamiaoMotorState_temp_rotor(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_torque
{
public:
  explicit Init_DamiaoMotorState_torque(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_temp_mos torque(::motor_control_ros2::msg::DamiaoMotorState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_DamiaoMotorState_temp_mos(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_velocity
{
public:
  explicit Init_DamiaoMotorState_velocity(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_torque velocity(::motor_control_ros2::msg::DamiaoMotorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DamiaoMotorState_torque(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_position
{
public:
  explicit Init_DamiaoMotorState_position(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_velocity position(::motor_control_ros2::msg::DamiaoMotorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_DamiaoMotorState_velocity(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_online
{
public:
  explicit Init_DamiaoMotorState_online(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_position online(::motor_control_ros2::msg::DamiaoMotorState::_online_type arg)
  {
    msg_.online = std::move(arg);
    return Init_DamiaoMotorState_position(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_model
{
public:
  explicit Init_DamiaoMotorState_model(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_online model(::motor_control_ros2::msg::DamiaoMotorState::_model_type arg)
  {
    msg_.model = std::move(arg);
    return Init_DamiaoMotorState_online(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_joint_name
{
public:
  explicit Init_DamiaoMotorState_joint_name(::motor_control_ros2::msg::DamiaoMotorState & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorState_model joint_name(::motor_control_ros2::msg::DamiaoMotorState::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_DamiaoMotorState_model(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

class Init_DamiaoMotorState_header
{
public:
  Init_DamiaoMotorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DamiaoMotorState_joint_name header(::motor_control_ros2::msg::DamiaoMotorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DamiaoMotorState_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::DamiaoMotorState>()
{
  return motor_control_ros2::msg::builder::Init_DamiaoMotorState_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__BUILDER_HPP_
