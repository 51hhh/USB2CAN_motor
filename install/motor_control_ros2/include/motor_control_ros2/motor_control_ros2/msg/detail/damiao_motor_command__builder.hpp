// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/DamiaoMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/damiao_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_DamiaoMotorCommand_torque_ff
{
public:
  explicit Init_DamiaoMotorCommand_torque_ff(::motor_control_ros2::msg::DamiaoMotorCommand & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::DamiaoMotorCommand torque_ff(::motor_control_ros2::msg::DamiaoMotorCommand::_torque_ff_type arg)
  {
    msg_.torque_ff = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorCommand msg_;
};

class Init_DamiaoMotorCommand_kd
{
public:
  explicit Init_DamiaoMotorCommand_kd(::motor_control_ros2::msg::DamiaoMotorCommand & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorCommand_torque_ff kd(::motor_control_ros2::msg::DamiaoMotorCommand::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_DamiaoMotorCommand_torque_ff(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorCommand msg_;
};

class Init_DamiaoMotorCommand_kp
{
public:
  explicit Init_DamiaoMotorCommand_kp(::motor_control_ros2::msg::DamiaoMotorCommand & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorCommand_kd kp(::motor_control_ros2::msg::DamiaoMotorCommand::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_DamiaoMotorCommand_kd(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorCommand msg_;
};

class Init_DamiaoMotorCommand_vel_des
{
public:
  explicit Init_DamiaoMotorCommand_vel_des(::motor_control_ros2::msg::DamiaoMotorCommand & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorCommand_kp vel_des(::motor_control_ros2::msg::DamiaoMotorCommand::_vel_des_type arg)
  {
    msg_.vel_des = std::move(arg);
    return Init_DamiaoMotorCommand_kp(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorCommand msg_;
};

class Init_DamiaoMotorCommand_pos_des
{
public:
  explicit Init_DamiaoMotorCommand_pos_des(::motor_control_ros2::msg::DamiaoMotorCommand & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorCommand_vel_des pos_des(::motor_control_ros2::msg::DamiaoMotorCommand::_pos_des_type arg)
  {
    msg_.pos_des = std::move(arg);
    return Init_DamiaoMotorCommand_vel_des(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorCommand msg_;
};

class Init_DamiaoMotorCommand_joint_name
{
public:
  explicit Init_DamiaoMotorCommand_joint_name(::motor_control_ros2::msg::DamiaoMotorCommand & msg)
  : msg_(msg)
  {}
  Init_DamiaoMotorCommand_pos_des joint_name(::motor_control_ros2::msg::DamiaoMotorCommand::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_DamiaoMotorCommand_pos_des(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorCommand msg_;
};

class Init_DamiaoMotorCommand_header
{
public:
  Init_DamiaoMotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DamiaoMotorCommand_joint_name header(::motor_control_ros2::msg::DamiaoMotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DamiaoMotorCommand_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::DamiaoMotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::DamiaoMotorCommand>()
{
  return motor_control_ros2::msg::builder::Init_DamiaoMotorCommand_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__BUILDER_HPP_
