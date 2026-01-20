// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_ros2:msg/UnitreeMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__BUILDER_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_ros2/msg/detail/unitree_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_ros2
{

namespace msg
{

namespace builder
{

class Init_UnitreeMotorCommand_torque_ff
{
public:
  explicit Init_UnitreeMotorCommand_torque_ff(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  ::motor_control_ros2::msg::UnitreeMotorCommand torque_ff(::motor_control_ros2::msg::UnitreeMotorCommand::_torque_ff_type arg)
  {
    msg_.torque_ff = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_vel_des
{
public:
  explicit Init_UnitreeMotorCommand_vel_des(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_torque_ff vel_des(::motor_control_ros2::msg::UnitreeMotorCommand::_vel_des_type arg)
  {
    msg_.vel_des = std::move(arg);
    return Init_UnitreeMotorCommand_torque_ff(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_pos_des
{
public:
  explicit Init_UnitreeMotorCommand_pos_des(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_vel_des pos_des(::motor_control_ros2::msg::UnitreeMotorCommand::_pos_des_type arg)
  {
    msg_.pos_des = std::move(arg);
    return Init_UnitreeMotorCommand_vel_des(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_kd
{
public:
  explicit Init_UnitreeMotorCommand_kd(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_pos_des kd(::motor_control_ros2::msg::UnitreeMotorCommand::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_UnitreeMotorCommand_pos_des(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_kp
{
public:
  explicit Init_UnitreeMotorCommand_kp(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_kd kp(::motor_control_ros2::msg::UnitreeMotorCommand::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_UnitreeMotorCommand_kd(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_mode
{
public:
  explicit Init_UnitreeMotorCommand_mode(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_kp mode(::motor_control_ros2::msg::UnitreeMotorCommand::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_UnitreeMotorCommand_kp(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_motor_id
{
public:
  explicit Init_UnitreeMotorCommand_motor_id(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_mode motor_id(::motor_control_ros2::msg::UnitreeMotorCommand::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_UnitreeMotorCommand_mode(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_leg_id
{
public:
  explicit Init_UnitreeMotorCommand_leg_id(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_motor_id leg_id(::motor_control_ros2::msg::UnitreeMotorCommand::_leg_id_type arg)
  {
    msg_.leg_id = std::move(arg);
    return Init_UnitreeMotorCommand_motor_id(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_joint_name
{
public:
  explicit Init_UnitreeMotorCommand_joint_name(::motor_control_ros2::msg::UnitreeMotorCommand & msg)
  : msg_(msg)
  {}
  Init_UnitreeMotorCommand_leg_id joint_name(::motor_control_ros2::msg::UnitreeMotorCommand::_joint_name_type arg)
  {
    msg_.joint_name = std::move(arg);
    return Init_UnitreeMotorCommand_leg_id(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

class Init_UnitreeMotorCommand_header
{
public:
  Init_UnitreeMotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UnitreeMotorCommand_joint_name header(::motor_control_ros2::msg::UnitreeMotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_UnitreeMotorCommand_joint_name(msg_);
  }

private:
  ::motor_control_ros2::msg::UnitreeMotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_ros2::msg::UnitreeMotorCommand>()
{
  return motor_control_ros2::msg::builder::Init_UnitreeMotorCommand_header();
}

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__BUILDER_HPP_
