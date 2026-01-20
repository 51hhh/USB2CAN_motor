// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_control_ros2:msg/DamiaoMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__TRAITS_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_control_ros2/msg/detail/damiao_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace motor_control_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const DamiaoMotorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: joint_name
  {
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << ", ";
  }

  // member: pos_des
  {
    out << "pos_des: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_des, out);
    out << ", ";
  }

  // member: vel_des
  {
    out << "vel_des: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_des, out);
    out << ", ";
  }

  // member: kp
  {
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << ", ";
  }

  // member: kd
  {
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << ", ";
  }

  // member: torque_ff
  {
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DamiaoMotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: joint_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_name: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_name, out);
    out << "\n";
  }

  // member: pos_des
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_des: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_des, out);
    out << "\n";
  }

  // member: vel_des
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_des: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_des, out);
    out << "\n";
  }

  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << "\n";
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << "\n";
  }

  // member: torque_ff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DamiaoMotorCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace motor_control_ros2

namespace rosidl_generator_traits
{

[[deprecated("use motor_control_ros2::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_control_ros2::msg::DamiaoMotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_control_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_control_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_control_ros2::msg::DamiaoMotorCommand & msg)
{
  return motor_control_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_control_ros2::msg::DamiaoMotorCommand>()
{
  return "motor_control_ros2::msg::DamiaoMotorCommand";
}

template<>
inline const char * name<motor_control_ros2::msg::DamiaoMotorCommand>()
{
  return "motor_control_ros2/msg/DamiaoMotorCommand";
}

template<>
struct has_fixed_size<motor_control_ros2::msg::DamiaoMotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_control_ros2::msg::DamiaoMotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_control_ros2::msg::DamiaoMotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_COMMAND__TRAITS_HPP_
