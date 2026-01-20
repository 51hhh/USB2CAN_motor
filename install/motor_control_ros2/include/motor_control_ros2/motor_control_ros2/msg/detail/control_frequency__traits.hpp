// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_control_ros2:msg/ControlFrequency.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__TRAITS_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_control_ros2/msg/detail/control_frequency__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace motor_control_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlFrequency & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: control_frequency
  {
    out << "control_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.control_frequency, out);
    out << ", ";
  }

  // member: can_tx_frequency
  {
    out << "can_tx_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.can_tx_frequency, out);
    out << ", ";
  }

  // member: target_frequency
  {
    out << "target_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.target_frequency, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlFrequency & msg,
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

  // member: control_frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.control_frequency, out);
    out << "\n";
  }

  // member: can_tx_frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "can_tx_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.can_tx_frequency, out);
    out << "\n";
  }

  // member: target_frequency
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_frequency: ";
    rosidl_generator_traits::value_to_yaml(msg.target_frequency, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlFrequency & msg, bool use_flow_style = false)
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
  const motor_control_ros2::msg::ControlFrequency & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_control_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_control_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_control_ros2::msg::ControlFrequency & msg)
{
  return motor_control_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_control_ros2::msg::ControlFrequency>()
{
  return "motor_control_ros2::msg::ControlFrequency";
}

template<>
inline const char * name<motor_control_ros2::msg::ControlFrequency>()
{
  return "motor_control_ros2/msg/ControlFrequency";
}

template<>
struct has_fixed_size<motor_control_ros2::msg::ControlFrequency>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<motor_control_ros2::msg::ControlFrequency>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<motor_control_ros2::msg::ControlFrequency>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__TRAITS_HPP_
