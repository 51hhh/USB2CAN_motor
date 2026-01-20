// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_control_ros2:msg/DamiaoMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__TRAITS_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_control_ros2/msg/detail/damiao_motor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace motor_control_ros2
{

namespace msg
{

inline void to_flow_style_yaml(
  const DamiaoMotorState & msg,
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

  // member: model
  {
    out << "model: ";
    rosidl_generator_traits::value_to_yaml(msg.model, out);
    out << ", ";
  }

  // member: online
  {
    out << "online: ";
    rosidl_generator_traits::value_to_yaml(msg.online, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: torque
  {
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
    out << ", ";
  }

  // member: temp_mos
  {
    out << "temp_mos: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_mos, out);
    out << ", ";
  }

  // member: temp_rotor
  {
    out << "temp_rotor: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_rotor, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DamiaoMotorState & msg,
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

  // member: model
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "model: ";
    rosidl_generator_traits::value_to_yaml(msg.model, out);
    out << "\n";
  }

  // member: online
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "online: ";
    rosidl_generator_traits::value_to_yaml(msg.online, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque: ";
    rosidl_generator_traits::value_to_yaml(msg.torque, out);
    out << "\n";
  }

  // member: temp_mos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temp_mos: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_mos, out);
    out << "\n";
  }

  // member: temp_rotor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temp_rotor: ";
    rosidl_generator_traits::value_to_yaml(msg.temp_rotor, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DamiaoMotorState & msg, bool use_flow_style = false)
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
  const motor_control_ros2::msg::DamiaoMotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_control_ros2::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_control_ros2::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_control_ros2::msg::DamiaoMotorState & msg)
{
  return motor_control_ros2::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_control_ros2::msg::DamiaoMotorState>()
{
  return "motor_control_ros2::msg::DamiaoMotorState";
}

template<>
inline const char * name<motor_control_ros2::msg::DamiaoMotorState>()
{
  return "motor_control_ros2/msg/DamiaoMotorState";
}

template<>
struct has_fixed_size<motor_control_ros2::msg::DamiaoMotorState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_control_ros2::msg::DamiaoMotorState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_control_ros2::msg::DamiaoMotorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__TRAITS_HPP_
