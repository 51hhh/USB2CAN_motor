// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:msg/MotorStateGeneric.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__motor_control_ros2__msg__MotorStateGeneric __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__msg__MotorStateGeneric __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorStateGeneric_
{
  using Type = MotorStateGeneric_<ContainerAllocator>;

  explicit MotorStateGeneric_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->motor_type = "";
      this->online = false;
      this->position = 0.0;
      this->velocity = 0.0;
      this->torque = 0.0;
      this->temperature = 0.0f;
    }
  }

  explicit MotorStateGeneric_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    joint_name(_alloc),
    motor_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->motor_type = "";
      this->online = false;
      this->position = 0.0;
      this->velocity = 0.0;
      this->torque = 0.0;
      this->temperature = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _motor_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor_type_type motor_type;
  using _online_type =
    bool;
  _online_type online;
  using _position_type =
    double;
  _position_type position;
  using _velocity_type =
    double;
  _velocity_type velocity;
  using _torque_type =
    double;
  _torque_type torque;
  using _temperature_type =
    float;
  _temperature_type temperature;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->joint_name = _arg;
    return *this;
  }
  Type & set__motor_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor_type = _arg;
    return *this;
  }
  Type & set__online(
    const bool & _arg)
  {
    this->online = _arg;
    return *this;
  }
  Type & set__position(
    const double & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const double & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__torque(
    const double & _arg)
  {
    this->torque = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__msg__MotorStateGeneric
    std::shared_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__msg__MotorStateGeneric
    std::shared_ptr<motor_control_ros2::msg::MotorStateGeneric_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorStateGeneric_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->motor_type != other.motor_type) {
      return false;
    }
    if (this->online != other.online) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorStateGeneric_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorStateGeneric_

// alias to use template instance with default allocator
using MotorStateGeneric =
  motor_control_ros2::msg::MotorStateGeneric_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__MOTOR_STATE_GENERIC__STRUCT_HPP_
