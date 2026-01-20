// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:msg/ControlFrequency.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__STRUCT_HPP_

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
# define DEPRECATED__motor_control_ros2__msg__ControlFrequency __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__msg__ControlFrequency __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControlFrequency_
{
  using Type = ControlFrequency_<ContainerAllocator>;

  explicit ControlFrequency_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_frequency = 0.0;
      this->can_tx_frequency = 0.0;
      this->target_frequency = 0.0;
    }
  }

  explicit ControlFrequency_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->control_frequency = 0.0;
      this->can_tx_frequency = 0.0;
      this->target_frequency = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _control_frequency_type =
    double;
  _control_frequency_type control_frequency;
  using _can_tx_frequency_type =
    double;
  _can_tx_frequency_type can_tx_frequency;
  using _target_frequency_type =
    double;
  _target_frequency_type target_frequency;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__control_frequency(
    const double & _arg)
  {
    this->control_frequency = _arg;
    return *this;
  }
  Type & set__can_tx_frequency(
    const double & _arg)
  {
    this->can_tx_frequency = _arg;
    return *this;
  }
  Type & set__target_frequency(
    const double & _arg)
  {
    this->target_frequency = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::msg::ControlFrequency_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::msg::ControlFrequency_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::ControlFrequency_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::ControlFrequency_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__msg__ControlFrequency
    std::shared_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__msg__ControlFrequency
    std::shared_ptr<motor_control_ros2::msg::ControlFrequency_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlFrequency_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->control_frequency != other.control_frequency) {
      return false;
    }
    if (this->can_tx_frequency != other.can_tx_frequency) {
      return false;
    }
    if (this->target_frequency != other.target_frequency) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlFrequency_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlFrequency_

// alias to use template instance with default allocator
using ControlFrequency =
  motor_control_ros2::msg::ControlFrequency_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__STRUCT_HPP_
