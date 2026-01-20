// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:msg/DJIMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__STRUCT_HPP_

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
# define DEPRECATED__motor_control_ros2__msg__DJIMotorState __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__msg__DJIMotorState __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DJIMotorState_
{
  using Type = DJIMotorState_<ContainerAllocator>;

  explicit DJIMotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->model = "";
      this->online = false;
      this->angle = 0.0;
      this->rpm = 0;
      this->current = 0;
      this->temperature = 0;
      this->control_frequency = 0.0;
    }
  }

  explicit DJIMotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    joint_name(_alloc),
    model(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->model = "";
      this->online = false;
      this->angle = 0.0;
      this->rpm = 0;
      this->current = 0;
      this->temperature = 0;
      this->control_frequency = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _model_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _model_type model;
  using _online_type =
    bool;
  _online_type online;
  using _angle_type =
    double;
  _angle_type angle;
  using _rpm_type =
    int16_t;
  _rpm_type rpm;
  using _current_type =
    int16_t;
  _current_type current;
  using _temperature_type =
    uint8_t;
  _temperature_type temperature;
  using _control_frequency_type =
    double;
  _control_frequency_type control_frequency;

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
  Type & set__model(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->model = _arg;
    return *this;
  }
  Type & set__online(
    const bool & _arg)
  {
    this->online = _arg;
    return *this;
  }
  Type & set__angle(
    const double & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__rpm(
    const int16_t & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__current(
    const int16_t & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__temperature(
    const uint8_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__control_frequency(
    const double & _arg)
  {
    this->control_frequency = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::msg::DJIMotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::msg::DJIMotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::DJIMotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::DJIMotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__msg__DJIMotorState
    std::shared_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__msg__DJIMotorState
    std::shared_ptr<motor_control_ros2::msg::DJIMotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DJIMotorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->model != other.model) {
      return false;
    }
    if (this->online != other.online) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->control_frequency != other.control_frequency) {
      return false;
    }
    return true;
  }
  bool operator!=(const DJIMotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DJIMotorState_

// alias to use template instance with default allocator
using DJIMotorState =
  motor_control_ros2::msg::DJIMotorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DJI_MOTOR_STATE__STRUCT_HPP_
