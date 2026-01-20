// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:msg/UnitreeMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__STRUCT_HPP_

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
# define DEPRECATED__motor_control_ros2__msg__UnitreeMotorState __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__msg__UnitreeMotorState __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UnitreeMotorState_
{
  using Type = UnitreeMotorState_<ContainerAllocator>;

  explicit UnitreeMotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->leg_id = 0;
      this->motor_id = 0;
      this->mode = 0;
      this->online = false;
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->torque = 0.0f;
      this->acceleration = 0;
      this->temperature = 0;
      this->error = 0;
    }
  }

  explicit UnitreeMotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    joint_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->leg_id = 0;
      this->motor_id = 0;
      this->mode = 0;
      this->online = false;
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->torque = 0.0f;
      this->acceleration = 0;
      this->temperature = 0;
      this->error = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _joint_name_type joint_name;
  using _leg_id_type =
    uint8_t;
  _leg_id_type leg_id;
  using _motor_id_type =
    uint8_t;
  _motor_id_type motor_id;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _online_type =
    bool;
  _online_type online;
  using _position_type =
    float;
  _position_type position;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _torque_type =
    float;
  _torque_type torque;
  using _acceleration_type =
    int16_t;
  _acceleration_type acceleration;
  using _temperature_type =
    int8_t;
  _temperature_type temperature;
  using _error_type =
    int8_t;
  _error_type error;

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
  Type & set__leg_id(
    const uint8_t & _arg)
  {
    this->leg_id = _arg;
    return *this;
  }
  Type & set__motor_id(
    const uint8_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__online(
    const bool & _arg)
  {
    this->online = _arg;
    return *this;
  }
  Type & set__position(
    const float & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__torque(
    const float & _arg)
  {
    this->torque = _arg;
    return *this;
  }
  Type & set__acceleration(
    const int16_t & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__temperature(
    const int8_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__error(
    const int8_t & _arg)
  {
    this->error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__msg__UnitreeMotorState
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__msg__UnitreeMotorState
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UnitreeMotorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_name != other.joint_name) {
      return false;
    }
    if (this->leg_id != other.leg_id) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->mode != other.mode) {
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
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    return true;
  }
  bool operator!=(const UnitreeMotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UnitreeMotorState_

// alias to use template instance with default allocator
using UnitreeMotorState =
  motor_control_ros2::msg::UnitreeMotorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_STATE__STRUCT_HPP_
