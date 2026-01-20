// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:msg/DamiaoMotorState.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__STRUCT_HPP_

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
# define DEPRECATED__motor_control_ros2__msg__DamiaoMotorState __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__msg__DamiaoMotorState __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DamiaoMotorState_
{
  using Type = DamiaoMotorState_<ContainerAllocator>;

  explicit DamiaoMotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->model = "";
      this->online = false;
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->torque = 0.0f;
      this->temp_mos = 0;
      this->temp_rotor = 0;
      this->error_code = 0;
    }
  }

  explicit DamiaoMotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->position = 0.0f;
      this->velocity = 0.0f;
      this->torque = 0.0f;
      this->temp_mos = 0;
      this->temp_rotor = 0;
      this->error_code = 0;
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
  using _position_type =
    float;
  _position_type position;
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _torque_type =
    float;
  _torque_type torque;
  using _temp_mos_type =
    uint8_t;
  _temp_mos_type temp_mos;
  using _temp_rotor_type =
    uint8_t;
  _temp_rotor_type temp_rotor;
  using _error_code_type =
    uint8_t;
  _error_code_type error_code;

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
  Type & set__temp_mos(
    const uint8_t & _arg)
  {
    this->temp_mos = _arg;
    return *this;
  }
  Type & set__temp_rotor(
    const uint8_t & _arg)
  {
    this->temp_rotor = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint8_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__msg__DamiaoMotorState
    std::shared_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__msg__DamiaoMotorState
    std::shared_ptr<motor_control_ros2::msg::DamiaoMotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DamiaoMotorState_ & other) const
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
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    if (this->temp_mos != other.temp_mos) {
      return false;
    }
    if (this->temp_rotor != other.temp_rotor) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    return true;
  }
  bool operator!=(const DamiaoMotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DamiaoMotorState_

// alias to use template instance with default allocator
using DamiaoMotorState =
  motor_control_ros2::msg::DamiaoMotorState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__DAMIAO_MOTOR_STATE__STRUCT_HPP_
