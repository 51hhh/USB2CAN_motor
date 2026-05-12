// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:msg/UnitreeGO8010Command.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__motor_control_ros2__msg__UnitreeGO8010Command __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__msg__UnitreeGO8010Command __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UnitreeGO8010Command_
{
  using Type = UnitreeGO8010Command_<ContainerAllocator>;

  explicit UnitreeGO8010Command_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->device = "";
      this->mode = 0;
      this->position_target = 0.0;
      this->velocity_target = 0.0;
      this->torque_ff = 0.0f;
      this->kp = 0.0f;
      this->kd = 0.0f;
    }
  }

  explicit UnitreeGO8010Command_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    device(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->device = "";
      this->mode = 0;
      this->position_target = 0.0;
      this->velocity_target = 0.0;
      this->torque_ff = 0.0f;
      this->kp = 0.0f;
      this->kd = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint8_t;
  _id_type id;
  using _device_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _device_type device;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _position_target_type =
    double;
  _position_target_type position_target;
  using _velocity_target_type =
    double;
  _velocity_target_type velocity_target;
  using _torque_ff_type =
    float;
  _torque_ff_type torque_ff;
  using _kp_type =
    float;
  _kp_type kp;
  using _kd_type =
    float;
  _kd_type kd;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__device(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->device = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__position_target(
    const double & _arg)
  {
    this->position_target = _arg;
    return *this;
  }
  Type & set__velocity_target(
    const double & _arg)
  {
    this->velocity_target = _arg;
    return *this;
  }
  Type & set__torque_ff(
    const float & _arg)
  {
    this->torque_ff = _arg;
    return *this;
  }
  Type & set__kp(
    const float & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const float & _arg)
  {
    this->kd = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MODE_BRAKE =
    0u;
  static constexpr uint8_t MODE_FOC =
    1u;
  static constexpr uint8_t MODE_CALIBRATE =
    2u;

  // pointer types
  using RawPtr =
    motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__msg__UnitreeGO8010Command
    std::shared_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__msg__UnitreeGO8010Command
    std::shared_ptr<motor_control_ros2::msg::UnitreeGO8010Command_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UnitreeGO8010Command_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->device != other.device) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->position_target != other.position_target) {
      return false;
    }
    if (this->velocity_target != other.velocity_target) {
      return false;
    }
    if (this->torque_ff != other.torque_ff) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    return true;
  }
  bool operator!=(const UnitreeGO8010Command_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UnitreeGO8010Command_

// alias to use template instance with default allocator
using UnitreeGO8010Command =
  motor_control_ros2::msg::UnitreeGO8010Command_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t UnitreeGO8010Command_<ContainerAllocator>::MODE_BRAKE;
template<typename ContainerAllocator>
constexpr uint8_t UnitreeGO8010Command_<ContainerAllocator>::MODE_FOC;
template<typename ContainerAllocator>
constexpr uint8_t UnitreeGO8010Command_<ContainerAllocator>::MODE_CALIBRATE;

}  // namespace msg

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_GO8010_COMMAND__STRUCT_HPP_
