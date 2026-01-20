// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_ros2:msg/UnitreeMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__STRUCT_HPP_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__motor_control_ros2__msg__UnitreeMotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_ros2__msg__UnitreeMotorCommand __declspec(deprecated)
#endif

namespace motor_control_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UnitreeMotorCommand_
{
  using Type = UnitreeMotorCommand_<ContainerAllocator>;

  explicit UnitreeMotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_name = "";
      this->leg_id = 0;
      this->motor_id = 0;
      this->mode = 0;
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->pos_des = 0.0f;
      this->vel_des = 0.0f;
      this->torque_ff = 0.0f;
    }
  }

  explicit UnitreeMotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->pos_des = 0.0f;
      this->vel_des = 0.0f;
      this->torque_ff = 0.0f;
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
  using _kp_type =
    float;
  _kp_type kp;
  using _kd_type =
    float;
  _kd_type kd;
  using _pos_des_type =
    float;
  _pos_des_type pos_des;
  using _vel_des_type =
    float;
  _vel_des_type vel_des;
  using _torque_ff_type =
    float;
  _torque_ff_type torque_ff;

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
  Type & set__pos_des(
    const float & _arg)
  {
    this->pos_des = _arg;
    return *this;
  }
  Type & set__vel_des(
    const float & _arg)
  {
    this->vel_des = _arg;
    return *this;
  }
  Type & set__torque_ff(
    const float & _arg)
  {
    this->torque_ff = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_ros2__msg__UnitreeMotorCommand
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_ros2__msg__UnitreeMotorCommand
    std::shared_ptr<motor_control_ros2::msg::UnitreeMotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UnitreeMotorCommand_ & other) const
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
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->pos_des != other.pos_des) {
      return false;
    }
    if (this->vel_des != other.vel_des) {
      return false;
    }
    if (this->torque_ff != other.torque_ff) {
      return false;
    }
    return true;
  }
  bool operator!=(const UnitreeMotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UnitreeMotorCommand_

// alias to use template instance with default allocator
using UnitreeMotorCommand =
  motor_control_ros2::msg::UnitreeMotorCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_ros2

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__UNITREE_MOTOR_COMMAND__STRUCT_HPP_
