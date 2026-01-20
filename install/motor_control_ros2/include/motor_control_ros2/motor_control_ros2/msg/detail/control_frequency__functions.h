// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from motor_control_ros2:msg/ControlFrequency.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__FUNCTIONS_H_
#define MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "motor_control_ros2/msg/rosidl_generator_c__visibility_control.h"

#include "motor_control_ros2/msg/detail/control_frequency__struct.h"

/// Initialize msg/ControlFrequency message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motor_control_ros2__msg__ControlFrequency
 * )) before or use
 * motor_control_ros2__msg__ControlFrequency__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
bool
motor_control_ros2__msg__ControlFrequency__init(motor_control_ros2__msg__ControlFrequency * msg);

/// Finalize msg/ControlFrequency message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
void
motor_control_ros2__msg__ControlFrequency__fini(motor_control_ros2__msg__ControlFrequency * msg);

/// Create msg/ControlFrequency message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motor_control_ros2__msg__ControlFrequency__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
motor_control_ros2__msg__ControlFrequency *
motor_control_ros2__msg__ControlFrequency__create();

/// Destroy msg/ControlFrequency message.
/**
 * It calls
 * motor_control_ros2__msg__ControlFrequency__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
void
motor_control_ros2__msg__ControlFrequency__destroy(motor_control_ros2__msg__ControlFrequency * msg);

/// Check for msg/ControlFrequency message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
bool
motor_control_ros2__msg__ControlFrequency__are_equal(const motor_control_ros2__msg__ControlFrequency * lhs, const motor_control_ros2__msg__ControlFrequency * rhs);

/// Copy a msg/ControlFrequency message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
bool
motor_control_ros2__msg__ControlFrequency__copy(
  const motor_control_ros2__msg__ControlFrequency * input,
  motor_control_ros2__msg__ControlFrequency * output);

/// Initialize array of msg/ControlFrequency messages.
/**
 * It allocates the memory for the number of elements and calls
 * motor_control_ros2__msg__ControlFrequency__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
bool
motor_control_ros2__msg__ControlFrequency__Sequence__init(motor_control_ros2__msg__ControlFrequency__Sequence * array, size_t size);

/// Finalize array of msg/ControlFrequency messages.
/**
 * It calls
 * motor_control_ros2__msg__ControlFrequency__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
void
motor_control_ros2__msg__ControlFrequency__Sequence__fini(motor_control_ros2__msg__ControlFrequency__Sequence * array);

/// Create array of msg/ControlFrequency messages.
/**
 * It allocates the memory for the array and calls
 * motor_control_ros2__msg__ControlFrequency__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
motor_control_ros2__msg__ControlFrequency__Sequence *
motor_control_ros2__msg__ControlFrequency__Sequence__create(size_t size);

/// Destroy array of msg/ControlFrequency messages.
/**
 * It calls
 * motor_control_ros2__msg__ControlFrequency__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
void
motor_control_ros2__msg__ControlFrequency__Sequence__destroy(motor_control_ros2__msg__ControlFrequency__Sequence * array);

/// Check for msg/ControlFrequency message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
bool
motor_control_ros2__msg__ControlFrequency__Sequence__are_equal(const motor_control_ros2__msg__ControlFrequency__Sequence * lhs, const motor_control_ros2__msg__ControlFrequency__Sequence * rhs);

/// Copy an array of msg/ControlFrequency messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_ros2
bool
motor_control_ros2__msg__ControlFrequency__Sequence__copy(
  const motor_control_ros2__msg__ControlFrequency__Sequence * input,
  motor_control_ros2__msg__ControlFrequency__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_ROS2__MSG__DETAIL__CONTROL_FREQUENCY__FUNCTIONS_H_
