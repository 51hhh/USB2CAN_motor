// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_control_ros2:msg/ControlFrequency.idl
// generated code does not contain a copyright notice
#include "motor_control_ros2/msg/detail/control_frequency__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
motor_control_ros2__msg__ControlFrequency__init(motor_control_ros2__msg__ControlFrequency * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    motor_control_ros2__msg__ControlFrequency__fini(msg);
    return false;
  }
  // control_frequency
  // can_tx_frequency
  // target_frequency
  return true;
}

void
motor_control_ros2__msg__ControlFrequency__fini(motor_control_ros2__msg__ControlFrequency * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // control_frequency
  // can_tx_frequency
  // target_frequency
}

bool
motor_control_ros2__msg__ControlFrequency__are_equal(const motor_control_ros2__msg__ControlFrequency * lhs, const motor_control_ros2__msg__ControlFrequency * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // control_frequency
  if (lhs->control_frequency != rhs->control_frequency) {
    return false;
  }
  // can_tx_frequency
  if (lhs->can_tx_frequency != rhs->can_tx_frequency) {
    return false;
  }
  // target_frequency
  if (lhs->target_frequency != rhs->target_frequency) {
    return false;
  }
  return true;
}

bool
motor_control_ros2__msg__ControlFrequency__copy(
  const motor_control_ros2__msg__ControlFrequency * input,
  motor_control_ros2__msg__ControlFrequency * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // control_frequency
  output->control_frequency = input->control_frequency;
  // can_tx_frequency
  output->can_tx_frequency = input->can_tx_frequency;
  // target_frequency
  output->target_frequency = input->target_frequency;
  return true;
}

motor_control_ros2__msg__ControlFrequency *
motor_control_ros2__msg__ControlFrequency__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__msg__ControlFrequency * msg = (motor_control_ros2__msg__ControlFrequency *)allocator.allocate(sizeof(motor_control_ros2__msg__ControlFrequency), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_control_ros2__msg__ControlFrequency));
  bool success = motor_control_ros2__msg__ControlFrequency__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_control_ros2__msg__ControlFrequency__destroy(motor_control_ros2__msg__ControlFrequency * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_control_ros2__msg__ControlFrequency__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_control_ros2__msg__ControlFrequency__Sequence__init(motor_control_ros2__msg__ControlFrequency__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__msg__ControlFrequency * data = NULL;

  if (size) {
    data = (motor_control_ros2__msg__ControlFrequency *)allocator.zero_allocate(size, sizeof(motor_control_ros2__msg__ControlFrequency), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_control_ros2__msg__ControlFrequency__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_control_ros2__msg__ControlFrequency__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
motor_control_ros2__msg__ControlFrequency__Sequence__fini(motor_control_ros2__msg__ControlFrequency__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      motor_control_ros2__msg__ControlFrequency__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

motor_control_ros2__msg__ControlFrequency__Sequence *
motor_control_ros2__msg__ControlFrequency__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__msg__ControlFrequency__Sequence * array = (motor_control_ros2__msg__ControlFrequency__Sequence *)allocator.allocate(sizeof(motor_control_ros2__msg__ControlFrequency__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_control_ros2__msg__ControlFrequency__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_control_ros2__msg__ControlFrequency__Sequence__destroy(motor_control_ros2__msg__ControlFrequency__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_control_ros2__msg__ControlFrequency__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_control_ros2__msg__ControlFrequency__Sequence__are_equal(const motor_control_ros2__msg__ControlFrequency__Sequence * lhs, const motor_control_ros2__msg__ControlFrequency__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_control_ros2__msg__ControlFrequency__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_control_ros2__msg__ControlFrequency__Sequence__copy(
  const motor_control_ros2__msg__ControlFrequency__Sequence * input,
  motor_control_ros2__msg__ControlFrequency__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_control_ros2__msg__ControlFrequency);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_control_ros2__msg__ControlFrequency * data =
      (motor_control_ros2__msg__ControlFrequency *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_control_ros2__msg__ControlFrequency__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_control_ros2__msg__ControlFrequency__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_control_ros2__msg__ControlFrequency__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
