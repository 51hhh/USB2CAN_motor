// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_control_ros2:msg/DamiaoMotorState.idl
// generated code does not contain a copyright notice
#include "motor_control_ros2/msg/detail/damiao_motor_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint_name`
// Member `model`
#include "rosidl_runtime_c/string_functions.h"

bool
motor_control_ros2__msg__DamiaoMotorState__init(motor_control_ros2__msg__DamiaoMotorState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    motor_control_ros2__msg__DamiaoMotorState__fini(msg);
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__init(&msg->joint_name)) {
    motor_control_ros2__msg__DamiaoMotorState__fini(msg);
    return false;
  }
  // model
  if (!rosidl_runtime_c__String__init(&msg->model)) {
    motor_control_ros2__msg__DamiaoMotorState__fini(msg);
    return false;
  }
  // online
  // position
  // velocity
  // torque
  // temp_mos
  // temp_rotor
  // error_code
  return true;
}

void
motor_control_ros2__msg__DamiaoMotorState__fini(motor_control_ros2__msg__DamiaoMotorState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // joint_name
  rosidl_runtime_c__String__fini(&msg->joint_name);
  // model
  rosidl_runtime_c__String__fini(&msg->model);
  // online
  // position
  // velocity
  // torque
  // temp_mos
  // temp_rotor
  // error_code
}

bool
motor_control_ros2__msg__DamiaoMotorState__are_equal(const motor_control_ros2__msg__DamiaoMotorState * lhs, const motor_control_ros2__msg__DamiaoMotorState * rhs)
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
  // joint_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->joint_name), &(rhs->joint_name)))
  {
    return false;
  }
  // model
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->model), &(rhs->model)))
  {
    return false;
  }
  // online
  if (lhs->online != rhs->online) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // torque
  if (lhs->torque != rhs->torque) {
    return false;
  }
  // temp_mos
  if (lhs->temp_mos != rhs->temp_mos) {
    return false;
  }
  // temp_rotor
  if (lhs->temp_rotor != rhs->temp_rotor) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  return true;
}

bool
motor_control_ros2__msg__DamiaoMotorState__copy(
  const motor_control_ros2__msg__DamiaoMotorState * input,
  motor_control_ros2__msg__DamiaoMotorState * output)
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
  // joint_name
  if (!rosidl_runtime_c__String__copy(
      &(input->joint_name), &(output->joint_name)))
  {
    return false;
  }
  // model
  if (!rosidl_runtime_c__String__copy(
      &(input->model), &(output->model)))
  {
    return false;
  }
  // online
  output->online = input->online;
  // position
  output->position = input->position;
  // velocity
  output->velocity = input->velocity;
  // torque
  output->torque = input->torque;
  // temp_mos
  output->temp_mos = input->temp_mos;
  // temp_rotor
  output->temp_rotor = input->temp_rotor;
  // error_code
  output->error_code = input->error_code;
  return true;
}

motor_control_ros2__msg__DamiaoMotorState *
motor_control_ros2__msg__DamiaoMotorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__msg__DamiaoMotorState * msg = (motor_control_ros2__msg__DamiaoMotorState *)allocator.allocate(sizeof(motor_control_ros2__msg__DamiaoMotorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_control_ros2__msg__DamiaoMotorState));
  bool success = motor_control_ros2__msg__DamiaoMotorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_control_ros2__msg__DamiaoMotorState__destroy(motor_control_ros2__msg__DamiaoMotorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_control_ros2__msg__DamiaoMotorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_control_ros2__msg__DamiaoMotorState__Sequence__init(motor_control_ros2__msg__DamiaoMotorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__msg__DamiaoMotorState * data = NULL;

  if (size) {
    data = (motor_control_ros2__msg__DamiaoMotorState *)allocator.zero_allocate(size, sizeof(motor_control_ros2__msg__DamiaoMotorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_control_ros2__msg__DamiaoMotorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_control_ros2__msg__DamiaoMotorState__fini(&data[i - 1]);
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
motor_control_ros2__msg__DamiaoMotorState__Sequence__fini(motor_control_ros2__msg__DamiaoMotorState__Sequence * array)
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
      motor_control_ros2__msg__DamiaoMotorState__fini(&array->data[i]);
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

motor_control_ros2__msg__DamiaoMotorState__Sequence *
motor_control_ros2__msg__DamiaoMotorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_control_ros2__msg__DamiaoMotorState__Sequence * array = (motor_control_ros2__msg__DamiaoMotorState__Sequence *)allocator.allocate(sizeof(motor_control_ros2__msg__DamiaoMotorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_control_ros2__msg__DamiaoMotorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_control_ros2__msg__DamiaoMotorState__Sequence__destroy(motor_control_ros2__msg__DamiaoMotorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_control_ros2__msg__DamiaoMotorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_control_ros2__msg__DamiaoMotorState__Sequence__are_equal(const motor_control_ros2__msg__DamiaoMotorState__Sequence * lhs, const motor_control_ros2__msg__DamiaoMotorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_control_ros2__msg__DamiaoMotorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_control_ros2__msg__DamiaoMotorState__Sequence__copy(
  const motor_control_ros2__msg__DamiaoMotorState__Sequence * input,
  motor_control_ros2__msg__DamiaoMotorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_control_ros2__msg__DamiaoMotorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_control_ros2__msg__DamiaoMotorState * data =
      (motor_control_ros2__msg__DamiaoMotorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_control_ros2__msg__DamiaoMotorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_control_ros2__msg__DamiaoMotorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_control_ros2__msg__DamiaoMotorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
