// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dh_gripper_msgs:msg/GripperRotState.idl
// generated code does not contain a copyright notice
#include "dh_gripper_msgs/msg/detail/gripper_rot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
dh_gripper_msgs__msg__GripperRotState__init(dh_gripper_msgs__msg__GripperRotState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    dh_gripper_msgs__msg__GripperRotState__fini(msg);
    return false;
  }
  // rot_state
  // angle
  // target_angle
  // target_force
  return true;
}

void
dh_gripper_msgs__msg__GripperRotState__fini(dh_gripper_msgs__msg__GripperRotState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rot_state
  // angle
  // target_angle
  // target_force
}

bool
dh_gripper_msgs__msg__GripperRotState__are_equal(const dh_gripper_msgs__msg__GripperRotState * lhs, const dh_gripper_msgs__msg__GripperRotState * rhs)
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
  // rot_state
  if (lhs->rot_state != rhs->rot_state) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // target_angle
  if (lhs->target_angle != rhs->target_angle) {
    return false;
  }
  // target_force
  if (lhs->target_force != rhs->target_force) {
    return false;
  }
  return true;
}

bool
dh_gripper_msgs__msg__GripperRotState__copy(
  const dh_gripper_msgs__msg__GripperRotState * input,
  dh_gripper_msgs__msg__GripperRotState * output)
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
  // rot_state
  output->rot_state = input->rot_state;
  // angle
  output->angle = input->angle;
  // target_angle
  output->target_angle = input->target_angle;
  // target_force
  output->target_force = input->target_force;
  return true;
}

dh_gripper_msgs__msg__GripperRotState *
dh_gripper_msgs__msg__GripperRotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperRotState * msg = (dh_gripper_msgs__msg__GripperRotState *)allocator.allocate(sizeof(dh_gripper_msgs__msg__GripperRotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dh_gripper_msgs__msg__GripperRotState));
  bool success = dh_gripper_msgs__msg__GripperRotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dh_gripper_msgs__msg__GripperRotState__destroy(dh_gripper_msgs__msg__GripperRotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dh_gripper_msgs__msg__GripperRotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dh_gripper_msgs__msg__GripperRotState__Sequence__init(dh_gripper_msgs__msg__GripperRotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperRotState * data = NULL;

  if (size) {
    data = (dh_gripper_msgs__msg__GripperRotState *)allocator.zero_allocate(size, sizeof(dh_gripper_msgs__msg__GripperRotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dh_gripper_msgs__msg__GripperRotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dh_gripper_msgs__msg__GripperRotState__fini(&data[i - 1]);
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
dh_gripper_msgs__msg__GripperRotState__Sequence__fini(dh_gripper_msgs__msg__GripperRotState__Sequence * array)
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
      dh_gripper_msgs__msg__GripperRotState__fini(&array->data[i]);
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

dh_gripper_msgs__msg__GripperRotState__Sequence *
dh_gripper_msgs__msg__GripperRotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperRotState__Sequence * array = (dh_gripper_msgs__msg__GripperRotState__Sequence *)allocator.allocate(sizeof(dh_gripper_msgs__msg__GripperRotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dh_gripper_msgs__msg__GripperRotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dh_gripper_msgs__msg__GripperRotState__Sequence__destroy(dh_gripper_msgs__msg__GripperRotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dh_gripper_msgs__msg__GripperRotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dh_gripper_msgs__msg__GripperRotState__Sequence__are_equal(const dh_gripper_msgs__msg__GripperRotState__Sequence * lhs, const dh_gripper_msgs__msg__GripperRotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dh_gripper_msgs__msg__GripperRotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dh_gripper_msgs__msg__GripperRotState__Sequence__copy(
  const dh_gripper_msgs__msg__GripperRotState__Sequence * input,
  dh_gripper_msgs__msg__GripperRotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dh_gripper_msgs__msg__GripperRotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dh_gripper_msgs__msg__GripperRotState * data =
      (dh_gripper_msgs__msg__GripperRotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dh_gripper_msgs__msg__GripperRotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dh_gripper_msgs__msg__GripperRotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dh_gripper_msgs__msg__GripperRotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
