// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dh_gripper_msgs:msg/GripperState.idl
// generated code does not contain a copyright notice
#include "dh_gripper_msgs/msg/detail/gripper_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
dh_gripper_msgs__msg__GripperState__init(dh_gripper_msgs__msg__GripperState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    dh_gripper_msgs__msg__GripperState__fini(msg);
    return false;
  }
  // is_initialized
  // grip_state
  // position
  // target_position
  // target_force
  return true;
}

void
dh_gripper_msgs__msg__GripperState__fini(dh_gripper_msgs__msg__GripperState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_initialized
  // grip_state
  // position
  // target_position
  // target_force
}

bool
dh_gripper_msgs__msg__GripperState__are_equal(const dh_gripper_msgs__msg__GripperState * lhs, const dh_gripper_msgs__msg__GripperState * rhs)
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
  // is_initialized
  if (lhs->is_initialized != rhs->is_initialized) {
    return false;
  }
  // grip_state
  if (lhs->grip_state != rhs->grip_state) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // target_position
  if (lhs->target_position != rhs->target_position) {
    return false;
  }
  // target_force
  if (lhs->target_force != rhs->target_force) {
    return false;
  }
  return true;
}

bool
dh_gripper_msgs__msg__GripperState__copy(
  const dh_gripper_msgs__msg__GripperState * input,
  dh_gripper_msgs__msg__GripperState * output)
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
  // is_initialized
  output->is_initialized = input->is_initialized;
  // grip_state
  output->grip_state = input->grip_state;
  // position
  output->position = input->position;
  // target_position
  output->target_position = input->target_position;
  // target_force
  output->target_force = input->target_force;
  return true;
}

dh_gripper_msgs__msg__GripperState *
dh_gripper_msgs__msg__GripperState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperState * msg = (dh_gripper_msgs__msg__GripperState *)allocator.allocate(sizeof(dh_gripper_msgs__msg__GripperState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dh_gripper_msgs__msg__GripperState));
  bool success = dh_gripper_msgs__msg__GripperState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dh_gripper_msgs__msg__GripperState__destroy(dh_gripper_msgs__msg__GripperState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dh_gripper_msgs__msg__GripperState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dh_gripper_msgs__msg__GripperState__Sequence__init(dh_gripper_msgs__msg__GripperState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperState * data = NULL;

  if (size) {
    data = (dh_gripper_msgs__msg__GripperState *)allocator.zero_allocate(size, sizeof(dh_gripper_msgs__msg__GripperState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dh_gripper_msgs__msg__GripperState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dh_gripper_msgs__msg__GripperState__fini(&data[i - 1]);
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
dh_gripper_msgs__msg__GripperState__Sequence__fini(dh_gripper_msgs__msg__GripperState__Sequence * array)
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
      dh_gripper_msgs__msg__GripperState__fini(&array->data[i]);
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

dh_gripper_msgs__msg__GripperState__Sequence *
dh_gripper_msgs__msg__GripperState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperState__Sequence * array = (dh_gripper_msgs__msg__GripperState__Sequence *)allocator.allocate(sizeof(dh_gripper_msgs__msg__GripperState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dh_gripper_msgs__msg__GripperState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dh_gripper_msgs__msg__GripperState__Sequence__destroy(dh_gripper_msgs__msg__GripperState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dh_gripper_msgs__msg__GripperState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dh_gripper_msgs__msg__GripperState__Sequence__are_equal(const dh_gripper_msgs__msg__GripperState__Sequence * lhs, const dh_gripper_msgs__msg__GripperState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dh_gripper_msgs__msg__GripperState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dh_gripper_msgs__msg__GripperState__Sequence__copy(
  const dh_gripper_msgs__msg__GripperState__Sequence * input,
  dh_gripper_msgs__msg__GripperState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dh_gripper_msgs__msg__GripperState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dh_gripper_msgs__msg__GripperState * data =
      (dh_gripper_msgs__msg__GripperState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dh_gripper_msgs__msg__GripperState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dh_gripper_msgs__msg__GripperState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dh_gripper_msgs__msg__GripperState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
