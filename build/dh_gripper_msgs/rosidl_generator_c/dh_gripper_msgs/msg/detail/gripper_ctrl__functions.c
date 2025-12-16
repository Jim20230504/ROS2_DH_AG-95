// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dh_gripper_msgs:msg/GripperCtrl.idl
// generated code does not contain a copyright notice
#include "dh_gripper_msgs/msg/detail/gripper_ctrl__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
dh_gripper_msgs__msg__GripperCtrl__init(dh_gripper_msgs__msg__GripperCtrl * msg)
{
  if (!msg) {
    return false;
  }
  // initialize
  // position
  // force
  // speed
  return true;
}

void
dh_gripper_msgs__msg__GripperCtrl__fini(dh_gripper_msgs__msg__GripperCtrl * msg)
{
  if (!msg) {
    return;
  }
  // initialize
  // position
  // force
  // speed
}

bool
dh_gripper_msgs__msg__GripperCtrl__are_equal(const dh_gripper_msgs__msg__GripperCtrl * lhs, const dh_gripper_msgs__msg__GripperCtrl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // initialize
  if (lhs->initialize != rhs->initialize) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // force
  if (lhs->force != rhs->force) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
dh_gripper_msgs__msg__GripperCtrl__copy(
  const dh_gripper_msgs__msg__GripperCtrl * input,
  dh_gripper_msgs__msg__GripperCtrl * output)
{
  if (!input || !output) {
    return false;
  }
  // initialize
  output->initialize = input->initialize;
  // position
  output->position = input->position;
  // force
  output->force = input->force;
  // speed
  output->speed = input->speed;
  return true;
}

dh_gripper_msgs__msg__GripperCtrl *
dh_gripper_msgs__msg__GripperCtrl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperCtrl * msg = (dh_gripper_msgs__msg__GripperCtrl *)allocator.allocate(sizeof(dh_gripper_msgs__msg__GripperCtrl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dh_gripper_msgs__msg__GripperCtrl));
  bool success = dh_gripper_msgs__msg__GripperCtrl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dh_gripper_msgs__msg__GripperCtrl__destroy(dh_gripper_msgs__msg__GripperCtrl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dh_gripper_msgs__msg__GripperCtrl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dh_gripper_msgs__msg__GripperCtrl__Sequence__init(dh_gripper_msgs__msg__GripperCtrl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperCtrl * data = NULL;

  if (size) {
    data = (dh_gripper_msgs__msg__GripperCtrl *)allocator.zero_allocate(size, sizeof(dh_gripper_msgs__msg__GripperCtrl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dh_gripper_msgs__msg__GripperCtrl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dh_gripper_msgs__msg__GripperCtrl__fini(&data[i - 1]);
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
dh_gripper_msgs__msg__GripperCtrl__Sequence__fini(dh_gripper_msgs__msg__GripperCtrl__Sequence * array)
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
      dh_gripper_msgs__msg__GripperCtrl__fini(&array->data[i]);
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

dh_gripper_msgs__msg__GripperCtrl__Sequence *
dh_gripper_msgs__msg__GripperCtrl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dh_gripper_msgs__msg__GripperCtrl__Sequence * array = (dh_gripper_msgs__msg__GripperCtrl__Sequence *)allocator.allocate(sizeof(dh_gripper_msgs__msg__GripperCtrl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dh_gripper_msgs__msg__GripperCtrl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dh_gripper_msgs__msg__GripperCtrl__Sequence__destroy(dh_gripper_msgs__msg__GripperCtrl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dh_gripper_msgs__msg__GripperCtrl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dh_gripper_msgs__msg__GripperCtrl__Sequence__are_equal(const dh_gripper_msgs__msg__GripperCtrl__Sequence * lhs, const dh_gripper_msgs__msg__GripperCtrl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dh_gripper_msgs__msg__GripperCtrl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dh_gripper_msgs__msg__GripperCtrl__Sequence__copy(
  const dh_gripper_msgs__msg__GripperCtrl__Sequence * input,
  dh_gripper_msgs__msg__GripperCtrl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dh_gripper_msgs__msg__GripperCtrl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dh_gripper_msgs__msg__GripperCtrl * data =
      (dh_gripper_msgs__msg__GripperCtrl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dh_gripper_msgs__msg__GripperCtrl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dh_gripper_msgs__msg__GripperCtrl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dh_gripper_msgs__msg__GripperCtrl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
