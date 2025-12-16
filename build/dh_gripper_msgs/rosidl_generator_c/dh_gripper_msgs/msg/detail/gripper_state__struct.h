// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dh_gripper_msgs:msg/GripperState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__STRUCT_H_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GripperState in the package dh_gripper_msgs.
typedef struct dh_gripper_msgs__msg__GripperState
{
  std_msgs__msg__Header header;
  bool is_initialized;
  int32_t grip_state;
  float position;
  float target_position;
  float target_force;
} dh_gripper_msgs__msg__GripperState;

// Struct for a sequence of dh_gripper_msgs__msg__GripperState.
typedef struct dh_gripper_msgs__msg__GripperState__Sequence
{
  dh_gripper_msgs__msg__GripperState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dh_gripper_msgs__msg__GripperState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__STRUCT_H_
