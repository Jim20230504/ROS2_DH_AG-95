// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dh_gripper_msgs:msg/GripperRotState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__STRUCT_H_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__STRUCT_H_

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

/// Struct defined in msg/GripperRotState in the package dh_gripper_msgs.
typedef struct dh_gripper_msgs__msg__GripperRotState
{
  std_msgs__msg__Header header;
  int32_t rot_state;
  float angle;
  float target_angle;
  float target_force;
} dh_gripper_msgs__msg__GripperRotState;

// Struct for a sequence of dh_gripper_msgs__msg__GripperRotState.
typedef struct dh_gripper_msgs__msg__GripperRotState__Sequence
{
  dh_gripper_msgs__msg__GripperRotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dh_gripper_msgs__msg__GripperRotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__STRUCT_H_
