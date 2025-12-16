// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dh_gripper_msgs:msg/GripperRotCtrl.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__STRUCT_H_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GripperRotCtrl in the package dh_gripper_msgs.
typedef struct dh_gripper_msgs__msg__GripperRotCtrl
{
  float angle;
  float force;
  float speed;
} dh_gripper_msgs__msg__GripperRotCtrl;

// Struct for a sequence of dh_gripper_msgs__msg__GripperRotCtrl.
typedef struct dh_gripper_msgs__msg__GripperRotCtrl__Sequence
{
  dh_gripper_msgs__msg__GripperRotCtrl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dh_gripper_msgs__msg__GripperRotCtrl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__STRUCT_H_
