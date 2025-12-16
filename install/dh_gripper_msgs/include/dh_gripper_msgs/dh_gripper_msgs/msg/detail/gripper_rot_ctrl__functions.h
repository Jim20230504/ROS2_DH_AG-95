// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dh_gripper_msgs:msg/GripperRotCtrl.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__FUNCTIONS_H_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dh_gripper_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dh_gripper_msgs/msg/detail/gripper_rot_ctrl__struct.h"

/// Initialize msg/GripperRotCtrl message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dh_gripper_msgs__msg__GripperRotCtrl
 * )) before or use
 * dh_gripper_msgs__msg__GripperRotCtrl__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
bool
dh_gripper_msgs__msg__GripperRotCtrl__init(dh_gripper_msgs__msg__GripperRotCtrl * msg);

/// Finalize msg/GripperRotCtrl message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
void
dh_gripper_msgs__msg__GripperRotCtrl__fini(dh_gripper_msgs__msg__GripperRotCtrl * msg);

/// Create msg/GripperRotCtrl message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dh_gripper_msgs__msg__GripperRotCtrl__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
dh_gripper_msgs__msg__GripperRotCtrl *
dh_gripper_msgs__msg__GripperRotCtrl__create();

/// Destroy msg/GripperRotCtrl message.
/**
 * It calls
 * dh_gripper_msgs__msg__GripperRotCtrl__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
void
dh_gripper_msgs__msg__GripperRotCtrl__destroy(dh_gripper_msgs__msg__GripperRotCtrl * msg);

/// Check for msg/GripperRotCtrl message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
bool
dh_gripper_msgs__msg__GripperRotCtrl__are_equal(const dh_gripper_msgs__msg__GripperRotCtrl * lhs, const dh_gripper_msgs__msg__GripperRotCtrl * rhs);

/// Copy a msg/GripperRotCtrl message.
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
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
bool
dh_gripper_msgs__msg__GripperRotCtrl__copy(
  const dh_gripper_msgs__msg__GripperRotCtrl * input,
  dh_gripper_msgs__msg__GripperRotCtrl * output);

/// Initialize array of msg/GripperRotCtrl messages.
/**
 * It allocates the memory for the number of elements and calls
 * dh_gripper_msgs__msg__GripperRotCtrl__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
bool
dh_gripper_msgs__msg__GripperRotCtrl__Sequence__init(dh_gripper_msgs__msg__GripperRotCtrl__Sequence * array, size_t size);

/// Finalize array of msg/GripperRotCtrl messages.
/**
 * It calls
 * dh_gripper_msgs__msg__GripperRotCtrl__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
void
dh_gripper_msgs__msg__GripperRotCtrl__Sequence__fini(dh_gripper_msgs__msg__GripperRotCtrl__Sequence * array);

/// Create array of msg/GripperRotCtrl messages.
/**
 * It allocates the memory for the array and calls
 * dh_gripper_msgs__msg__GripperRotCtrl__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
dh_gripper_msgs__msg__GripperRotCtrl__Sequence *
dh_gripper_msgs__msg__GripperRotCtrl__Sequence__create(size_t size);

/// Destroy array of msg/GripperRotCtrl messages.
/**
 * It calls
 * dh_gripper_msgs__msg__GripperRotCtrl__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
void
dh_gripper_msgs__msg__GripperRotCtrl__Sequence__destroy(dh_gripper_msgs__msg__GripperRotCtrl__Sequence * array);

/// Check for msg/GripperRotCtrl message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
bool
dh_gripper_msgs__msg__GripperRotCtrl__Sequence__are_equal(const dh_gripper_msgs__msg__GripperRotCtrl__Sequence * lhs, const dh_gripper_msgs__msg__GripperRotCtrl__Sequence * rhs);

/// Copy an array of msg/GripperRotCtrl messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dh_gripper_msgs
bool
dh_gripper_msgs__msg__GripperRotCtrl__Sequence__copy(
  const dh_gripper_msgs__msg__GripperRotCtrl__Sequence * input,
  dh_gripper_msgs__msg__GripperRotCtrl__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__FUNCTIONS_H_
