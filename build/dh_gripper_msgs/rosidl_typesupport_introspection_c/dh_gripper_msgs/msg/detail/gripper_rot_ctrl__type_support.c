// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dh_gripper_msgs:msg/GripperRotCtrl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dh_gripper_msgs/msg/detail/gripper_rot_ctrl__rosidl_typesupport_introspection_c.h"
#include "dh_gripper_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dh_gripper_msgs/msg/detail/gripper_rot_ctrl__functions.h"
#include "dh_gripper_msgs/msg/detail/gripper_rot_ctrl__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dh_gripper_msgs__msg__GripperRotCtrl__init(message_memory);
}

void dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_fini_function(void * message_memory)
{
  dh_gripper_msgs__msg__GripperRotCtrl__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_member_array[3] = {
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperRotCtrl, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperRotCtrl, force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperRotCtrl, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_members = {
  "dh_gripper_msgs__msg",  // message namespace
  "GripperRotCtrl",  // message name
  3,  // number of fields
  sizeof(dh_gripper_msgs__msg__GripperRotCtrl),
  dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_member_array,  // message members
  dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_init_function,  // function to initialize message memory (memory has to be allocated)
  dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_type_support_handle = {
  0,
  &dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dh_gripper_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dh_gripper_msgs, msg, GripperRotCtrl)() {
  if (!dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_type_support_handle.typesupport_identifier) {
    dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dh_gripper_msgs__msg__GripperRotCtrl__rosidl_typesupport_introspection_c__GripperRotCtrl_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
