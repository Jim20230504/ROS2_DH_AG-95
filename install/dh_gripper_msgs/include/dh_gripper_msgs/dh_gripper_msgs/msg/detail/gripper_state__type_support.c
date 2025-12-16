// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dh_gripper_msgs:msg/GripperState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dh_gripper_msgs/msg/detail/gripper_state__rosidl_typesupport_introspection_c.h"
#include "dh_gripper_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dh_gripper_msgs/msg/detail/gripper_state__functions.h"
#include "dh_gripper_msgs/msg/detail/gripper_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dh_gripper_msgs__msg__GripperState__init(message_memory);
}

void dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_fini_function(void * message_memory)
{
  dh_gripper_msgs__msg__GripperState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_initialized",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperState, is_initialized),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grip_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperState, grip_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperState, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperState, target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dh_gripper_msgs__msg__GripperState, target_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_members = {
  "dh_gripper_msgs__msg",  // message namespace
  "GripperState",  // message name
  6,  // number of fields
  sizeof(dh_gripper_msgs__msg__GripperState),
  dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_member_array,  // message members
  dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_init_function,  // function to initialize message memory (memory has to be allocated)
  dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle = {
  0,
  &dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dh_gripper_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dh_gripper_msgs, msg, GripperState)() {
  dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle.typesupport_identifier) {
    dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dh_gripper_msgs__msg__GripperState__rosidl_typesupport_introspection_c__GripperState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
