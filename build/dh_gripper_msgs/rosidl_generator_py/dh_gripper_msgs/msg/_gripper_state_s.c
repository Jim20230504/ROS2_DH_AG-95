// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dh_gripper_msgs:msg/GripperState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "dh_gripper_msgs/msg/detail/gripper_state__struct.h"
#include "dh_gripper_msgs/msg/detail/gripper_state__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool dh_gripper_msgs__msg__gripper_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("dh_gripper_msgs.msg._gripper_state.GripperState", full_classname_dest, 47) == 0);
  }
  dh_gripper_msgs__msg__GripperState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // is_initialized
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_initialized");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_initialized = (Py_True == field);
    Py_DECREF(field);
  }
  {  // grip_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "grip_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->grip_state = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_position = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_force = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dh_gripper_msgs__msg__gripper_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GripperState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dh_gripper_msgs.msg._gripper_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GripperState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dh_gripper_msgs__msg__GripperState * ros_message = (dh_gripper_msgs__msg__GripperState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_initialized
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_initialized ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_initialized", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // grip_state
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->grip_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "grip_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
