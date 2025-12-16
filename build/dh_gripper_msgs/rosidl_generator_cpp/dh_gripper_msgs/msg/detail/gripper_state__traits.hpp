// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dh_gripper_msgs:msg/GripperState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__TRAITS_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dh_gripper_msgs/msg/detail/gripper_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace dh_gripper_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GripperState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_initialized
  {
    out << "is_initialized: ";
    rosidl_generator_traits::value_to_yaml(msg.is_initialized, out);
    out << ", ";
  }

  // member: grip_state
  {
    out << "grip_state: ";
    rosidl_generator_traits::value_to_yaml(msg.grip_state, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: target_force
  {
    out << "target_force: ";
    rosidl_generator_traits::value_to_yaml(msg.target_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: is_initialized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_initialized: ";
    rosidl_generator_traits::value_to_yaml(msg.is_initialized, out);
    out << "\n";
  }

  // member: grip_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grip_state: ";
    rosidl_generator_traits::value_to_yaml(msg.grip_state, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << "\n";
  }

  // member: target_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_force: ";
    rosidl_generator_traits::value_to_yaml(msg.target_force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dh_gripper_msgs

namespace rosidl_generator_traits
{

[[deprecated("use dh_gripper_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dh_gripper_msgs::msg::GripperState & msg,
  std::ostream & out, size_t indentation = 0)
{
  dh_gripper_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dh_gripper_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dh_gripper_msgs::msg::GripperState & msg)
{
  return dh_gripper_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dh_gripper_msgs::msg::GripperState>()
{
  return "dh_gripper_msgs::msg::GripperState";
}

template<>
inline const char * name<dh_gripper_msgs::msg::GripperState>()
{
  return "dh_gripper_msgs/msg/GripperState";
}

template<>
struct has_fixed_size<dh_gripper_msgs::msg::GripperState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<dh_gripper_msgs::msg::GripperState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<dh_gripper_msgs::msg::GripperState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__TRAITS_HPP_
