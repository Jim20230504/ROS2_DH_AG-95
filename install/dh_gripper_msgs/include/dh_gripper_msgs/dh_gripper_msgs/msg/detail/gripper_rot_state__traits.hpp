// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dh_gripper_msgs:msg/GripperRotState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__TRAITS_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dh_gripper_msgs/msg/detail/gripper_rot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace dh_gripper_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GripperRotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: rot_state
  {
    out << "rot_state: ";
    rosidl_generator_traits::value_to_yaml(msg.rot_state, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: target_angle
  {
    out << "target_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.target_angle, out);
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
  const GripperRotState & msg,
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

  // member: rot_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rot_state: ";
    rosidl_generator_traits::value_to_yaml(msg.rot_state, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: target_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.target_angle, out);
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

inline std::string to_yaml(const GripperRotState & msg, bool use_flow_style = false)
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
  const dh_gripper_msgs::msg::GripperRotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  dh_gripper_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dh_gripper_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dh_gripper_msgs::msg::GripperRotState & msg)
{
  return dh_gripper_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dh_gripper_msgs::msg::GripperRotState>()
{
  return "dh_gripper_msgs::msg::GripperRotState";
}

template<>
inline const char * name<dh_gripper_msgs::msg::GripperRotState>()
{
  return "dh_gripper_msgs/msg/GripperRotState";
}

template<>
struct has_fixed_size<dh_gripper_msgs::msg::GripperRotState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<dh_gripper_msgs::msg::GripperRotState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<dh_gripper_msgs::msg::GripperRotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__TRAITS_HPP_
