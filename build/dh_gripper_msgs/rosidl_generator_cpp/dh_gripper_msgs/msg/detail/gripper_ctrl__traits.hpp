// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dh_gripper_msgs:msg/GripperCtrl.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_CTRL__TRAITS_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_CTRL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dh_gripper_msgs/msg/detail/gripper_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dh_gripper_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GripperCtrl & msg,
  std::ostream & out)
{
  out << "{";
  // member: initialize
  {
    out << "initialize: ";
    rosidl_generator_traits::value_to_yaml(msg.initialize, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: force
  {
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: initialize
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initialize: ";
    rosidl_generator_traits::value_to_yaml(msg.initialize, out);
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

  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperCtrl & msg, bool use_flow_style = false)
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
  const dh_gripper_msgs::msg::GripperCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  dh_gripper_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dh_gripper_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dh_gripper_msgs::msg::GripperCtrl & msg)
{
  return dh_gripper_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dh_gripper_msgs::msg::GripperCtrl>()
{
  return "dh_gripper_msgs::msg::GripperCtrl";
}

template<>
inline const char * name<dh_gripper_msgs::msg::GripperCtrl>()
{
  return "dh_gripper_msgs/msg/GripperCtrl";
}

template<>
struct has_fixed_size<dh_gripper_msgs::msg::GripperCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dh_gripper_msgs::msg::GripperCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dh_gripper_msgs::msg::GripperCtrl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_CTRL__TRAITS_HPP_
