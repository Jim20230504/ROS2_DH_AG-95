// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dh_gripper_msgs:msg/GripperRotCtrl.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__TRAITS_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dh_gripper_msgs/msg/detail/gripper_rot_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dh_gripper_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GripperRotCtrl & msg,
  std::ostream & out)
{
  out << "{";
  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
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
  const GripperRotCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
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

inline std::string to_yaml(const GripperRotCtrl & msg, bool use_flow_style = false)
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
  const dh_gripper_msgs::msg::GripperRotCtrl & msg,
  std::ostream & out, size_t indentation = 0)
{
  dh_gripper_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dh_gripper_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dh_gripper_msgs::msg::GripperRotCtrl & msg)
{
  return dh_gripper_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dh_gripper_msgs::msg::GripperRotCtrl>()
{
  return "dh_gripper_msgs::msg::GripperRotCtrl";
}

template<>
inline const char * name<dh_gripper_msgs::msg::GripperRotCtrl>()
{
  return "dh_gripper_msgs/msg/GripperRotCtrl";
}

template<>
struct has_fixed_size<dh_gripper_msgs::msg::GripperRotCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dh_gripper_msgs::msg::GripperRotCtrl>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dh_gripper_msgs::msg::GripperRotCtrl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__TRAITS_HPP_
