// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dh_gripper_msgs:msg/GripperRotCtrl.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__BUILDER_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dh_gripper_msgs/msg/detail/gripper_rot_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dh_gripper_msgs
{

namespace msg
{

namespace builder
{

class Init_GripperRotCtrl_speed
{
public:
  explicit Init_GripperRotCtrl_speed(::dh_gripper_msgs::msg::GripperRotCtrl & msg)
  : msg_(msg)
  {}
  ::dh_gripper_msgs::msg::GripperRotCtrl speed(::dh_gripper_msgs::msg::GripperRotCtrl::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotCtrl msg_;
};

class Init_GripperRotCtrl_force
{
public:
  explicit Init_GripperRotCtrl_force(::dh_gripper_msgs::msg::GripperRotCtrl & msg)
  : msg_(msg)
  {}
  Init_GripperRotCtrl_speed force(::dh_gripper_msgs::msg::GripperRotCtrl::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_GripperRotCtrl_speed(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotCtrl msg_;
};

class Init_GripperRotCtrl_angle
{
public:
  Init_GripperRotCtrl_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperRotCtrl_force angle(::dh_gripper_msgs::msg::GripperRotCtrl::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_GripperRotCtrl_force(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotCtrl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dh_gripper_msgs::msg::GripperRotCtrl>()
{
  return dh_gripper_msgs::msg::builder::Init_GripperRotCtrl_angle();
}

}  // namespace dh_gripper_msgs

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__BUILDER_HPP_
