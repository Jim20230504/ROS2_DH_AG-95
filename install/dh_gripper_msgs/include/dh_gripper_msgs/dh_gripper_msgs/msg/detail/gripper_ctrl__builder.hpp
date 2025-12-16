// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dh_gripper_msgs:msg/GripperCtrl.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_CTRL__BUILDER_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_CTRL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dh_gripper_msgs/msg/detail/gripper_ctrl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dh_gripper_msgs
{

namespace msg
{

namespace builder
{

class Init_GripperCtrl_speed
{
public:
  explicit Init_GripperCtrl_speed(::dh_gripper_msgs::msg::GripperCtrl & msg)
  : msg_(msg)
  {}
  ::dh_gripper_msgs::msg::GripperCtrl speed(::dh_gripper_msgs::msg::GripperCtrl::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperCtrl msg_;
};

class Init_GripperCtrl_force
{
public:
  explicit Init_GripperCtrl_force(::dh_gripper_msgs::msg::GripperCtrl & msg)
  : msg_(msg)
  {}
  Init_GripperCtrl_speed force(::dh_gripper_msgs::msg::GripperCtrl::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_GripperCtrl_speed(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperCtrl msg_;
};

class Init_GripperCtrl_position
{
public:
  explicit Init_GripperCtrl_position(::dh_gripper_msgs::msg::GripperCtrl & msg)
  : msg_(msg)
  {}
  Init_GripperCtrl_force position(::dh_gripper_msgs::msg::GripperCtrl::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_GripperCtrl_force(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperCtrl msg_;
};

class Init_GripperCtrl_initialize
{
public:
  Init_GripperCtrl_initialize()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperCtrl_position initialize(::dh_gripper_msgs::msg::GripperCtrl::_initialize_type arg)
  {
    msg_.initialize = std::move(arg);
    return Init_GripperCtrl_position(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperCtrl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dh_gripper_msgs::msg::GripperCtrl>()
{
  return dh_gripper_msgs::msg::builder::Init_GripperCtrl_initialize();
}

}  // namespace dh_gripper_msgs

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_CTRL__BUILDER_HPP_
