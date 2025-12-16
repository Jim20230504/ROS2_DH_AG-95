// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dh_gripper_msgs:msg/GripperState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__BUILDER_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dh_gripper_msgs/msg/detail/gripper_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dh_gripper_msgs
{

namespace msg
{

namespace builder
{

class Init_GripperState_target_force
{
public:
  explicit Init_GripperState_target_force(::dh_gripper_msgs::msg::GripperState & msg)
  : msg_(msg)
  {}
  ::dh_gripper_msgs::msg::GripperState target_force(::dh_gripper_msgs::msg::GripperState::_target_force_type arg)
  {
    msg_.target_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperState msg_;
};

class Init_GripperState_target_position
{
public:
  explicit Init_GripperState_target_position(::dh_gripper_msgs::msg::GripperState & msg)
  : msg_(msg)
  {}
  Init_GripperState_target_force target_position(::dh_gripper_msgs::msg::GripperState::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_GripperState_target_force(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperState msg_;
};

class Init_GripperState_position
{
public:
  explicit Init_GripperState_position(::dh_gripper_msgs::msg::GripperState & msg)
  : msg_(msg)
  {}
  Init_GripperState_target_position position(::dh_gripper_msgs::msg::GripperState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_GripperState_target_position(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperState msg_;
};

class Init_GripperState_grip_state
{
public:
  explicit Init_GripperState_grip_state(::dh_gripper_msgs::msg::GripperState & msg)
  : msg_(msg)
  {}
  Init_GripperState_position grip_state(::dh_gripper_msgs::msg::GripperState::_grip_state_type arg)
  {
    msg_.grip_state = std::move(arg);
    return Init_GripperState_position(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperState msg_;
};

class Init_GripperState_is_initialized
{
public:
  explicit Init_GripperState_is_initialized(::dh_gripper_msgs::msg::GripperState & msg)
  : msg_(msg)
  {}
  Init_GripperState_grip_state is_initialized(::dh_gripper_msgs::msg::GripperState::_is_initialized_type arg)
  {
    msg_.is_initialized = std::move(arg);
    return Init_GripperState_grip_state(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperState msg_;
};

class Init_GripperState_header
{
public:
  Init_GripperState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperState_is_initialized header(::dh_gripper_msgs::msg::GripperState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GripperState_is_initialized(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dh_gripper_msgs::msg::GripperState>()
{
  return dh_gripper_msgs::msg::builder::Init_GripperState_header();
}

}  // namespace dh_gripper_msgs

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__BUILDER_HPP_
