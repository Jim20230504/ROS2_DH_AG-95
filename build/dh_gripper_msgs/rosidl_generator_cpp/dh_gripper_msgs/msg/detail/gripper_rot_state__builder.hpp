// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dh_gripper_msgs:msg/GripperRotState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__BUILDER_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dh_gripper_msgs/msg/detail/gripper_rot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dh_gripper_msgs
{

namespace msg
{

namespace builder
{

class Init_GripperRotState_target_force
{
public:
  explicit Init_GripperRotState_target_force(::dh_gripper_msgs::msg::GripperRotState & msg)
  : msg_(msg)
  {}
  ::dh_gripper_msgs::msg::GripperRotState target_force(::dh_gripper_msgs::msg::GripperRotState::_target_force_type arg)
  {
    msg_.target_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotState msg_;
};

class Init_GripperRotState_target_angle
{
public:
  explicit Init_GripperRotState_target_angle(::dh_gripper_msgs::msg::GripperRotState & msg)
  : msg_(msg)
  {}
  Init_GripperRotState_target_force target_angle(::dh_gripper_msgs::msg::GripperRotState::_target_angle_type arg)
  {
    msg_.target_angle = std::move(arg);
    return Init_GripperRotState_target_force(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotState msg_;
};

class Init_GripperRotState_angle
{
public:
  explicit Init_GripperRotState_angle(::dh_gripper_msgs::msg::GripperRotState & msg)
  : msg_(msg)
  {}
  Init_GripperRotState_target_angle angle(::dh_gripper_msgs::msg::GripperRotState::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_GripperRotState_target_angle(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotState msg_;
};

class Init_GripperRotState_rot_state
{
public:
  explicit Init_GripperRotState_rot_state(::dh_gripper_msgs::msg::GripperRotState & msg)
  : msg_(msg)
  {}
  Init_GripperRotState_angle rot_state(::dh_gripper_msgs::msg::GripperRotState::_rot_state_type arg)
  {
    msg_.rot_state = std::move(arg);
    return Init_GripperRotState_angle(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotState msg_;
};

class Init_GripperRotState_header
{
public:
  Init_GripperRotState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperRotState_rot_state header(::dh_gripper_msgs::msg::GripperRotState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GripperRotState_rot_state(msg_);
  }

private:
  ::dh_gripper_msgs::msg::GripperRotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dh_gripper_msgs::msg::GripperRotState>()
{
  return dh_gripper_msgs::msg::builder::Init_GripperRotState_header();
}

}  // namespace dh_gripper_msgs

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__BUILDER_HPP_
