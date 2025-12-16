// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dh_gripper_msgs:msg/GripperState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dh_gripper_msgs__msg__GripperState __attribute__((deprecated))
#else
# define DEPRECATED__dh_gripper_msgs__msg__GripperState __declspec(deprecated)
#endif

namespace dh_gripper_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GripperState_
{
  using Type = GripperState_<ContainerAllocator>;

  explicit GripperState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_initialized = false;
      this->grip_state = 0l;
      this->position = 0.0f;
      this->target_position = 0.0f;
      this->target_force = 0.0f;
    }
  }

  explicit GripperState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_initialized = false;
      this->grip_state = 0l;
      this->position = 0.0f;
      this->target_position = 0.0f;
      this->target_force = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_initialized_type =
    bool;
  _is_initialized_type is_initialized;
  using _grip_state_type =
    int32_t;
  _grip_state_type grip_state;
  using _position_type =
    float;
  _position_type position;
  using _target_position_type =
    float;
  _target_position_type target_position;
  using _target_force_type =
    float;
  _target_force_type target_force;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_initialized(
    const bool & _arg)
  {
    this->is_initialized = _arg;
    return *this;
  }
  Type & set__grip_state(
    const int32_t & _arg)
  {
    this->grip_state = _arg;
    return *this;
  }
  Type & set__position(
    const float & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__target_position(
    const float & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__target_force(
    const float & _arg)
  {
    this->target_force = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dh_gripper_msgs::msg::GripperState_<ContainerAllocator> *;
  using ConstRawPtr =
    const dh_gripper_msgs::msg::GripperState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dh_gripper_msgs::msg::GripperState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dh_gripper_msgs::msg::GripperState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dh_gripper_msgs__msg__GripperState
    std::shared_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dh_gripper_msgs__msg__GripperState
    std::shared_ptr<dh_gripper_msgs::msg::GripperState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_initialized != other.is_initialized) {
      return false;
    }
    if (this->grip_state != other.grip_state) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->target_force != other.target_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperState_

// alias to use template instance with default allocator
using GripperState =
  dh_gripper_msgs::msg::GripperState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dh_gripper_msgs

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_
