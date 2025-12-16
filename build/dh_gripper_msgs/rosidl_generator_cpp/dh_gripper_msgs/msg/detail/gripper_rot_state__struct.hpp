// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dh_gripper_msgs:msg/GripperRotState.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__STRUCT_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__STRUCT_HPP_

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
# define DEPRECATED__dh_gripper_msgs__msg__GripperRotState __attribute__((deprecated))
#else
# define DEPRECATED__dh_gripper_msgs__msg__GripperRotState __declspec(deprecated)
#endif

namespace dh_gripper_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GripperRotState_
{
  using Type = GripperRotState_<ContainerAllocator>;

  explicit GripperRotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rot_state = 0l;
      this->angle = 0.0f;
      this->target_angle = 0.0f;
      this->target_force = 0.0f;
    }
  }

  explicit GripperRotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rot_state = 0l;
      this->angle = 0.0f;
      this->target_angle = 0.0f;
      this->target_force = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rot_state_type =
    int32_t;
  _rot_state_type rot_state;
  using _angle_type =
    float;
  _angle_type angle;
  using _target_angle_type =
    float;
  _target_angle_type target_angle;
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
  Type & set__rot_state(
    const int32_t & _arg)
  {
    this->rot_state = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__target_angle(
    const float & _arg)
  {
    this->target_angle = _arg;
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
    dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dh_gripper_msgs__msg__GripperRotState
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dh_gripper_msgs__msg__GripperRotState
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperRotState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rot_state != other.rot_state) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->target_angle != other.target_angle) {
      return false;
    }
    if (this->target_force != other.target_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperRotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperRotState_

// alias to use template instance with default allocator
using GripperRotState =
  dh_gripper_msgs::msg::GripperRotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dh_gripper_msgs

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_STATE__STRUCT_HPP_
