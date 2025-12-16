// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dh_gripper_msgs:msg/GripperRotCtrl.idl
// generated code does not contain a copyright notice

#ifndef DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__STRUCT_HPP_
#define DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dh_gripper_msgs__msg__GripperRotCtrl __attribute__((deprecated))
#else
# define DEPRECATED__dh_gripper_msgs__msg__GripperRotCtrl __declspec(deprecated)
#endif

namespace dh_gripper_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GripperRotCtrl_
{
  using Type = GripperRotCtrl_<ContainerAllocator>;

  explicit GripperRotCtrl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->force = 0.0f;
      this->speed = 0.0f;
    }
  }

  explicit GripperRotCtrl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->force = 0.0f;
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _angle_type =
    float;
  _angle_type angle;
  using _force_type =
    float;
  _force_type force;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__force(
    const float & _arg)
  {
    this->force = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator> *;
  using ConstRawPtr =
    const dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dh_gripper_msgs__msg__GripperRotCtrl
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dh_gripper_msgs__msg__GripperRotCtrl
    std::shared_ptr<dh_gripper_msgs::msg::GripperRotCtrl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperRotCtrl_ & other) const
  {
    if (this->angle != other.angle) {
      return false;
    }
    if (this->force != other.force) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperRotCtrl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperRotCtrl_

// alias to use template instance with default allocator
using GripperRotCtrl =
  dh_gripper_msgs::msg::GripperRotCtrl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dh_gripper_msgs

#endif  // DH_GRIPPER_MSGS__MSG__DETAIL__GRIPPER_ROT_CTRL__STRUCT_HPP_
