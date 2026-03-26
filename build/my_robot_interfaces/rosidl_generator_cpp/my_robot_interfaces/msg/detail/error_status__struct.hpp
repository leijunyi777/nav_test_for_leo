// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_robot_interfaces:msg/ErrorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/error_status.hpp"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__STRUCT_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_robot_interfaces__msg__ErrorStatus __attribute__((deprecated))
#else
# define DEPRECATED__my_robot_interfaces__msg__ErrorStatus __declspec(deprecated)
#endif

namespace my_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ErrorStatus_
{
  using Type = ErrorStatus_<ContainerAllocator>;

  explicit ErrorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_state = false;
      this->error_number = 0l;
    }
  }

  explicit ErrorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error_state = false;
      this->error_number = 0l;
    }
  }

  // field types and members
  using _error_state_type =
    bool;
  _error_state_type error_state;
  using _error_number_type =
    int32_t;
  _error_number_type error_number;

  // setters for named parameter idiom
  Type & set__error_state(
    const bool & _arg)
  {
    this->error_state = _arg;
    return *this;
  }
  Type & set__error_number(
    const int32_t & _arg)
  {
    this->error_number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_robot_interfaces__msg__ErrorStatus
    std::shared_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_robot_interfaces__msg__ErrorStatus
    std::shared_ptr<my_robot_interfaces::msg::ErrorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ErrorStatus_ & other) const
  {
    if (this->error_state != other.error_state) {
      return false;
    }
    if (this->error_number != other.error_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const ErrorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ErrorStatus_

// alias to use template instance with default allocator
using ErrorStatus =
  my_robot_interfaces::msg::ErrorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__STRUCT_HPP_
