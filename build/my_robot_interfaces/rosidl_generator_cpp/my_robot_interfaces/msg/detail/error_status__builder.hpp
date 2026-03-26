// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:msg/ErrorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/error_status.hpp"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/msg/detail/error_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_ErrorStatus_error_number
{
public:
  explicit Init_ErrorStatus_error_number(::my_robot_interfaces::msg::ErrorStatus & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::msg::ErrorStatus error_number(::my_robot_interfaces::msg::ErrorStatus::_error_number_type arg)
  {
    msg_.error_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::msg::ErrorStatus msg_;
};

class Init_ErrorStatus_error_state
{
public:
  Init_ErrorStatus_error_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ErrorStatus_error_number error_state(::my_robot_interfaces::msg::ErrorStatus::_error_state_type arg)
  {
    msg_.error_state = std::move(arg);
    return Init_ErrorStatus_error_number(msg_);
  }

private:
  ::my_robot_interfaces::msg::ErrorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::msg::ErrorStatus>()
{
  return my_robot_interfaces::msg::builder::Init_ErrorStatus_error_state();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__BUILDER_HPP_
