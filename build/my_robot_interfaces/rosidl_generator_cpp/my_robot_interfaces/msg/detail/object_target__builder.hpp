// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:msg/ObjectTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/object_target.hpp"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/msg/detail/object_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectTarget_color
{
public:
  explicit Init_ObjectTarget_color(::my_robot_interfaces::msg::ObjectTarget & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::msg::ObjectTarget color(::my_robot_interfaces::msg::ObjectTarget::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::msg::ObjectTarget msg_;
};

class Init_ObjectTarget_object_name
{
public:
  explicit Init_ObjectTarget_object_name(::my_robot_interfaces::msg::ObjectTarget & msg)
  : msg_(msg)
  {}
  Init_ObjectTarget_color object_name(::my_robot_interfaces::msg::ObjectTarget::_object_name_type arg)
  {
    msg_.object_name = std::move(arg);
    return Init_ObjectTarget_color(msg_);
  }

private:
  ::my_robot_interfaces::msg::ObjectTarget msg_;
};

class Init_ObjectTarget_observe_state
{
public:
  Init_ObjectTarget_observe_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectTarget_object_name observe_state(::my_robot_interfaces::msg::ObjectTarget::_observe_state_type arg)
  {
    msg_.observe_state = std::move(arg);
    return Init_ObjectTarget_object_name(msg_);
  }

private:
  ::my_robot_interfaces::msg::ObjectTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::msg::ObjectTarget>()
{
  return my_robot_interfaces::msg::builder::Init_ObjectTarget_observe_state();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__BUILDER_HPP_
