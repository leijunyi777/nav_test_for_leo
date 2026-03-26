// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_interfaces:msg/ObjectTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/object_target.hpp"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__TRAITS_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_interfaces/msg/detail/object_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObjectTarget & msg,
  std::ostream & out)
{
  out << "{";
  // member: observe_state
  {
    out << "observe_state: ";
    rosidl_generator_traits::value_to_yaml(msg.observe_state, out);
    out << ", ";
  }

  // member: object_name
  {
    out << "object_name: ";
    rosidl_generator_traits::value_to_yaml(msg.object_name, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObjectTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: observe_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "observe_state: ";
    rosidl_generator_traits::value_to_yaml(msg.observe_state, out);
    out << "\n";
  }

  // member: object_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_name: ";
    rosidl_generator_traits::value_to_yaml(msg.object_name, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color: ";
    rosidl_generator_traits::value_to_yaml(msg.color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObjectTarget & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace my_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_interfaces::msg::ObjectTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_interfaces::msg::ObjectTarget & msg)
{
  return my_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_interfaces::msg::ObjectTarget>()
{
  return "my_robot_interfaces::msg::ObjectTarget";
}

template<>
inline const char * name<my_robot_interfaces::msg::ObjectTarget>()
{
  return "my_robot_interfaces/msg/ObjectTarget";
}

template<>
struct has_fixed_size<my_robot_interfaces::msg::ObjectTarget>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_robot_interfaces::msg::ObjectTarget>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_robot_interfaces::msg::ObjectTarget>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__TRAITS_HPP_
