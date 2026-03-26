// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_interfaces:msg/ErrorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/error_status.hpp"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__TRAITS_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_interfaces/msg/detail/error_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ErrorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: error_state
  {
    out << "error_state: ";
    rosidl_generator_traits::value_to_yaml(msg.error_state, out);
    out << ", ";
  }

  // member: error_number
  {
    out << "error_number: ";
    rosidl_generator_traits::value_to_yaml(msg.error_number, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ErrorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: error_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_state: ";
    rosidl_generator_traits::value_to_yaml(msg.error_state, out);
    out << "\n";
  }

  // member: error_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_number: ";
    rosidl_generator_traits::value_to_yaml(msg.error_number, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ErrorStatus & msg, bool use_flow_style = false)
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
  const my_robot_interfaces::msg::ErrorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_interfaces::msg::ErrorStatus & msg)
{
  return my_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_interfaces::msg::ErrorStatus>()
{
  return "my_robot_interfaces::msg::ErrorStatus";
}

template<>
inline const char * name<my_robot_interfaces::msg::ErrorStatus>()
{
  return "my_robot_interfaces/msg/ErrorStatus";
}

template<>
struct has_fixed_size<my_robot_interfaces::msg::ErrorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_robot_interfaces::msg::ErrorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_robot_interfaces::msg::ErrorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__TRAITS_HPP_
