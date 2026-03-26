// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_interfaces:msg/ErrorStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/error_status.h"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__STRUCT_H_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ErrorStatus in the package my_robot_interfaces.
typedef struct my_robot_interfaces__msg__ErrorStatus
{
  bool error_state;
  int32_t error_number;
} my_robot_interfaces__msg__ErrorStatus;

// Struct for a sequence of my_robot_interfaces__msg__ErrorStatus.
typedef struct my_robot_interfaces__msg__ErrorStatus__Sequence
{
  my_robot_interfaces__msg__ErrorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__msg__ErrorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__ERROR_STATUS__STRUCT_H_
