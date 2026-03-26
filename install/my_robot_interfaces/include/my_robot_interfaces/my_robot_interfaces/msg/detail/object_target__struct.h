// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_interfaces:msg/ObjectTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/object_target.h"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__STRUCT_H_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'object_name'
// Member 'color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ObjectTarget in the package my_robot_interfaces.
typedef struct my_robot_interfaces__msg__ObjectTarget
{
  bool observe_state;
  rosidl_runtime_c__String object_name;
  rosidl_runtime_c__String color;
} my_robot_interfaces__msg__ObjectTarget;

// Struct for a sequence of my_robot_interfaces__msg__ObjectTarget.
typedef struct my_robot_interfaces__msg__ObjectTarget__Sequence
{
  my_robot_interfaces__msg__ObjectTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__msg__ObjectTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__OBJECT_TARGET__STRUCT_H_
