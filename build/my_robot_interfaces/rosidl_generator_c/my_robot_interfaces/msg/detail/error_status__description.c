// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_robot_interfaces:msg/ErrorStatus.idl
// generated code does not contain a copyright notice

#include "my_robot_interfaces/msg/detail/error_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces
const rosidl_type_hash_t *
my_robot_interfaces__msg__ErrorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x62, 0xd1, 0xc6, 0x46, 0x10, 0x3a, 0x01, 0x82,
      0x80, 0xab, 0x44, 0xba, 0x82, 0xe0, 0x52, 0x54,
      0x4c, 0x46, 0x9f, 0x87, 0x5c, 0x5f, 0xf2, 0x7b,
      0x30, 0x1d, 0xc6, 0x4c, 0x53, 0x34, 0x42, 0xdb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_robot_interfaces__msg__ErrorStatus__TYPE_NAME[] = "my_robot_interfaces/msg/ErrorStatus";

// Define type names, field names, and default values
static char my_robot_interfaces__msg__ErrorStatus__FIELD_NAME__error_state[] = "error_state";
static char my_robot_interfaces__msg__ErrorStatus__FIELD_NAME__error_number[] = "error_number";

static rosidl_runtime_c__type_description__Field my_robot_interfaces__msg__ErrorStatus__FIELDS[] = {
  {
    {my_robot_interfaces__msg__ErrorStatus__FIELD_NAME__error_state, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_interfaces__msg__ErrorStatus__FIELD_NAME__error_number, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_interfaces__msg__ErrorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_interfaces__msg__ErrorStatus__TYPE_NAME, 35, 35},
      {my_robot_interfaces__msg__ErrorStatus__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool error_state\n"
  "int32 error_number";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_robot_interfaces__msg__ErrorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_interfaces__msg__ErrorStatus__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 35, 35},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_interfaces__msg__ErrorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_interfaces__msg__ErrorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
