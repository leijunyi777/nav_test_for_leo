// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_robot_interfaces:msg/ObjectTarget.idl
// generated code does not contain a copyright notice

#include "my_robot_interfaces/msg/detail/object_target__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces
const rosidl_type_hash_t *
my_robot_interfaces__msg__ObjectTarget__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x96, 0x9f, 0xc4, 0xf5, 0x99, 0xdc, 0x2f, 0xfc,
      0x01, 0xf0, 0x2c, 0x52, 0x43, 0xdf, 0x1d, 0x8f,
      0xf5, 0x61, 0x94, 0xee, 0xba, 0x78, 0xf5, 0xea,
      0xd8, 0xce, 0xdb, 0xf9, 0x8d, 0x6c, 0xa3, 0x89,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_robot_interfaces__msg__ObjectTarget__TYPE_NAME[] = "my_robot_interfaces/msg/ObjectTarget";

// Define type names, field names, and default values
static char my_robot_interfaces__msg__ObjectTarget__FIELD_NAME__observe_state[] = "observe_state";
static char my_robot_interfaces__msg__ObjectTarget__FIELD_NAME__object_name[] = "object_name";
static char my_robot_interfaces__msg__ObjectTarget__FIELD_NAME__color[] = "color";

static rosidl_runtime_c__type_description__Field my_robot_interfaces__msg__ObjectTarget__FIELDS[] = {
  {
    {my_robot_interfaces__msg__ObjectTarget__FIELD_NAME__observe_state, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_interfaces__msg__ObjectTarget__FIELD_NAME__object_name, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {my_robot_interfaces__msg__ObjectTarget__FIELD_NAME__color, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_interfaces__msg__ObjectTarget__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_interfaces__msg__ObjectTarget__TYPE_NAME, 36, 36},
      {my_robot_interfaces__msg__ObjectTarget__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool observe_state\n"
  "string object_name\n"
  "string color";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_robot_interfaces__msg__ObjectTarget__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_interfaces__msg__ObjectTarget__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 51, 51},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_interfaces__msg__ObjectTarget__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_interfaces__msg__ObjectTarget__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
