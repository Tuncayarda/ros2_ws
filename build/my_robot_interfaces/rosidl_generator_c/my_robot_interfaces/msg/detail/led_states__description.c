// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_robot_interfaces:msg/LedStates.idl
// generated code does not contain a copyright notice

#include "my_robot_interfaces/msg/detail/led_states__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces
const rosidl_type_hash_t *
my_robot_interfaces__msg__LedStates__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x97, 0xfe, 0x92, 0x43, 0x0d, 0x83, 0x55, 0x8d,
      0xd7, 0x07, 0x98, 0x6f, 0x84, 0xb2, 0xf5, 0xf2,
      0x8b, 0xcb, 0x80, 0x41, 0x88, 0x46, 0x93, 0xc3,
      0x3f, 0xf6, 0xc7, 0xbf, 0x27, 0xf3, 0x8e, 0xf1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char my_robot_interfaces__msg__LedStates__TYPE_NAME[] = "my_robot_interfaces/msg/LedStates";

// Define type names, field names, and default values
static char my_robot_interfaces__msg__LedStates__FIELD_NAME__led_states[] = "led_states";

static rosidl_runtime_c__type_description__Field my_robot_interfaces__msg__LedStates__FIELDS[] = {
  {
    {my_robot_interfaces__msg__LedStates__FIELD_NAME__led_states, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_interfaces__msg__LedStates__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_interfaces__msg__LedStates__TYPE_NAME, 33, 33},
      {my_robot_interfaces__msg__LedStates__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int64[] led_states";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_robot_interfaces__msg__LedStates__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_interfaces__msg__LedStates__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 18, 18},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_interfaces__msg__LedStates__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_interfaces__msg__LedStates__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
