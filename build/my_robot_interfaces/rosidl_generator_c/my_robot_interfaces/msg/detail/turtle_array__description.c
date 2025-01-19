// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from my_robot_interfaces:msg/TurtleArray.idl
// generated code does not contain a copyright notice

#include "my_robot_interfaces/msg/detail/turtle_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_my_robot_interfaces
const rosidl_type_hash_t *
my_robot_interfaces__msg__TurtleArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xef, 0x3a, 0x06, 0xa5, 0x35, 0xb6, 0x17, 0x56,
      0x92, 0xa6, 0x8c, 0xba, 0x13, 0xee, 0xac, 0xea,
      0x06, 0x38, 0x2c, 0x75, 0xa9, 0xad, 0xad, 0x78,
      0x59, 0x59, 0xf3, 0x53, 0xac, 0x03, 0x92, 0x67,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "my_robot_interfaces/msg/detail/turtle__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t my_robot_interfaces__msg__Turtle__EXPECTED_HASH = {1, {
    0xf1, 0x16, 0x09, 0xf9, 0xb9, 0x08, 0xb0, 0xa0,
    0x75, 0xb1, 0x0f, 0x9c, 0x05, 0xc5, 0x21, 0xd7,
    0x56, 0xc1, 0x92, 0x63, 0x1c, 0xbd, 0x42, 0x13,
    0xd7, 0xd8, 0x93, 0x25, 0x41, 0x19, 0x4b, 0x18,
  }};
#endif

static char my_robot_interfaces__msg__TurtleArray__TYPE_NAME[] = "my_robot_interfaces/msg/TurtleArray";
static char my_robot_interfaces__msg__Turtle__TYPE_NAME[] = "my_robot_interfaces/msg/Turtle";

// Define type names, field names, and default values
static char my_robot_interfaces__msg__TurtleArray__FIELD_NAME__turtles[] = "turtles";

static rosidl_runtime_c__type_description__Field my_robot_interfaces__msg__TurtleArray__FIELDS[] = {
  {
    {my_robot_interfaces__msg__TurtleArray__FIELD_NAME__turtles, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {my_robot_interfaces__msg__Turtle__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription my_robot_interfaces__msg__TurtleArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {my_robot_interfaces__msg__Turtle__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
my_robot_interfaces__msg__TurtleArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {my_robot_interfaces__msg__TurtleArray__TYPE_NAME, 35, 35},
      {my_robot_interfaces__msg__TurtleArray__FIELDS, 1, 1},
    },
    {my_robot_interfaces__msg__TurtleArray__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&my_robot_interfaces__msg__Turtle__EXPECTED_HASH, my_robot_interfaces__msg__Turtle__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = my_robot_interfaces__msg__Turtle__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "Turtle[] turtles";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
my_robot_interfaces__msg__TurtleArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {my_robot_interfaces__msg__TurtleArray__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 16, 16},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
my_robot_interfaces__msg__TurtleArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *my_robot_interfaces__msg__TurtleArray__get_individual_type_description_source(NULL),
    sources[1] = *my_robot_interfaces__msg__Turtle__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
