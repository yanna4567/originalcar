// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from origincar_base:msg/Position.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "origincar_base/msg/detail/position__rosidl_typesupport_introspection_c.h"
#include "origincar_base/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "origincar_base/msg/detail/position__functions.h"
#include "origincar_base/msg/detail/position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  origincar_base__msg__Position__init(message_memory);
}

void origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_fini_function(void * message_memory)
{
  origincar_base__msg__Position__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_member_array[3] = {
  {
    "angle_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(origincar_base__msg__Position, angle_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(origincar_base__msg__Position, angle_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(origincar_base__msg__Position, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_members = {
  "origincar_base__msg",  // message namespace
  "Position",  // message name
  3,  // number of fields
  sizeof(origincar_base__msg__Position),
  origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_member_array,  // message members
  origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_init_function,  // function to initialize message memory (memory has to be allocated)
  origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_type_support_handle = {
  0,
  &origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_origincar_base
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, origincar_base, msg, Position)() {
  if (!origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_type_support_handle.typesupport_identifier) {
    origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &origincar_base__msg__Position__rosidl_typesupport_introspection_c__Position_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
