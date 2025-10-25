// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from smartcar_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "smartcar_msgs/msg/detail/status__rosidl_typesupport_introspection_c.h"
#include "smartcar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "smartcar_msgs/msg/detail/status__functions.h"
#include "smartcar_msgs/msg/detail/status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  smartcar_msgs__msg__Status__init(message_memory);
}

void smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_fini_function(void * message_memory)
{
  smartcar_msgs__msg__Status__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_member_array[5] = {
  {
    "battery_voltage_mv",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(smartcar_msgs__msg__Status, battery_voltage_mv),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_current_ma",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(smartcar_msgs__msg__Status, battery_current_ma),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_percentage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(smartcar_msgs__msg__Status, battery_percentage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "steering_angle_rad",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(smartcar_msgs__msg__Status, steering_angle_rad),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "engine_speed_rpm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(smartcar_msgs__msg__Status, engine_speed_rpm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_members = {
  "smartcar_msgs__msg",  // message namespace
  "Status",  // message name
  5,  // number of fields
  sizeof(smartcar_msgs__msg__Status),
  smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_member_array,  // message members
  smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_init_function,  // function to initialize message memory (memory has to be allocated)
  smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle = {
  0,
  &smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_smartcar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, smartcar_msgs, msg, Status)() {
  if (!smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle.typesupport_identifier) {
    smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &smartcar_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
