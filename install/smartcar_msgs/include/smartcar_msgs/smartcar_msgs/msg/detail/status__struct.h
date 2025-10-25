// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from smartcar_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SMARTCAR_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
#define SMARTCAR_MSGS__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Status in the package smartcar_msgs.
typedef struct smartcar_msgs__msg__Status
{
  int32_t battery_voltage_mv;
  int32_t battery_current_ma;
  double battery_percentage;
  double steering_angle_rad;
  int32_t engine_speed_rpm;
} smartcar_msgs__msg__Status;

// Struct for a sequence of smartcar_msgs__msg__Status.
typedef struct smartcar_msgs__msg__Status__Sequence
{
  smartcar_msgs__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} smartcar_msgs__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SMARTCAR_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
