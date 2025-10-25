// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from smartcar_msgs:msg/Status.idl
// generated code does not contain a copyright notice
#include "smartcar_msgs/msg/detail/status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
smartcar_msgs__msg__Status__init(smartcar_msgs__msg__Status * msg)
{
  if (!msg) {
    return false;
  }
  // battery_voltage_mv
  // battery_current_ma
  // battery_percentage
  // steering_angle_rad
  // engine_speed_rpm
  return true;
}

void
smartcar_msgs__msg__Status__fini(smartcar_msgs__msg__Status * msg)
{
  if (!msg) {
    return;
  }
  // battery_voltage_mv
  // battery_current_ma
  // battery_percentage
  // steering_angle_rad
  // engine_speed_rpm
}

bool
smartcar_msgs__msg__Status__are_equal(const smartcar_msgs__msg__Status * lhs, const smartcar_msgs__msg__Status * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // battery_voltage_mv
  if (lhs->battery_voltage_mv != rhs->battery_voltage_mv) {
    return false;
  }
  // battery_current_ma
  if (lhs->battery_current_ma != rhs->battery_current_ma) {
    return false;
  }
  // battery_percentage
  if (lhs->battery_percentage != rhs->battery_percentage) {
    return false;
  }
  // steering_angle_rad
  if (lhs->steering_angle_rad != rhs->steering_angle_rad) {
    return false;
  }
  // engine_speed_rpm
  if (lhs->engine_speed_rpm != rhs->engine_speed_rpm) {
    return false;
  }
  return true;
}

bool
smartcar_msgs__msg__Status__copy(
  const smartcar_msgs__msg__Status * input,
  smartcar_msgs__msg__Status * output)
{
  if (!input || !output) {
    return false;
  }
  // battery_voltage_mv
  output->battery_voltage_mv = input->battery_voltage_mv;
  // battery_current_ma
  output->battery_current_ma = input->battery_current_ma;
  // battery_percentage
  output->battery_percentage = input->battery_percentage;
  // steering_angle_rad
  output->steering_angle_rad = input->steering_angle_rad;
  // engine_speed_rpm
  output->engine_speed_rpm = input->engine_speed_rpm;
  return true;
}

smartcar_msgs__msg__Status *
smartcar_msgs__msg__Status__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  smartcar_msgs__msg__Status * msg = (smartcar_msgs__msg__Status *)allocator.allocate(sizeof(smartcar_msgs__msg__Status), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(smartcar_msgs__msg__Status));
  bool success = smartcar_msgs__msg__Status__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
smartcar_msgs__msg__Status__destroy(smartcar_msgs__msg__Status * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    smartcar_msgs__msg__Status__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
smartcar_msgs__msg__Status__Sequence__init(smartcar_msgs__msg__Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  smartcar_msgs__msg__Status * data = NULL;

  if (size) {
    data = (smartcar_msgs__msg__Status *)allocator.zero_allocate(size, sizeof(smartcar_msgs__msg__Status), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = smartcar_msgs__msg__Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        smartcar_msgs__msg__Status__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
smartcar_msgs__msg__Status__Sequence__fini(smartcar_msgs__msg__Status__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      smartcar_msgs__msg__Status__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

smartcar_msgs__msg__Status__Sequence *
smartcar_msgs__msg__Status__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  smartcar_msgs__msg__Status__Sequence * array = (smartcar_msgs__msg__Status__Sequence *)allocator.allocate(sizeof(smartcar_msgs__msg__Status__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = smartcar_msgs__msg__Status__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
smartcar_msgs__msg__Status__Sequence__destroy(smartcar_msgs__msg__Status__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    smartcar_msgs__msg__Status__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
smartcar_msgs__msg__Status__Sequence__are_equal(const smartcar_msgs__msg__Status__Sequence * lhs, const smartcar_msgs__msg__Status__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!smartcar_msgs__msg__Status__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
smartcar_msgs__msg__Status__Sequence__copy(
  const smartcar_msgs__msg__Status__Sequence * input,
  smartcar_msgs__msg__Status__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(smartcar_msgs__msg__Status);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    smartcar_msgs__msg__Status * data =
      (smartcar_msgs__msg__Status *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!smartcar_msgs__msg__Status__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          smartcar_msgs__msg__Status__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!smartcar_msgs__msg__Status__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
