// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from smartcar_msgs:msg/Status.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "smartcar_msgs/msg/detail/status__struct.h"
#include "smartcar_msgs/msg/detail/status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool smartcar_msgs__msg__status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("smartcar_msgs.msg._status.Status", full_classname_dest, 32) == 0);
  }
  smartcar_msgs__msg__Status * ros_message = _ros_message;
  {  // battery_voltage_mv
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage_mv");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->battery_voltage_mv = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // battery_current_ma
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_current_ma");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->battery_current_ma = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // battery_percentage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_percentage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_percentage = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // steering_angle_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "steering_angle_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->steering_angle_rad = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // engine_speed_rpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "engine_speed_rpm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->engine_speed_rpm = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * smartcar_msgs__msg__status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Status */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("smartcar_msgs.msg._status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Status");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  smartcar_msgs__msg__Status * ros_message = (smartcar_msgs__msg__Status *)raw_ros_message;
  {  // battery_voltage_mv
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->battery_voltage_mv);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage_mv", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_current_ma
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->battery_current_ma);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_current_ma", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_percentage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_percentage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_percentage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // steering_angle_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->steering_angle_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "steering_angle_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // engine_speed_rpm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->engine_speed_rpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "engine_speed_rpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
