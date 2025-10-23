// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from smartcar_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SMARTCAR_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define SMARTCAR_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "smartcar_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace smartcar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: battery_voltage_mv
  {
    out << "battery_voltage_mv: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage_mv, out);
    out << ", ";
  }

  // member: battery_current_ma
  {
    out << "battery_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current_ma, out);
    out << ", ";
  }

  // member: battery_percentage
  {
    out << "battery_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percentage, out);
    out << ", ";
  }

  // member: steering_angle_rad
  {
    out << "steering_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle_rad, out);
    out << ", ";
  }

  // member: engine_speed_rpm
  {
    out << "engine_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.engine_speed_rpm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: battery_voltage_mv
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage_mv: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage_mv, out);
    out << "\n";
  }

  // member: battery_current_ma
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current_ma, out);
    out << "\n";
  }

  // member: battery_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percentage, out);
    out << "\n";
  }

  // member: steering_angle_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle_rad, out);
    out << "\n";
  }

  // member: engine_speed_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "engine_speed_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.engine_speed_rpm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace smartcar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use smartcar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const smartcar_msgs::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  smartcar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use smartcar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const smartcar_msgs::msg::Status & msg)
{
  return smartcar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<smartcar_msgs::msg::Status>()
{
  return "smartcar_msgs::msg::Status";
}

template<>
inline const char * name<smartcar_msgs::msg::Status>()
{
  return "smartcar_msgs/msg/Status";
}

template<>
struct has_fixed_size<smartcar_msgs::msg::Status>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<smartcar_msgs::msg::Status>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<smartcar_msgs::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SMARTCAR_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
