// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from smartcar_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SMARTCAR_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define SMARTCAR_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "smartcar_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace smartcar_msgs
{

namespace msg
{

namespace builder
{

class Init_Status_engine_speed_rpm
{
public:
  explicit Init_Status_engine_speed_rpm(::smartcar_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  ::smartcar_msgs::msg::Status engine_speed_rpm(::smartcar_msgs::msg::Status::_engine_speed_rpm_type arg)
  {
    msg_.engine_speed_rpm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::smartcar_msgs::msg::Status msg_;
};

class Init_Status_steering_angle_rad
{
public:
  explicit Init_Status_steering_angle_rad(::smartcar_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_engine_speed_rpm steering_angle_rad(::smartcar_msgs::msg::Status::_steering_angle_rad_type arg)
  {
    msg_.steering_angle_rad = std::move(arg);
    return Init_Status_engine_speed_rpm(msg_);
  }

private:
  ::smartcar_msgs::msg::Status msg_;
};

class Init_Status_battery_percentage
{
public:
  explicit Init_Status_battery_percentage(::smartcar_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_steering_angle_rad battery_percentage(::smartcar_msgs::msg::Status::_battery_percentage_type arg)
  {
    msg_.battery_percentage = std::move(arg);
    return Init_Status_steering_angle_rad(msg_);
  }

private:
  ::smartcar_msgs::msg::Status msg_;
};

class Init_Status_battery_current_ma
{
public:
  explicit Init_Status_battery_current_ma(::smartcar_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_battery_percentage battery_current_ma(::smartcar_msgs::msg::Status::_battery_current_ma_type arg)
  {
    msg_.battery_current_ma = std::move(arg);
    return Init_Status_battery_percentage(msg_);
  }

private:
  ::smartcar_msgs::msg::Status msg_;
};

class Init_Status_battery_voltage_mv
{
public:
  Init_Status_battery_voltage_mv()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_battery_current_ma battery_voltage_mv(::smartcar_msgs::msg::Status::_battery_voltage_mv_type arg)
  {
    msg_.battery_voltage_mv = std::move(arg);
    return Init_Status_battery_current_ma(msg_);
  }

private:
  ::smartcar_msgs::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::smartcar_msgs::msg::Status>()
{
  return smartcar_msgs::msg::builder::Init_Status_battery_voltage_mv();
}

}  // namespace smartcar_msgs

#endif  // SMARTCAR_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
