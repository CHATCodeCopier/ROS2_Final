// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from smartcar_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef SMARTCAR_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define SMARTCAR_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__smartcar_msgs__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__smartcar_msgs__msg__Status __declspec(deprecated)
#endif

namespace smartcar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage_mv = 0l;
      this->battery_current_ma = 0l;
      this->battery_percentage = 0.0;
      this->steering_angle_rad = 0.0;
      this->engine_speed_rpm = 0l;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage_mv = 0l;
      this->battery_current_ma = 0l;
      this->battery_percentage = 0.0;
      this->steering_angle_rad = 0.0;
      this->engine_speed_rpm = 0l;
    }
  }

  // field types and members
  using _battery_voltage_mv_type =
    int32_t;
  _battery_voltage_mv_type battery_voltage_mv;
  using _battery_current_ma_type =
    int32_t;
  _battery_current_ma_type battery_current_ma;
  using _battery_percentage_type =
    double;
  _battery_percentage_type battery_percentage;
  using _steering_angle_rad_type =
    double;
  _steering_angle_rad_type steering_angle_rad;
  using _engine_speed_rpm_type =
    int32_t;
  _engine_speed_rpm_type engine_speed_rpm;

  // setters for named parameter idiom
  Type & set__battery_voltage_mv(
    const int32_t & _arg)
  {
    this->battery_voltage_mv = _arg;
    return *this;
  }
  Type & set__battery_current_ma(
    const int32_t & _arg)
  {
    this->battery_current_ma = _arg;
    return *this;
  }
  Type & set__battery_percentage(
    const double & _arg)
  {
    this->battery_percentage = _arg;
    return *this;
  }
  Type & set__steering_angle_rad(
    const double & _arg)
  {
    this->steering_angle_rad = _arg;
    return *this;
  }
  Type & set__engine_speed_rpm(
    const int32_t & _arg)
  {
    this->engine_speed_rpm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    smartcar_msgs::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const smartcar_msgs::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<smartcar_msgs::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<smartcar_msgs::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      smartcar_msgs::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<smartcar_msgs::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      smartcar_msgs::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<smartcar_msgs::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<smartcar_msgs::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<smartcar_msgs::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__smartcar_msgs__msg__Status
    std::shared_ptr<smartcar_msgs::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__smartcar_msgs__msg__Status
    std::shared_ptr<smartcar_msgs::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->battery_voltage_mv != other.battery_voltage_mv) {
      return false;
    }
    if (this->battery_current_ma != other.battery_current_ma) {
      return false;
    }
    if (this->battery_percentage != other.battery_percentage) {
      return false;
    }
    if (this->steering_angle_rad != other.steering_angle_rad) {
      return false;
    }
    if (this->engine_speed_rpm != other.engine_speed_rpm) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  smartcar_msgs::msg::Status_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace smartcar_msgs

#endif  // SMARTCAR_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
