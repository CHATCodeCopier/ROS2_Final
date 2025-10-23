cmake_minimum_required(VERSION 3.8)
project(smartcar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate code for our messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

