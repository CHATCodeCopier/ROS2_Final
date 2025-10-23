cmake_minimum_required(VERSION 3.5)

project(car_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(smartcar_msgs REQUIRED)

add_library(car_gazebo_plugin SHARED src/car_gazebo_plugin.cpp)


ament_target_dependencies(car_gazebo_plugin
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "ackermann_msgs"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "smartcar_msgs"
  )


  ament_export_libraries(car_gazebo_plugin)

  ament_export_include_directories(include)
  ament_export_dependencies(rclcpp)
  ament_export_dependencies(gazebo_dev)
  ament_export_dependencies(gazebo_ros)
  ament_export_dependencies(sensor_msgs)
  ament_export_dependencies(tf2)
  
  ament_package()
  
  # install(DIRECTORY include/
  #         DESTINATION include)
  
  install(TARGETS
      car_gazebo_plugin
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
