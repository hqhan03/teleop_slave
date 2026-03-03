cmake_minimum_required(VERSION 3.8)
project(manus_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(manus_receiver_node src/manus_receiver_node.cpp)
ament_target_dependencies(manus_receiver_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  manus_receiver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()