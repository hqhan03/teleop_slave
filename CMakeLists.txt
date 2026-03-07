cmake_minimum_required(VERSION 3.8)
project(teleop_slave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(delto_tcp_comm SHARED libdelto/delto_tcp_comm/src/delto_developer_TCP.cpp)
target_include_directories(delto_tcp_comm PUBLIC libdelto/delto_tcp_comm/include)
target_link_libraries(delto_tcp_comm Boost::system)

add_library(delto_gripper_helper SHARED IMPORTED)
set_target_properties(delto_gripper_helper PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libdelto/delto_hardware/lib/libdelto_gripper_helper.so)

add_library(delto_hardware SHARED libdelto/delto_hardware/src/system_interface.cpp)
target_include_directories(delto_hardware PUBLIC libdelto/delto_hardware/include)
ament_target_dependencies(delto_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle std_srvs)
target_link_libraries(delto_hardware delto_tcp_comm delto_gripper_helper Boost::system)

pluginlib_export_plugin_description_file(hardware_interface delto_hardware.xml)

add_executable(master_bridge_node src/master_bridge_node.cpp)
target_include_directories(master_bridge_node PUBLIC include)
ament_target_dependencies(master_bridge_node rclcpp geometry_msgs sensor_msgs)

add_executable(fairino_lowlevel_controller_node src/fairino_lowlevel_controller_node.cpp)
target_include_directories(fairino_lowlevel_controller_node PUBLIC 
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/include
)
target_link_libraries(fairino_lowlevel_controller_node
  ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.3
)
ament_target_dependencies(fairino_lowlevel_controller_node rclcpp std_msgs sensor_msgs std_srvs)

add_executable(fairino_slave_node src/fairino_slave_node.cpp)
target_include_directories(fairino_slave_node PUBLIC include)
ament_target_dependencies(fairino_slave_node rclcpp geometry_msgs sensor_msgs std_srvs tf2 tf2_geometry_msgs)

add_executable(tesollo_slave_node src/tesollo_slave_node.cpp)
target_include_directories(tesollo_slave_node PUBLIC include libdelto/delto_tcp_comm/include)
target_link_libraries(tesollo_slave_node delto_tcp_comm)
ament_target_dependencies(tesollo_slave_node
  rclcpp
  sensor_msgs
  trajectory_msgs
)

install(TARGETS
  master_bridge_node
  fairino_lowlevel_controller_node
  fairino_slave_node
  tesollo_slave_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/curobo_mppi_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  libfairino/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

install(TARGETS
  delto_tcp_comm
  delto_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  libdelto/delto_hardware/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

install(FILES delto_hardware.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()