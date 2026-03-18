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
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_gtest QUIET)

add_library(delto_tcp_comm SHARED libdelto/delto_tcp_comm/src/delto_developer_TCP.cpp)
target_include_directories(delto_tcp_comm PUBLIC libdelto/delto_tcp_comm/include)
target_link_libraries(delto_tcp_comm Boost::system)

add_library(delto_gripper_helper SHARED IMPORTED)
set_target_properties(delto_gripper_helper PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libdelto/delto_hardware/lib/libdelto_gripper_helper.so)

add_library(delto_hardware SHARED libdelto/delto_hardware/src/system_interface.cpp)
target_include_directories(delto_hardware PUBLIC libdelto/delto_hardware/include)
ament_target_dependencies(delto_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle std_srvs)
target_link_libraries(delto_hardware delto_tcp_comm delto_gripper_helper Boost::system)

add_library(fr5_teleop_utils STATIC
  src/fairino_robot_interface.cpp
  src/fr5_teleop_utils.cpp
)
target_include_directories(fr5_teleop_utils PUBLIC
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/include
)
ament_target_dependencies(fr5_teleop_utils geometry_msgs tf2)
target_link_libraries(fr5_teleop_utils
  ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib/libfairino.so.2.3.3
)

pluginlib_export_plugin_description_file(hardware_interface delto_hardware.xml)

add_executable(master_bridge_node src/master_bridge_node.cpp)
target_include_directories(master_bridge_node PUBLIC include)
ament_target_dependencies(master_bridge_node rclcpp geometry_msgs sensor_msgs tf2)

add_executable(fairino_lowlevel_controller_node src/fairino_lowlevel_controller_node.cpp)
target_include_directories(fairino_lowlevel_controller_node PUBLIC 
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/include
)
target_link_libraries(fairino_lowlevel_controller_node
  fr5_teleop_utils
)
ament_target_dependencies(fairino_lowlevel_controller_node rclcpp std_msgs sensor_msgs std_srvs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(fairino_slave_node src/fairino_slave_node.cpp)
target_include_directories(fairino_slave_node PUBLIC include)
target_link_libraries(fairino_slave_node fr5_teleop_utils yaml-cpp)
ament_target_dependencies(fairino_slave_node rclcpp geometry_msgs sensor_msgs std_srvs tf2 tf2_geometry_msgs)

add_executable(tesollo_slave_node src/tesollo_slave_node.cpp)
target_include_directories(tesollo_slave_node PUBLIC include libdelto/delto_tcp_comm/include)
target_link_libraries(tesollo_slave_node delto_tcp_comm yaml-cpp)
ament_target_dependencies(tesollo_slave_node
  rclcpp
  sensor_msgs
  trajectory_msgs
  std_srvs
)

install(TARGETS
  fr5_teleop_utils
  master_bridge_node
  fairino_lowlevel_controller_node
  fairino_slave_node
  tesollo_slave_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

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

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/manus_retarget_vendor.py
  src/manus_retarget_vendor
  DESTINATION lib/${PROJECT_NAME}
)

if(ament_cmake_gtest_FOUND)
  ament_add_gtest(test_fr5_teleop_utils
    test/test_fr5_teleop_utils.cpp
  )
  if(TARGET test_fr5_teleop_utils)
    target_link_libraries(test_fr5_teleop_utils fr5_teleop_utils)
    ament_target_dependencies(test_fr5_teleop_utils geometry_msgs tf2)
  endif()
endif()

ament_package()
