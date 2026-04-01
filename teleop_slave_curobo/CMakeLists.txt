cmake_minimum_required(VERSION 3.8)
project(teleop_slave_curobo)

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

set(FAIRINO_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libfairino/lib)
set(FAIRINO_LIB_REALNAME libfairino.so.2.3.3)
set(FAIRINO_LIB_REALPATH ${FAIRINO_LIB_DIR}/${FAIRINO_LIB_REALNAME})
set(FAIRINO_LIB_SONAME libfairino.so.2)
set(FAIRINO_LIB_SONAME_PATH ${FAIRINO_LIB_DIR}/${FAIRINO_LIB_SONAME})
set(FAIRINO_LIB_LINKNAME libfairino.so)

if(NOT EXISTS ${FAIRINO_LIB_SONAME_PATH})
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${FAIRINO_LIB_REALNAME} ${FAIRINO_LIB_SONAME}
    WORKING_DIRECTORY ${FAIRINO_LIB_DIR}
    RESULT_VARIABLE FAIRINO_SONAME_LINK_RESULT
  )
  if(NOT FAIRINO_SONAME_LINK_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to create ${FAIRINO_LIB_SONAME} symlink for bundled Fairino SDK")
  endif()
endif()

if(NOT EXISTS ${FAIRINO_LIB_DIR}/${FAIRINO_LIB_LINKNAME})
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${FAIRINO_LIB_SONAME} ${FAIRINO_LIB_LINKNAME}
    WORKING_DIRECTORY ${FAIRINO_LIB_DIR}
    RESULT_VARIABLE FAIRINO_LINK_RESULT
  )
  if(NOT FAIRINO_LINK_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to create ${FAIRINO_LIB_LINKNAME} symlink for bundled Fairino SDK")
  endif()
endif()

function(configure_fairino_runtime target_name)
  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH "${FAIRINO_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN/.."
    INSTALL_RPATH_USE_LINK_PATH FALSE
  )
  target_link_options(${target_name} PRIVATE "-Wl,--disable-new-dtags")
endfunction()

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
ament_target_dependencies(fr5_teleop_utils
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(fr5_teleop_utils
  ${FAIRINO_LIB_REALPATH}
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
configure_fairino_runtime(fairino_lowlevel_controller_node)
ament_target_dependencies(fairino_lowlevel_controller_node rclcpp std_msgs sensor_msgs std_srvs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(fairino_slave_node src/fairino_slave_node.cpp)
target_include_directories(fairino_slave_node PUBLIC include)
target_link_libraries(fairino_slave_node fr5_teleop_utils)
configure_fairino_runtime(fairino_slave_node)
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

install(FILES
  ${FAIRINO_LIB_REALPATH}
  DESTINATION lib
)

install(CODE
  "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink \"${FAIRINO_LIB_REALNAME}\" \"${FAIRINO_LIB_SONAME}\" WORKING_DIRECTORY \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib\")"
)

install(CODE
  "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink \"${FAIRINO_LIB_SONAME}\" \"${FAIRINO_LIB_LINKNAME}\" WORKING_DIRECTORY \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib\")"
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
  scripts/calibrate_tracker_frame.py
  scripts/curobo_motion_generator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(ament_cmake_gtest_FOUND)
  ament_add_gtest(test_fr5_teleop_utils
    test/test_fr5_teleop_utils.cpp
  )
  if(TARGET test_fr5_teleop_utils)
    target_link_libraries(test_fr5_teleop_utils fr5_teleop_utils)
    configure_fairino_runtime(test_fr5_teleop_utils)
    ament_target_dependencies(test_fr5_teleop_utils geometry_msgs tf2 tf2_geometry_msgs)
  endif()

  ament_add_gtest(test_kalman_filter
    test/test_kalman_filter.cpp
  )
  if(TARGET test_kalman_filter)
    target_include_directories(test_kalman_filter PUBLIC include)
  endif()
endif()

ament_package()
