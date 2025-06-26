cmake_minimum_required(VERSION 3.8)
project(hdr_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hdr_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hdr_client_driver REQUIRED)

include_directories(
  include
)

add_executable(hdr_ros2_driver_node
  src/hdr_ros2_node.cpp
  src/service_manager.cpp
  
  src/ros2_services/version.cpp
  src/ros2_services/project.cpp
  src/ros2_services/control.cpp
  src/ros2_services/robot.cpp
  src/ros2_services/io_plc.cpp
  src/ros2_services/etc.cpp
  src/ros2_services/file_manager.cpp
  src/ros2_services/task.cpp
  src/ros2_services/console.cpp
  src/ros2_services/robot_pose.cpp
)

ament_target_dependencies(hdr_ros2_driver_node
  rclcpp
  std_srvs
  hdr_msgs
  sensor_msgs
  rclcpp_action
  control_msgs
  trajectory_msgs
)

target_include_directories(hdr_ros2_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hdr_ros2_driver_node
  hdr_client_driver::hdr_client_driver
)

set_target_properties(hdr_ros2_driver_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../../hdr_client_driver/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS hdr_ros2_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
)

ament_package()
