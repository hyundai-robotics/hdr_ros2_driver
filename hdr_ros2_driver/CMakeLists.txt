cmake_minimum_required(VERSION 3.8)
project(hdr_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(std_msgs    REQUIRED)
find_package(std_srvs    REQUIRED)
find_package(hdr_msgs    REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(hdr_ros2_driver 
  src/main.cpp

  src/UDPManager.cpp
  
  src/ComManager.cpp

  src/ServiceManager.cpp
  src/RequestLib.cpp
  
  src/functions/version.cpp
  src/functions/project.cpp
  src/functions/control.cpp
  src/functions/robot.cpp
  src/functions/etc.cpp
  src/functions/task.cpp
  src/functions/file_manager.cpp
  src/functions/io_plc.cpp
  src/functions/console.cpp
)

ament_target_dependencies(hdr_ros2_driver 
  rclcpp
  rclcpp_action

  std_msgs
  std_srvs
  hdr_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
)

include_directories(include)

target_link_libraries(
  hdr_ros2_driver
  curl
)

install (TARGETS
  hdr_ros2_driver
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
