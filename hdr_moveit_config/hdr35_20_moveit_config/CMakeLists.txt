cmake_minimum_required(VERSION 3.8)
project(hdr35_20_moveit_config)

find_package(ament_cmake REQUIRED)

find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)
find_package(pilz_industrial_motion_planner REQUIRED)

find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(xacro REQUIRED)

find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()