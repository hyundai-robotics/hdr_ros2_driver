#!/usr/bin/env python3
import os
import yaml

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare launch arguments and return LaunchDescription."""
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='hdf7_9',
            description="HDR robot model to use.",
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description="Enable Ignition Gazebo simulation hardware plugin.",
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description="Enable mock hardware interface instead of real or simulation.",
        ),
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value=PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('hdf7_9_moveit_config')),
                'config',
                'initial_positions.yaml'
            ]),
            description="Path to the YAML file containing joint initial positions.",
        ),
    ]

    return LaunchDescription(declared_arguments + [
            OpaqueFunction(function=launch_setup)
    ])
    

def launch_setup(context, *args, **kwargs):
    """Set up MoveIt move_group and RViz nodes based on launch configurations."""
    robot_model = LaunchConfiguration("robot_model")
    use_sim = LaunchConfiguration("use_sim")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    moveit_config_pkg = get_package_share_directory('hdf7_9_moveit_config')
    
    pkg_path = FindPackageShare("hdr_description")
    xacro_path = PathJoinSubstitution([pkg_path, "urdf", "hdr.urdf.xacro"])
    
    robot_description_content = Command(
        [ 
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_path,
            " ",
            "use_sim:=",
            use_sim,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "robot_model:=",
            robot_model,
            " ",
            "name:=",
            "hdr_robot",
            " ",
            "hdr_ros2_control:=",
            "",
            " ",
            "initial_positions_file:=",
            initial_positions_file,
            " ",
        ]
    )

    srdf_xacro = os.path.join(moveit_config_pkg, 'config', 'hdf7_9.srdf.xacro')

    robot_description_semantic_content = load_srdf_semantic(
        srdf_xacro,
        _name='hdr_robot',
    )

    robot_description = {'robot_description': robot_description_content}
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    publish_robot_description_semantic = {"publish_robot_description_semantic": True}
    robot_description_kinematics_yaml = load_yaml("hdf7_9_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {
        "robot_description_kinematics": robot_description_kinematics_yaml
    }

    joint_limits_yaml = load_yaml(
        "hdf7_9_moveit_config", "config/joint_limits.yaml"
    )
    pilz_cartesian_limits_yaml = load_yaml(
        "hdf7_9_moveit_config", "config/pilz_cartesian_limits.yaml"
    )
    robot_description_planning = {
        "robot_description_planning": {
            **joint_limits_yaml,
            **pilz_cartesian_limits_yaml,
        }
    }

    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["pilz", "ompl"],
        "pilz": {
             "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
         },
         "ompl": {
             "planning_plugin": "ompl_interface/OMPLPlanner",
         },
    }
    
    for planner_name, config_yaml in [
        ("pilz", "config/pilz_industrial_motion_planner_planning.yaml"),
        ("ompl", "config/ompl_planning.yaml"),
    ]:
        planner_yaml = load_yaml("hdf7_9_moveit_config", config_yaml)
        planning_pipeline_config[planner_name].update(planner_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml("hdf7_9_moveit_config", "config/moveit_controllers.yaml"),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "move_interactive_markers": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim},
        ],
    )

    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim},
        ],
    )
    
    delay_rviz_node = TimerAction(
        period=3.0,
        actions=[rviz_node],
    )

    return [move_group_node, delay_rviz_node]
    
    
def load_yaml(package_name: str, file_path: str):
    """Load and parse a YAML file from the given ROS2 package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def load_srdf_semantic(xacro_path: str, _name: str) -> str:
    """Process an SRDF Xacro file and return the resulting XML string."""
    mappings = {
        'name':        _name,
    }
    doc = xacro.process_file(xacro_path, mappings=mappings)
    return doc.toxml()
