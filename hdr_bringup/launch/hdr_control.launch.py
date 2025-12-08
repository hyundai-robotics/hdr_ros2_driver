#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare launch arguments and return the LaunchDescription."""
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            default_value='ha006b',
            choices=[
                'ha006b', 'hdf7_9', 'hdf8_8', 'hdr10l_19',
                'hdr20_17', 'hdr50_22', 'hdr220_26', 'hh020', 'hdr35_20'
            ],
            description="HDR robot model to use.",
        ),
        DeclareLaunchArgument(
            "openapi_ip",
            default_value="192.168.1.150",
            description="IP address of the robots OpenAPI server.",
        ),
        DeclareLaunchArgument(
            "command_buffer_size",
            default_value="20",
            description="Buffer size for command data.",
        ),
        DeclareLaunchArgument(
            'controllers_config_package',
            default_value="hdr_hardware_interface",
            description="Name of the package providing controller configurations.",
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value="default_controllers.yaml",
            description="YAML file name defining the ROS2 controllers to load.",
        ),
        DeclareLaunchArgument(
            'kinematics_config_package',
            default_value="hdr_hardware_interface",
            description="Name of the package providing robot kinematics file.",
        ),
        DeclareLaunchArgument(
            'kinematics_file',
            default_value="default_kinematics.yaml",
            description="YAML file name defining the robot kinematics.",
        ),
    ]

    return LaunchDescription(declared_arguments + [
            OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    """Set up and return the nodes/actions for ros2_control and state publishing."""
    robot_model = LaunchConfiguration("robot_model").perform(context)
    openapi_ip = LaunchConfiguration("openapi_ip").perform(context)
    command_buffer_size = LaunchConfiguration("command_buffer_size").perform(context)
    controllers_config_package = LaunchConfiguration("controllers_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    kinematics_config_package = LaunchConfiguration("kinematics_config_package")
    kinematics_file = LaunchConfiguration("kinematics_file")

    hdr_ros2_driver_share = get_package_share_directory('hdr_ros2_driver')

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(controllers_config_package),
                'launch',
                'ros2_control.launch.py',
            ])
        ),
        launch_arguments={
            'robot_model': robot_model,
            'openapi_ip': openapi_ip,
            'command_buffer_size': command_buffer_size,
            'controllers_config_package': controllers_config_package,
            'controllers_file': controllers_file,
            'kinematics_config_package': kinematics_config_package,
            'kinematics_file': kinematics_file,
        }.items(),
    )

    ros2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                hdr_ros2_driver_share,
                'launch',
                'hdr_ros2_driver.launch.py',
            )
        ),
        launch_arguments={
            'openapi_ip': openapi_ip
        }.items(),
    )

    return [ros2_control_launch, ros2_driver_launch]
