#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Defines launch arguments required to start the HDR ROS2 driver."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'openapi_ip',
            default_value='192.168.1.150',
            description='IP address for the OpenAPI server'
        ),
        Node(
            package='hdr_ros2_driver',
            executable='hdr_ros2_driver_node',
            name='hdr_ros2_driver',
            parameters=[{
                'openapi_ip': LaunchConfiguration('openapi_ip')
            }],
            output='screen',
            emulate_tty=True,
        )
    ])
