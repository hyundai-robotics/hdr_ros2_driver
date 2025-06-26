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
        DeclareLaunchArgument(
            'openapi_port',
            default_value='8888',
            description='Port number for the OpenAPI server'
        ),
        DeclareLaunchArgument(
            'robot_pose_enable',
            default_value='true',
            description='Enable robot pose publisher'
        ),
        DeclareLaunchArgument(
            'robot_pose_hz',
            default_value='100',
            description='Publishing frequency (Hz) for robot pose'
        ),
        DeclareLaunchArgument(
            'robot_pose_topic_name',
            default_value='/hdr_ros2_driver/joint_states',
            description='Topic name for published robot joint states'
        ),
        DeclareLaunchArgument(
            'robot_pose_action_name',
            default_value='/hdr_ros2_driver/follow_joint_trajectory',
            description='Action name for robot Trajectory'
        ),

        Node(
            package='hdr_ros2_driver',
            executable='hdr_ros2_driver_node',
            name='hdr_ros2_driver',
            parameters=[{
                'openapi_ip': LaunchConfiguration('openapi_ip'),
                'openapi_port': LaunchConfiguration('openapi_port'),
                'robot_pose_enable': LaunchConfiguration('robot_pose_enable'),
                'robot_pose_hz': LaunchConfiguration('robot_pose_hz'),
                'robot_pose_topic_name': LaunchConfiguration('robot_pose_topic_name'),
                'robot_pose_action_name': LaunchConfiguration('robot_pose_action_name')
            }],
            output='screen',
            emulate_tty=True,
        )
    ])
