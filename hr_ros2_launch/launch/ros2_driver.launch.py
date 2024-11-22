from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate ROS 2 launch description with UDP, OpenAPI, and motor configurations"""
    
    # Parameter definitions
    param_configs = {
        # Network settings
        'udp_remote_host': ('192.168.1.150', 'UDP remote host IP address'),
        'udp_remote_port': ('7000', 'UDP remote port'),
        'udp_local_host': ('0.0.0.0', 'UDP local host'),
        'udp_local_port': ('7001', 'UDP local port'),
        
        # OpenAPI settings
        'openapi_ip': ('192.168.1.150', 'OpenAPI server IP address'),
        'openapi_port': ('8888', 'OpenAPI server port'),
        
        # Joint and action settings
        'desired_joint_order': (
            '["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]',
            'Desired joint order as JSON-formatted string'),
        'joint_states_topic': ('joint_states', 'Joint states topic name'),
        'action_name': (
            '/arm_controller/follow_joint_trajectory',
            'Action server name for following joint trajectories'
        ),
        
        # Motor settings
        'motor_pose_state': ('true', 'Enable/disable motor pose updates'),
        'motor_pose_interval_ms': ('100', 'Motor position update interval in milliseconds'),
        'motor_pose_topic': ('/api_agent/joint_position', 'Topic name for motor pose updates'),
        'pose_param' : ('[0, -1, 0, 0]', '''Default joint position parameters:
           1) task_no: Task number (0~7)
              - Not specified: Applied as task 0
              - >=0: When mechinfo is not specified, current mechinfo of the task is applied
           2) crd: Coordinate system
              - Not specified: Get all tcp, axis, encoder values
              - <0: Follow current recorded coordinate system  
              - >=0: Specific coordinate system
           3) ucrd_no: User coordinate system number (only specified when crd is user)
           4) mechinfo: Mechanism information
           ''')
    }

    # Create launch arguments
    launch_arguments = [
        DeclareLaunchArgument(
            name=name,
            default_value=default,
            description=desc
        )
        for name, (default, desc) in param_configs.items()
    ]

    # Node configuration
    node = Node(
        package='hr_ros2_driver',
        executable='hr_ros2_driver',
        name='hr_ros2_driver',
        output='screen',
        parameters=[{
            param: LaunchConfiguration(param)
            for param in param_configs.keys()
        }]
    )

    return LaunchDescription(launch_arguments + [node])