#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    GroupAction,
)
from launch.conditions import UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
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
            default_value="5",
            description="Buffer size for command data.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Enable Ignition Gazebo simulation hardware plugin.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Enable mock hardware interface instead of real or simulation.",
        ),
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value="",
            description="YAML file name containing joint initial positions.",
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
            'initial_joint_controller',
            default_value="joint_trajectory_controller",
            description="Initial joint controller to activate.",
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
    robot_model = LaunchConfiguration("robot_model")
    openapi_ip = LaunchConfiguration("openapi_ip")
    command_buffer_size = LaunchConfiguration("command_buffer_size")
    use_sim = LaunchConfiguration("use_sim")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    controllers_config_package = LaunchConfiguration("controllers_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    kinematics_config_package = LaunchConfiguration("kinematics_config_package")
    kinematics_file = LaunchConfiguration("kinematics_file")

    pkg_path = FindPackageShare("hdr_description")
    xacro_path = PathJoinSubstitution([pkg_path, "urdf", "hdr.urdf.xacro"])

    controllers_yaml = load_yaml(
        controllers_config_package.perform(context),
        os.path.join("config", controllers_file.perform(context))
    )

    def get_update_rate(controllers_yaml, default_rate=100):
        """Retrieve the update_rate value from the controller_manager configuration."""
        return controllers_yaml.get("controller_manager", {}).get("ros__parameters", {}).get("update_rate", default_rate)

    update_rate = str(get_update_rate(controllers_yaml))

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
            "openapi_ip:=",
            openapi_ip,
            " ",
            "update_rate:=",
            update_rate,
            " ",
            "command_buffer_size:=",
            command_buffer_size,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {"use_sim_time": use_sim}
        ],
    )

    def create_controller_spawner(controllers, active=True):
        """Create a controller spawner node for ROS2 control."""
        arguments = [
            "--controller-manager", "/controller_manager",
        ]
        
        if not active:
            arguments.append("--inactive")
        
        arguments.extend(controllers)
        
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arguments,
            parameters=[{'use_sim_time': use_sim}],
            output='screen',
        )
    
    controllers = PathJoinSubstitution([
        FindPackageShare(controllers_config_package),
        'config',
        controllers_file
    ])

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            controllers,
            {
                'robot_description_kinematics.file_path': PathJoinSubstitution([
                    FindPackageShare(kinematics_config_package),
                    'config',
                    kinematics_file
                ])
            },
            {'use_sim_time': use_sim}
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim")),
    )


    controllers = controllers_yaml.get("controller_state", {}).get("ros__parameters", {})
    active_controllers = controllers.get("active_controllers", [])
    inactive_controllers = controllers.get("inactive_controllers", [])

    joint_controller = initial_joint_controller.perform(context)
    if joint_controller in inactive_controllers:
        inactive_controllers.remove(joint_controller)
        if joint_controller not in active_controllers:
            active_controllers.append(joint_controller)

    controller_spawners = []
    if active_controllers:
        controller_spawners.append(create_controller_spawner(active_controllers, active=True))
    if inactive_controllers:
        controller_spawners.append(create_controller_spawner(inactive_controllers, active=False))

    delayed_controller_spawner = TimerAction(
        period=2.0,
        actions=controller_spawners
    )

    return [
        robot_state_publisher_node,
        controller_manager_node,
        delayed_controller_spawner
    ]

def load_yaml(package_name: str, file_path: str):
    """Load and parse a YAML file from the given ROS2 package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None
