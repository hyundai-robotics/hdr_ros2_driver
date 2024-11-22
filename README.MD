# HD Hyundai Robotics Robots ROS2 Driver

## Table of Contents
* [Introduction](#introduction)
* [System Requirements](#system-requirements)
* [Packages](#packages)
* [Installation](#installation)
* [Control the Robot](#control-the-robot)
* [Launch File Explanation](#launch-file-explanation)

## Introduction

The HD Hyundai Robotics ROS 2 Driver provides a ROS 2 interface for controlling HD Hyundai robotic arms. This driver enables users to integrate HD Hyundai robots into ROS 2 applications, facilitating advanced functionalities such as motion planning, perception, and AI integration.

## System Requirements

* Operating System: Ubuntu 22.04 LTS
* ROS Version: ROS 2 Humble Hawksbill

## Packages in the Repository:

| Package Name | Description |
|--------------|-------------|
| `api_msgs` | Package defining ROS 2 messages for communication with HD Hyundai Robotics robot. |
| `hr_ros2_driver` | Contains ROS 2 launch files and configuration parameters for starting the robot driver. |
| `hr_ros2_launch` | A ROS2 package that interfaces with HD Hyundai Robotics robot. |

## Installation

Follow the steps below to install and set up the HD Hyundai Robotics ROS2 Driver:

1. **Git clone package:**

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/hyundai-robotics/hr_ros2_driver.git
   ```

2. **Install dependencies:**

   ```bash
   cd ~/ros2_ws/
   sudo apt update && sudo apt-get install -y curl libboost-all-dev nlohmann-json3-dev
   sudo apt install -y ros-humble-ros2-control-msgs ros-humble-trajectory-msgs ros-humble-sensor-msgs
   rosdep install --ignore-src --from-paths src -y
   ```

3. **Build the workspace using `colcon`:**

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

## Controlling the Robot

### Setup the Teach Pendant (TP)

**Important:** If there are any errors displayed on the Teach Pendant (TP), the robot will not operate. Please ensure that the TP is error-free before proceeding.

Before controlling the robot through ROS 2, follow these steps on the robot controller:

1. Execute program number `7000` on the Teach Pendant (TP).
2. Change the robot mode to 'auto'.
3. Motor ON on the Teach Pendant (TP).
4. Start Program on the Teach Pendant (TP).

### Start the ROS2 Driver

After completing these steps, you can proceed with the following commands to control the robot:

1. **Source the setup script to overlay the workspace on your environment:**

   ```bash
   source install/setup.bash
   ```

2. **After connecting to the robot controller via network, launch the ROS 2 driver:**

   ```bash
   ros2 launch hr_ros2_launch ros2_driver.launch.py
   ```

3. **Upload the job files to the controller:**

   Check the location of the job code in the package and run the following commands:

   ```bash
   ros2 service call /api_agent/file/post/files api_msgs/srv/FileSend "{target_file: '/project/jobs/7000_ethernet.job', source_file: '/home/<your_username>/hr_ros2_driver_github/7000_ethernet.job'}"
   ```

   ```bash
   ros2 service call /api_agent/file/post/files api_msgs/srv/FileSend "{target_file: '/project/jobs/7001.job', source_file: '/home/<your_username>/hr_ros2_driver_github/7001.job'}"
   ```

   **Note:** Replace `<your_username>` with your actual username.

   Alternatively, you can copy the `7000_ethernet.job` and `7001.job` files onto a USB drive, connect it to the Teach Pendant (TP), and paste the files into the `project/jobs` directory. After copying, execute program number `7000` on the TP.


### Examples

1. **Turn the robot's motor ON:**

   ```bash
   ros2 service call /api_agent/robot/post/motor_control std_srvs/srv/SetBool "{data: true}"
   ```

2. **Start the robot program:**

   ```bash
   ros2 service call /api_agent/robot/post/robot_control std_srvs/srv/SetBool "{data: true}"
   ```

3. **Stop the robot program:**

   ```bash
   ros2 service call /api_agent/robot/post/robot_control std_srvs/srv/SetBool "{data: false}"
   ```

4. **Turn the robot's motor OFF:**

   ```bash
   ros2 service call /api_agent/robot/post/motor_control std_srvs/srv/SetBool "{data: false}"
   ```

**Note:** Ensure that you have completed the necessary setup on the Teach Pendant before sending commands through ROS 2.

## ROS2 Interface

### Actions
The driver provides a ROS2 Action interface for trajectory control:

* **Action Server**: `follow_joint_trajectory` (type: `control_msgs::action::FollowJointTrajectory`)
  - Controls robot motion by following specified joint trajectories
  - Action name can be configured via the `action_name` parameter in the launch file
  - Default action name: `/arm_controller/follow_joint_trajectory`
  - Used for executing motion plans and coordinated joint movements

### Topics
The driver publishes the robot's state information:

* **Joint States**: `joint_states` (type: `sensor_msgs::msg::JointState`)
  - Publishes current joint positions, velocities, and efforts
  - Topic name can be configured via the `joint_states_topic` parameter
  - Default topic name: `joint_states`
  - Publishing rate: Real-time updates from the robot controller
  - Data includes:
    - Position of each joint in radians
    - Velocity of each joint in radians/second
    - Effort (torque) of each joint (if available)

**Note:** The joint order in both the action goals and joint state messages follows the order specified in the `desired_joint_order` parameter of the launch file.

## Launch File Explanation

The `ros2_driver.launch.py` file includes several parameters that can be configured to suit your setup. Here's the complete launch file configuration:

```python
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
```

# ROS2 Driver Launch Parameters

## Network Parameters

### UDP Communication Parameters
* `udp_remote_host` (default: '192.168.1.150')
  * IP address of the UDP remote host (robot controller)
  * Target IP for communication with the robot controller

* `udp_remote_port` (default: '7000')
  * UDP remote port number on the robot controller
  * Target port for sending data

* `udp_local_host` (default: '0.0.0.0')
  * Local IP address for UDP communication
  * Set to receive from all network interfaces

* `udp_local_port` (default: '7001')
  * Local UDP port number for incoming data
  * Port for receiving data from the robot

### OpenAPI Parameters
* `openapi_ip` (default: '192.168.1.150')
  * IP address of the OpenAPI server
  * Address of the robot controller's API server

* `openapi_port` (default: '8888')
  * Port number for the OpenAPI server
  * Port for API communication

## Robot Configuration Parameters

### Joint Configuration
* `desired_joint_order` (default: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
  * JSON-formatted string specifying the desired order of robot joints

* `joint_states_topic` (default: 'joint_states')
  * ROS 2 topic name where joint states are published
  * Topic for subscribing to current joint state information

### Motion Control Parameters
* `action_name` (default: '/arm_controller/follow_joint_trajectory')
  * Action server name for following joint trajectories
  * Action server for controlling robot movement

### Motor Parameters
* `motor_pose_state` (default: 'true')
  * Enable/disable motor pose updates
  * Controls whether motor position information is updated

* `motor_pose_interval_ms` (default: '100')
  * Motor position update interval in milliseconds
  * Sets the frequency of position information updates

* `motor_pose_topic` (default: '/api_agent/joint_position')
  * Topic name for motor pose updates
  * Topic where motor position information is published

* `pose_param` (default: [0, -1, 0, 0])
  * Default joint position parameters
  * Components:
    1. task_no: Task number (0~7)
       - Not specified: Applied as task 0
       - >=0: Current mechinfo of the task is applied when mechinfo is not specified
    2. crd: Coordinate system
       - Not specified: Get all tcp, axis, encoder values
       - <0: Follow current recorded coordinate system
       - >=0: Specific coordinate system
    3. ucrd_no: User coordinate system number (only specified when crd is user)
    4. mechinfo: Mechanism information

## Usage Notes
* All parameters can be configured through the launch file or command line arguments
* Default values are set for standard setup but may need adjustment based on your specific robot configuration
* Network parameters should match your robot controller's network settings
* Joint order must match the physical configuration of your robot
* Update intervals and topic names can be adjusted based on your application's requirements
