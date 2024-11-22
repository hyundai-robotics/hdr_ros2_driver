# HD Hyundai Robotics Robots ROS2 Driver

## Table of Contents
* [Introduction](#introduction)
* [System Requirements](#system-requirements)
* [Packages](#packages)
* [Installation](#installation)
* [Control the Robot](#control-the-robot)
* [Launch File Explanation](#launch-file-explanation)

## Introduction

The HD Hyundai Robotics Robots ROS2 Driver provides a ROS 2 interface for controlling HD Hyundai robotic arms. This driver allows users to integrate HD Hyundai robots into ROS 2 applications, enabling advanced robotic functionalities such as motion planning, perception, and AI integration.

For detailed information, refer to the following documentation:
[https://hrbook-hrc.web.app/#/view/doc-hr-simulation-gz/english/README](https://hrbook-hrc.web.app/#/view/doc-hr-simulation-gz/english/README)

## System Requirements

* Operating System: Ubuntu 22.04 LTS
* ROS Version: ROS 2 Humble Hawksbill

## Packages

| Package Name | Description |
|--------------|-------------|
| **hr_ros2_launch** | Contains ROS 2 launch files and configuration parameters for starting the robot driver. |
| **install** | Includes installation scripts and necessary resources for setting up the driver. |

## Installation

Follow the steps below to install and set up the HD Hyundai Robotics ROS2 Driver:

1. **Create a ROS 2 workspace and navigate to the source directory:**

   ```bash
   mkdir -p ~/ros2_ws/src
   ```

2. **Move the `install` and `hr_ros2_launch` packages into your workspace:**

   ```bash
   cd ~/hr_ros2_driver_github
   mv install ~/ros2_ws
   mv hr_ros2_launch ~/ros2_ws/src
   ```

3. **Build the workspace using `colcon`:**

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

4. **Source the setup script to overlay the workspace on your environment:**

   ```bash
   source install/setup.bash
   ```

5. **After connecting to the robot controller via network, launch the ROS 2 driver:**

   ```bash
   ros2 launch hr_ros2_launch ros2_driver.launch.py
   ```

6. **Upload the job files to the controller:**

   Check the location of the job code in the package and run the following commands:

   ```bash
   ros2 service call /api_agent/file/post/files api_msgs/srv/FileSend "{target_file: '/project/jobs/7000_ethernet.job', source_file: '/home/<your_username>/hr_ros2_driver_github/7000_ethernet.job'}"
   ```

   ```bash
   ros2 service call /api_agent/file/post/files api_msgs/srv/FileSend "{target_file: '/project/jobs/7001.job', source_file: '/home/<your_username>/hr_ros2_driver_github/7001.job'}"
   ```

   **Note:** Replace `<your_username>` with your actual username.

   Alternatively, you can copy the `7000_ethernet.job` and `7001.job` files onto a USB drive, connect it to the Teach Pendant (TP), and paste the files into the `project/jobs` directory. After copying, execute program number `7000` on the TP.

## Control the Robot

**Important:** If there are any errors displayed on the Teach Pendant (TP), the robot will not operate. Please ensure that the TP is error-free before proceeding.

Before controlling the robot through ROS 2, follow these steps on the robot controller:

1. Execute program number `7000` on the Teach Pendant (TP).
2. Change the robot mode to 'auto'.
3. Motor ON on the Teach Pendant (TP).
4. Start Program on the Teach Pendant (TP).

After completing these steps, you can proceed with the following commands to control the robot:

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

[나머지 내용 동일...]
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

## Launch File Explanation

The `ros2_driver.launch.py` file includes several parameters that can be configured to suit your setup. Here's the complete launch file configuration:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Combined UDP and OpenAPI arguments
    launch_arguments = [
        DeclareLaunchArgument(
            name, default_value=default, description=desc
        ) for name, default, desc in [
            ('udp_remote_host', '192.168.1.150', 'UDP remote host IP address'),
            ('udp_remote_port', '7000', 'UDP remote port'),
            ('udp_local_host', '0.0.0.0', 'UDP local host'),
            ('udp_local_port', '7001', 'UDP local port'),
            ('desired_joint_order', '["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint" ]', 'Desired joint order as a JSON-formatted string'),
            ('joint_states_topic', 'joint_states', 'Joint states topic name'),
            ('action_name', '/arm_controller/follow_joint_trajectory', 'Name of the action server for following joint trajectories'),
            ('openapi_ip', '192.168.1.150', 'OpenAPI server IP address'),
            ('openapi_port', '8888', 'OpenAPI server port')
        ]
    ]

    node = Node(
        package='hr_ros2_driver',
        executable='hr_ros2_driver',
        name='hr_ros2_driver',
        output='screen',
        parameters=[{
            'udp_remote_host': LaunchConfiguration('udp_remote_host'),
            'udp_remote_port': LaunchConfiguration('udp_remote_port'),
            'udp_local_host': LaunchConfiguration('udp_local_host'),
            'udp_local_port': LaunchConfiguration('udp_local_port'),
            'desired_joint_order': LaunchConfiguration('desired_joint_order'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'action_name': LaunchConfiguration('action_name'),
            'openapi_ip': LaunchConfiguration('openapi_ip'),
            'openapi_port': LaunchConfiguration('openapi_port')
        }]
    )

    return LaunchDescription(launch_arguments + [node])
```

### Launch Parameters

#### UDP Communication Parameters:
* `udp_remote_host`: `'192.168.1.150'` - The IP address of the UDP remote host (robot controller)
* `udp_remote_port`: `'7000'` - The UDP remote port number on the robot controller
* `udp_local_host`: `'0.0.0.0'` - The local IP address to bind for UDP communication
* `udp_local_port`: `'7001'` - The local UDP port number for incoming data

#### Robot Configuration Parameters:
* `desired_joint_order`: `'["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]'` - A JSON-formatted string specifying the desired order of the robot joints
* `joint_states_topic`: `'joint_states'` - The name of the ROS 2 topic where joint states are published
* `action_name`: `'/arm_controller/follow_joint_trajectory'` - The name of the action server used for following joint trajectories

#### OpenAPI Parameters:
* `openapi_ip`: `'192.168.1.150'` - The IP address of the OpenAPI server
* `openapi_port`: `'8888'` - The port number for the OpenAPI server

**Note:** 
- All parameters can be overridden when launching the node using command-line arguments
- Ensure that the IP addresses and ports match your network configuration
- The OpenAPI server is used for additional robot control and monitoring functionality
- The default joint order matches the standard configuration of the robot, modify if your setup differs

## Distribution Notice

### Confidentiality and Distribution Restrictions

This software driver and all associated documentation are proprietary to HD Hyundai Robotics and are provided for authorized use only. The following restrictions apply:

1. **Confidentiality**: This software and documentation contain confidential and proprietary information of HD Hyundai Robotics.

2. **Distribution Prohibition**: 
   - Unauthorized distribution, sharing, or transfer of this software is strictly prohibited
   - Do not upload or share this driver on public repositories or forums
   - Do not distribute to third parties without explicit written permission from HD Hyundai Robotics

3. **Usage Restrictions**:
   - For internal use only within authorized organizations
   - Must be used in accordance with the terms of your license agreement with HD Hyundai Robotics

4. **Copyright**: © 2024 HD Hyundai Robotics. All rights reserved.

Any unauthorized use, reproduction, or distribution of this software may result in legal action. For questions about usage rights or distribution permissions, please contact HD Hyundai Robotics directly.