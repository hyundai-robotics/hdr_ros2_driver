# HD Hyundai Robotcis Hardware Interface

#### Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)

---

## 1. Overview

The `hdr_hardware_interface` package provides a `ros2_control` SystemInterface for connecting HD Hyundai Robotics' Open API-based controllers with the ROS2 control framework. It maps joint-level position state/command interfaces to HTTP-based robot services, handling controller lifecycle, real-time pose tracking, and safety checks.

---

## 2. Package Structure

| Directory                           | Description                                                                 |
| ----------------------------------- | ----------------------------------------------------------------------------|
| `include/`                          | C++ headers including `HDRRobotHardware` and utility helpers.                |
| `src/`                              | Implementation of SystemInterface logic.                                     |
| `launch/`                           | Provides a launch file to bring up the `ros2_control` interface.             |
| `config/`                           | Contains YAML configuration files for controllers and kinematics settings.   |
| `hdr_hardware_interface_plugin.xml` | Export metadata for pluginlib.                                               |

---

## 3. Usage

##### Launch the HDR hardware interface with ros2_control

```bash
ros2 launch hdr_hardware_interface ros2_control.launch.py \
  robot_model:=hdf7_9 \
  openapi_ip:=192.168.1.150 \
  openapi_port:=8888
```

##### Plugin Configuration

To activate the hardware interface, include it within your URDF or xacro under `<ros2_control>`

```xml
<ros2_control name="HDRRobotHardware" type="system">
  <hardware>
    <plugin>hdr_hardware_interface/HDRRobotHardware</plugin>
    <param name="robot_model">ha006b</param>
    <param name="openapi_ip">192.168.1.150</param>
    <param name="openapi_port">8888</param>
  </hardware>
</ros2_control>
```

#### Configuration Options

| Argument Name                      | Type   | Default                         | Description                                                                 |
|---------------------------|--------|----------------------------------|-----------------------------------------------------------------------------|
| `robot_model`             | string | `"ha006b"`                      | Robot model name used in URDF and validation                               |
| `openapi_ip`              | string | `"192.168.1.150"`               | IP address of robot controller’s HTTP API                                  |
| `openapi_port`            | int    | `8888`                          | HTTP port used by robot OpenAPI server                                     |
| `command_port`         | int    | `8000`                          | Port number for trajectory command transmission                              |
| `command_start_time`   | float  | `-1.0`                          | Start time for command execution (-1.0 for immediate start)             |
| `command_buffer_size`  | int    | `5`                             | Buffer size for command data management                                 |
| `use_sim`                 | bool   | `false`                         | Enables simulation mode using the `gz_ros2_control/GazeboSimSystem` plugin, typically for integration with Ignition Gazebo<br>The `use_sim_time` parameter is also set to true to synchronize with simulation time     |
| `use_mock_hardware`       | bool   | `false`                         | Enables a mock hardware interface using `mock_components/GenericSystem` for testing without a physical robot   |
| `initial_positions_file`  | string | `""`                            | Optional YAML file specifying initial joint positions                       |
| `controllers_config_package` | string | `"hdr_hardware_interface"`     | Package name containing config YAMLs                                       |
| `controllers_file`        | string | `"default_controllers.yaml"`   | Controller configuration YAML filename                                     |
| `kinematics_file`         | string | `"default_kinematics.yaml"`    | Kinematics plugin configuration YAML filename                              |


#### Topics

| Topic name                   | Message type                   | Description                               |
| ---------------------------- | ------------------------------ | ----------------------------------------- |
| `/joint_states`              | sensor_msgs::msg::JointState | Publishes the current joint states, including position |
| `/controller_manager/status` | lifecycle_msgs::msg::State   | Lifecycle status of ros2\_control manager |

#### Actions

| Action name                                            | Action type                                   | Description                                |
| ------------------------------------------------------ | --------------------------------------------- | ------------------------------------------ |
| `/joint_trajectory_controller/follow_joint_trajectory` | control_msgs::action::FollowJointTrajectory | Executes joint trajectory goals via action |

#### Services

| Service name                                   | Service Type                                         | Description                         |
| ---------------------------------------------- | ---------------------------------------------------- | ----------------------------------- |
| `/controller_manager/list_controllers`         | controller_manager_msgs/srv/ListControllers        | Returns a list of active controllers     |
| `/controller_manager/list_hardware_interfaces` | controller_manager_msgs/srv/ListHardwareInterfaces | Returns the available joint command and state interfaces |
| `/controller_manager/switch_controller`        | controller_manager_msgs/srv/SwitchController       | Activate or deactivate controllers  |
| `/controller_manager/load_controller`          | controller_manager_msgs/srv/LoadController         | Load controller by name             |
| `/controller_manager/unload_controller`        | controller_manager_msgs/srv/UnloadController       | Unloads and removes the specified controller    |

---

## 4. Troubleshooting

- **Controller not found**: Check plugin name in URDF, and rebuild.
- **Timeout on launch**: Verify that the OpenAPI IP address is reachable from the host machine.
- **No joint states**: Ensure the driver is running and that the `robot_pose` feature is enabled.
- **Interface fails to start**: Check for supported firmware version ≥ **60.34-00**. (This version is planned for release in **October**.)

For detailed API documentation, please refer to the [HDR Open API Documentation](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/).

---

## Maintainers and License

- **Maintainer**: HD Hyundai Robotics R&D Team
- **License**: This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE)
- **Contact**: [kwon.hyojun@hd.com, vewry12@hd.com]
