# HD Hyundai Robotics ROS2 Messages

#### Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Usage](#usage)

---

## 1. Overview

The `hdr_msgs` package defines the custom ROS2 message types used by the HD Hyundai Robotics software stack. These messages are designed to support communication for robot control, monitoring, and API integration between the ROS2 ecosystem and industrial robot controllers.

---

## 2. Package Structure

| Message Type          | Description                                           |
|-----------------------|-------------------------------------------------------|
| `srv/DateTime.srv`    | Get or set the robot controller's system clock. Input includes full date/time fields (year, mon, day, hour, min, sec). |
| `srv/Emergency.srv`   | Test emergency stop logic with staged parameters (step_no, stop_at, stop_mode). Used for simulation/test scenarios. |
| `srv/ExecuteCmd.srv`  | Execute console command(s) as a list of string lines with configurable interval between executions. Useful for raw low-level commands like rl.stop.  |
| `srv/ExecuteMove.srv` | Executes a robot movement command using a string-based statement, e.g., "move L,spd=1sec,tool=1 [0, 0, 0, 0, 90, 0]". The task_no identifies the task index (typically 0). |
| `srv/FileList.srv`    | Query directory contents from the robot. Can filter by including files or directories via booleans. |
| `srv/FilePath.srv`    | Allows sending or querying file paths for operations such as reading, deleting, or verifying existence. |
| `srv/FileRename.srv`  | Rename or move a file on the robot controller’s file system. |
| `srv/FileSend.srv`    | Uploads a file from the local PC to the robot controller. Requires source and target paths. |
| `srv/IoplcGet.srv`    | Read PLC memory (e.g., relays, M, S, R). Supports both direct and named signal addressing. |
| `srv/IoplcPost.srv`   | Writes to PLC memory (relays) using symbolic names such as M, S, R, or FBx.y.|
| `srv/IoRequest.srv`   | Used to access digital, serial, or user I/O. The type field specifies the kind of I/O, such as 'di', 'do', 'si', or 'so'. and blk_no and sig_no specify block and signal indices. The 'val' field is used when setting I/O values and is ignored when performing a read operation. |
| `srv/JointTrajectoryPoints.srv` | Sends a full ROS2 `trajectory_msgs/JointTrajectory` message (joint names and multiple points) to the robot controller. Used for inserting motion trajectories into the controller buffer via `/project/robot/trajectory/joint_traject_insert_points`. |
| `srv/LogManager.srv`  | Query log entries with filters: category (E, W, etc.), ID range, and timestamp. |
| `srv/Number.srv`      | Generic service that sends/receives an integer. Used for tool number, coordinate system, index setting, etc.|
| `srv/OpCnd.srv`       | Read or write operational condition such as playback mode or user coordinate system. |
| `srv/PoseCur.srv`     | Get current robot pose (position + orientation) in joint or task space, depending on internal configuration. |
| `srv/ProgramCnt.srv`  | Sets the program execution pointer (pno, sno, fno, etc.) to jump to a specific location in the job logic. |
| `srv/ProgramVar.srv`  | Read or assign a variable. Can specify scope (local/global), expression, and persistence. |

---

## 3. Usage

Ensure that `hdr_msgs` is listed as a dependency in both your package.xml and CMakeLists.txt files.

##### Package Dependencies Setting

**package.xml**:

```xml
<depend>hdr_msgs</depend>
```

**CMakeLists.txt**:

```cmake
find_package(hdr_msgs REQUIRED)
ament_target_dependencies(your_node hdr_msgs)
```


##### Python Example

```python
from hdr_msgs.srv import ExecuteCmd
```

##### C++ Example

```cpp
#include "hdr_msgs/srv/execute_cmd.hpp"
```


---

## Maintainers and License

- **Maintainer**: HD Hyundai Robotics R&D Team
- **License**: This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE)
- **Contact**: [kwon.hyojun@hd.com]
