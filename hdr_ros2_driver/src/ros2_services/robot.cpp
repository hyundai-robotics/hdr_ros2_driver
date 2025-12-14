#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the robot motor state (on/off).
 *
 * @details
 * This service retrieves the current motor state of the robot.
 * 🔗 API Reference:
 * [Get Motor
 * State](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/1-motor_on_state)
 *
 */
void ServiceManager::HandleGetRobotMotorState(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetRobotMotorState();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get robot motor state: %s", e.what());
  }
}

/**
 * @param request The request object containing task number, coordinate system, etc.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the robot's current pose.
 *
 * @details
 * This service retrieves the robot's current pose based on the provided task number and coordinate
 * system.
 * 🔗 API Reference:
 * [Get Robot
 * Pose](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/2-po_cur)
 *
 */
void ServiceManager::HandleGetRobotPoCur(
    const std::shared_ptr<hdr_msgs::srv::PoseCur::Request> request,
    std::shared_ptr<hdr_msgs::srv::PoseCur::Response> response) {
  try {
    auto [result, success] =
        driver_->GetRobotPoCur(request->task_no, request->crd, request->ucrd_no, request->mechinfo);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get robot position: %s", e.what());
  }
}

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the robot's current tool data.
 *
 * @details
 * This service retrieves the current tool data used by the robot.
 * 🔗 API Reference:
 * [Get Current Tool
 * Data](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/3-cur_tool_data)
 *
 */
void ServiceManager::HandleGetRobotCurTool(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetRobotCurTool();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get robot current tool: %s", e.what());
  }
}

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the list of available tools for the robot.
 *
 * @details
 * This service retrieves all tools available to the robot.
 * 🔗 API Reference:
 * [Get Tools](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/4-tools)
 *
 */
void ServiceManager::HandleGetRobotTools(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetRobotTools();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get robot tools: %s", e.what());
  }
}

/**
 * @param request The request object containing the tool number.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the tools' count.
 *
 * @details
 * This service retrieves the count of tools for the robot.
 * 🔗 API Reference:
 * [Get Tools
 * Count](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/5-tools_t)
 *
 */
void ServiceManager::HandleGetRobotToolsT(
    const std::shared_ptr<hdr_msgs::srv::Number::Request> request,
    std::shared_ptr<hdr_msgs::srv::Number::Response> response) {
  try {
    auto [result, success] = driver_->GetRobotToolsT(request->data);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get robot tool count: %s", e.what());
  }
}
/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the robot's current emergency stop status.
 *
 * @details
 * This service retrieves the current emergency stop state from the robot controller.
 * It sends a GET request to the `/project/robot/emergency_stop` endpoint through the driver.
 * The response contains information about whether the emergency stop is currently active or
 * inactive.
 *
 * Possible response values may include:
 * - Emergency stop status (active/inactive)
 * - Timestamp of last status change
 * - Additional safety-related information
 *
 * This service is critical for safety monitoring and should be used to check the robot's
 * emergency stop state before performing operations. The response is returned as a JSON
 * string in the service response message field.
 *
 * 🔗 API Reference:
 * [Get Emergency Stop
 * Status](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/6-emergency_stop)
 *
 */
void ServiceManager::HandleGetRobotEmergency(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetEmergencyStop();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get Emergency: %s", e.what());
  }
}

/**
 * @param request The request object containing the motor power setting.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to set the robot motor power (on/off).
 *
 * @details
 * This service controls the robot's motor power, turning it on or off as specified in the
 * request. 🔗 API Reference: [Post Motor
 * Power](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/1-motor-on)
 *
 */
void ServiceManager::HandlePostRobotMotorPower(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->PostRobotMotorPower();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to set robot motor power: %s", e.what());
  }
}

/**
 * @param request The request object containing the operation status (start/stop).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to set the robot operation (start/stop).
 *
 * @details
 * This service controls the robot's operation, starting or stopping it as specified in the request.
 * 🔗 API Reference:
 * [Post
 * Start/Stop](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/2-start-stop)
 *
 */
void ServiceManager::HandlePostRobotOperation(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  try {
    auto [relay_result, relay_success] = driver_->RelayExternalStopClear();
    if (!relay_success) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to clear remote stop state: %s",
                   relay_result.dump().c_str());
      response->success = false;
      response->message = relay_result.dump();
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto [result, success] = driver_->PostRobotOperation(request->data);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to set robot operation: %s", e.what());
  }
}

/**
 * @param request The request object containing the tool number.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to set the robot tool number.
 *
 * @details
 * This service sets the tool number for the robot to the specified number.
 * 🔗 API Reference:
 * [Post Tool
 * Number](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/3-tool_no)
 *
 */
void ServiceManager::HandlePostRobotToolNo(
    const std::shared_ptr<hdr_msgs::srv::Number::Request> request,
    std::shared_ptr<hdr_msgs::srv::Number::Response> response) {
  try {
    auto [result, success] = driver_->PostRobotToolNo(request->data);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to set robot tool number: %s", e.what());
  }
}

/**
 * @param request The request object containing the coordinate system number.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to set the robot's coordinate system.
 *
 * @details
 * This service sets the robot's coordinate system based on the provided coordinate system number.
 * 🔗 API Reference:
 * [Post Coordinate
 * System](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/4-crd_sys)
 *
 */
void ServiceManager::HandlePostRobotCrdSys(
    const std::shared_ptr<hdr_msgs::srv::Number::Request> request,
    std::shared_ptr<hdr_msgs::srv::Number::Response> response) {
  try {
    auto [result, success] = driver_->PostRobotCrdSys(request->data);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to set robot coordinate system: %s", e.what());
  }
}

/**
 * @param request Empty request (Trigger service has no fields).
 * @param response The response object containing success status and response message.
 *
 * @brief Handles the request to execute an emergency stop on the robot.
 *
 * @details
 * This service initiates an **actual** emergency stop on the robot. It immediately halts all robot
 * motion for safety reasons. This is a critical stop operation and should only be triggered in real
 * emergency situations.
 * Since this uses `std_srvs::srv::Trigger`, it does not require input parameters.
 * 🔗 API Reference:
 * [Post Emergency
 * Stop](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/5-emergency_stop)
 *
 */
void ServiceManager::HandlePostRobotEmergencyStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->PostRobotEmergencyStop();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute emergency stop: %s", e.what());
  }
}
/**
 * @param request The request object containing test parameters: step number, stop conditions, and
 * category.
 * @param response The response object with success status and server message.
 *
 * @brief Handles the request to simulate an emergency stop on the robot.
 *
 * @details
 * This service sends a test emergency stop command to the robot controller using the provided
 * parameters. This does **not** perform a real emergency stop but is used to test behavior in
 * emergency scenarios.
 * 🔗 API Reference:
 * [Post Emergency Stop
 * Test](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/6-emergency_stop_test)
 *
 */
void ServiceManager::HandlePostRobotEmergencyStopTest(
    const std::shared_ptr<hdr_msgs::srv::Emergency::Request> request,
    std::shared_ptr<hdr_msgs::srv::Emergency::Response> response) {
  try {
    auto [result, success] = driver_->PostRobotEmergencyStopTest(
        request->step_no, request->stop_at, request->stop_at_corner, request->category);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute emergency stop test: %s", e.what());
  }
}
/**
 * @param request Empty request object (Trigger service has no fields).
 * @param response The response object containing success status and the returned JSON message.
 *
 * @brief Handles the request to get the available joint trajectory buffer size.
 *
 * @details
 * This service queries the number of available slots in the controller's internal joint trajectory
 * buffer. The returned JSON includes the field `"val"` representing the available slot count
 * (0–2048). Use this service before sending new trajectory data to ensure there is enough buffer
 * capacity for additional points.
 *
 * Example response:
 * @code{.json}
 * {
 *   "val": 2048
 * }
 * @endcode
 *
 * @note The maximum buffer capacity is 2048 points. If the value is 0, the buffer is full and
 * new trajectories cannot be inserted until motion execution clears space.
 *
 * 🔗 API Reference:
 * [GET
 * joint_traject_buf_avail](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/7-joint_traject_buf_avail)
 */
void ServiceManager::HandleGetJointTrajBuffAvail(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetJointTrajBuffAvail();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get joint trajectory buffer availability: %s",
                 e.what());
  }
}

/**
 * @param request Empty request object (Trigger service has no fields).
 * @param response The response object containing success status and the returned JSON message.
 *
 * @brief Handles the request to initialize (clear) the joint trajectory buffer.
 *
 * @details
 * This service calls the controller API to clear all stored trajectory points from the internal
 * buffer. It should be used:
 * - Before sending the first trajectory when the robot is stopped.
 * - After an error recovery (e.g., motion error or E-stop event) to reset the buffer state.
 *
 * @warning
 * Calling this service **while motion is active** will immediately stop the robot and clear
 * all pending trajectory points, possibly causing E-stop or program interruption.
 *
 * 🔗 API Reference:
 * [POST
 * joint_traject_init](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/7-joint_traject_init)
 */
void ServiceManager::HandlePostInitJointTrajectory(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->PostInitJointTrajectory();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize joint trajectory buffer: %s", e.what());
  }
}
/**
 * @param request The request object containing a ROS2 JointTrajectory message
 *        with joint names and trajectory points.
 * @param response The response object containing success status and the JSON result message.
 *
 * @brief Handles the request to insert joint trajectory points into the controller buffer.
 *
 * @details
 * Converts a ROS2 `trajectory_msgs/JointTrajectory` message into the JSON structure required by
 * the HDR Open API, then sends it to the controller via
 * `POST /project/robot/trajectory/joint_traject_insert_points`.
 *
 * Requirements:
 * - Robot program must be running in **auto mode** (e.g., job with `"wait di1"`).
 * - Each trajectory must contain at least 2 points.
 * - The first point’s `time_from_start` must be `0.0` when starting from a stop state.
 * - The time sequence must be strictly increasing across points.
 *
 * Example request payload:
 * @code{.json}
 * {
 *   "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
 *   "points": {
 *     "point_1": {
 *       "positions": [0, 1.570796, 0, 0, 0, 0],
 *       "time_from_start": 0.0
 *     },
 *     "point_2": {
 *       "positions": [0.05, 1.570796, 0, 0, 0, 0],
 *       "time_from_start": 3.0
 *     }
 *   }
 * }
 * @endcode
 *
 * Usage notes:
 * - Use `HandlePostInitJointTrajectory()` to clear the buffer before the first request.
 * - Use `HandleGetJointTrajBuffAvail()` to check buffer availability before sending.
 * - Continuous trajectories should ensure smooth time/position transitions between segments.
 *
 * Common 403 errors:
 * - **E01554:** Program not running (job not active).
 * - **ERR_TOO_FEW_POINTS (-6):** Fewer than 2 points.
 * - **ERR_POINTS_EXCEED_BUFFER (-8):** Not enough buffer capacity.
 *
 * 🔗 API Reference:
 * [POST
 * joint_traject_insert_points](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/5-trajectory/2-post/2-joint_traject_insert_points)
 */
void ServiceManager::HandlePostInsertJointTrajectoryPoints(
    const std::shared_ptr<hdr_msgs::srv::JointTrajectoryPoints::Request> request,
    std::shared_ptr<hdr_msgs::srv::JointTrajectoryPoints::Response> response) {
  try {
    // Extract the JointTrajectory message from the request
    const auto& traj = request->trajectory;

    // -------------------------------------------------------------
    // Convert ROS2 JointTrajectory message to JSON format
    // required by the REST API endpoint:
    // POST /project/robot/trajectory/joint_traject_insert_points
    // -------------------------------------------------------------
    nlohmann::json j_points;
    for (size_t i = 0; i < traj.points.size(); ++i) {
      const auto& pt = traj.points[i];

      // Convert ROS2 builtin_interfaces/Duration to double (seconds)
      double tfs = static_cast<double>(pt.time_from_start.sec) +
                   static_cast<double>(pt.time_from_start.nanosec) * 1e-9;

      // Each trajectory point must be named as "point_1", "point_2", ...
      std::string key = "point_" + std::to_string(i + 1);

      // Construct a JSON entry for this point
      j_points[key] = {
          {"positions", pt.positions},
          {"time_from_start", tfs},
      };
    }

    // Final JSON payload for the REST request
    nlohmann::json trajectory_json = {
        {"joint_names", traj.joint_names},
        {"points", j_points},
    };

    // -------------------------------------------------------------
    // Call the HDR driver to post trajectory points
    // -------------------------------------------------------------
    auto [result, success] = driver_->PostInsertJointTrajectoryPoints(trajectory_json);

    // Fill service response
    response->success = success;
    response->message = result.dump();

  } catch (const std::exception& e) {
    // Handle any exception and return error message
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to insert joint trajectory points: %s", e.what());
  }
}
