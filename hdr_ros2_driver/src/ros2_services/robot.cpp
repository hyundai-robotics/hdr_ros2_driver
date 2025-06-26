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
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  try {
    auto [result, success] = driver_->PostRobotMotorPower(request->data);
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
