#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param request  Shared pointer to the incoming service request (command list)
 * @param response Shared pointer to the outgoing service response (execution result)
 *
 * @brief Handler for executing console commands on the HDR controller.
 *
 * @details
 * This service sends a list of command strings to the robot controller using
 * the OpenAPI endpoint for console execution.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/10-console/2-post/1-execute_cmd
 * ROS2 Service Name: ~/console/post/execute_cmd
 *
 */
void ServiceManager::HandlePostExecuteCmd(
    const std::shared_ptr<hdr_msgs::srv::ExecuteCmd::Request> request,
    std::shared_ptr<hdr_msgs::srv::ExecuteCmd::Response> response) {
  try {
    // Send the command list to HDR controller via the driver
    auto [result, success] = driver_->ExecuteCommand(request->cmd_line, request->period_ms);

    // Set response with execution result
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute command: %s", e.what());
  }
}
