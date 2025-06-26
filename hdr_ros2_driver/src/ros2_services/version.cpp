#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param response A standard Trigger response with:
 * - success: true if request succeeded
 * - message: stringified version number or error message
 *
 * @brief Handles request to retrieve the HDR OpenAPI version.
 *
 * @details
 * This service calls the driver to obtain the version of the robot's OpenAPI interface.
 * The version is returned as a float (e.g., "1.1") in the `message` field.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/2-version/1-get/1-api_ver
 *
 */
void ServiceManager::HandleGetApiVersion(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto version = driver_->GetApiVersion();
    response->success = true;
    response->message = std::to_string(version);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get API version: %s", e.what());
  }
}

/**
 * @param response A standard Trigger response with:
 * - success: true if request succeeded
 * - message: stringified version number or error message
 *
 * @brief Handles request to retrieve the robot controller's system (com module) version.
 *
 * @details
 * This service retrieves the version of the underlying communication module used by the robot.
 * The version is returned as a float string (e.g., "2.0") in the `message` field.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/2-version/1-get/2-sysver
 *
 */
void ServiceManager::HandleGetSystemVersion(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto version = driver_->GetSysVersion();
    response->success = true;
    response->message = std::to_string(version);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get system version: %s", e.what());
  }
}