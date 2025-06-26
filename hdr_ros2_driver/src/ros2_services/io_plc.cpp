#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param request Request object containing the details such as relay type, object type, index, etc.
 * @param response Response object with the result and success status.
 *
 * @brief Handles request to get the relay value from the PLC.
 *
 * @details
 * This service queries the relay values from the PLC for a specific object and relay type. The
 * parameters like `relay_type`, `obj_type`, `obj_idx`, `st`, and `len` are used to filter the relay
 * data.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/1-get/1-relay-value
 *
 */
void ServiceManager::HandleGetRelayValue(
    const std::shared_ptr<hdr_msgs::srv::IoplcGet::Request> request,
    std::shared_ptr<hdr_msgs::srv::IoplcGet::Response> response) {
  try {
    // Get relay value from the driver based on the request parameters
    auto [result, success] = driver_->GetRelayValue(request->type, request->st, request->len);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    // If an exception occurs, set the response as failed and log the error
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get relay value: %s", e.what());
  }
}

/**
 * @param request Request object containing the relay name and the value to set.
 * @param response Response object with the result and success status.
 *
 * @brief Handles request to set the relay value on the PLC.
 *
 * @details
 * This service sets a specific relay value for a given relay name. The relay name and value
 * are passed in the request, and the system updates the PLC's state accordingly.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/2-post/1-set_relay_value
 *
 */
void ServiceManager::HandlePostRelayValue(
    const std::shared_ptr<hdr_msgs::srv::IoplcPost::Request> request,
    std::shared_ptr<hdr_msgs::srv::IoplcPost::Response> response) {
  try {
    // Set relay value on the driver using the request's parameters
    auto [result, success] = driver_->SetRelayValue(request->name, request->value);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    // If an exception occurs, set the response as failed and log the error
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post relay value: %s", e.what());
  }
}