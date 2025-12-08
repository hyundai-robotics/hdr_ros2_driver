#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the control operation condition.
 *
 * @details
 * This service queries the controller for its current operational condition (op_cnd).
 * 🔗 API Reference:
 * [Get Control
 * op_cnd](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/1-op_cnd)
 *
 */
void ServiceManager::HandleGetControlOpCnd(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetControlOpCnd();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get control op_cnd: %s", e.what());
  }
}

/**
 * @param request The request object containing I/O type, block number, and signal number.
 * @param response The response object with success status, message, and I/O values.
 *
 * @brief Handles the request to get the control I/O digital input (di) values.
 *
 * @details
 * This service retrieves the control I/O digital input values from the robot controller.
 * 🔗 API Reference:
 * [Get /project/control/ios/dio/di_val |
 * do_val](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/1-get/2-ios-dio)
 *
 */
void ServiceManager::HandleGetControlIosDio(
    const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
    std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response) {
  try {
    auto [result, success] =
        driver_->GetControlIosDio(request->type, request->blk_no, request->sig_no);

    response->success = success;
    response->message = result.value("error", "");
    response->val = result.value("val", 0);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get control ios_dio: %s", e.what());
  }
}

/**
 * @param request Shared pointer to the service request containing:
 *  - type: the I/O type (e.g., "si", "so", etc.)
 *  - sig_no: the signal number to query.
 * @param response Shared pointer to the service response containing:
 *  - success: whether the retrieval succeeded
 *  - message: error or success message
 *  - val: the retrieved signal value
 *
 * @brief Handles the service request to retrieve a signal value from the robot's SIO (Special I/O).
 *
 * @details
 * This function acts as a ROS2 service server callback for the `/get_control_ios_sio` service.
 * It calls the underlying HDR driver to retrieve the value of a specified SIO signal (input or
 * output), and returns the result in the response message.
 * 🔗 API Reference:
 * [GET /project/control/ios/sio/si_val |
 * so_val](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/1-get/3-ios-sio)
 *
 */
void ServiceManager::HandleGetControlIosSio(
    const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
    std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response) {
  try {
    auto [result, success] = driver_->GetControlIosSio(request->type, request->sig_no);
    response->success = success;
    response->message = result.value("error", "");
    response->val = result.value("val", 0);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get control ios_sio: %s", e.what());
  }
}

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the control unit's operational status (ucs_nos).
 *
 * @details
 * This service retrieves the operational status for the control unit.
 * 🔗 API Reference:
 * [Get Control
 * ucs_nos](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/4-ucss-ucs_nos)
 *
 */
void ServiceManager::HandleGetControlUcsNos(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetControlUcsNos();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get control ucs_nos: %s", e.what());
  }
}

/**
 * @param request The request object containing I/O type, block number, signal number, and value.
 * @param response The response object with success status, message, and I/O values.
 *
 * @brief Handles the request to post the control I/O digital output (do) values.
 *
 * @details
 * This service sets the control I/O digital output values in the robot controller.
 * 🔗 API Reference:
 * [Post Control
 * ios_do](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/2-post/2-ios-dio)
 *
 */
void ServiceManager::HandlePostControlIosDio(
    const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
    std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response) {
  std::cout << request->type << request->blk_no << request->sig_no << request->val << std::endl;
  try {
    auto [result, success] =
        driver_->PostControlIosDio(request->type, request->blk_no, request->sig_no, request->val);
    response->success = success;
    response->message = result.value("error", "");
    response->val = result.value("val", 0);
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post control ios_do: %s", e.what());
  }
}

/**
 * @param request The request object containing the new operation condition parameters.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to update the control operation condition.
 *
 * @details
 * This service updates the operation condition in the robot controller.
 * 🔗 API Reference:
 * [Put Control
 * op_cnd](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/3-put/1-op_cnd)
 *
 */
void ServiceManager::HandlePutControlOpCnd(
    const std::shared_ptr<hdr_msgs::srv::OpCnd::Request> request,
    std::shared_ptr<hdr_msgs::srv::OpCnd::Response> response) {
  try {
    auto [result, success] = driver_->PutControlOpCnd(
        request->playback_mode, request->step_goback_max_spd, request->ucrd_num);
    response->success = success;
    response->message = success ? "Successfully updated op_cnd" : "Failed to update op_cnd";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to put control op_cnd: %s", e.what());
  }
}
