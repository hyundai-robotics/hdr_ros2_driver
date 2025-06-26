#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @brief Handles the retrieval of the current date and time from the controller.
 *
 * @details
 * This service queries the controller for its current clock settings.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/11-etc/1-clock/1-get/1-date_time
 *
 */
void ServiceManager::HandleGetDateTime(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->GetDateTime();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get date time: %s", e.what());
  }
}

/**
 * @param request  Contains year, month, day, hour, minute, and second values.
 * @param response Response object with success flag and message.
 *
 * @brief Handles the setting of the date and time on the controller.
 *
 * @details
 * This service updates the controller's internal clock to the specified time.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/11-etc/1-clock/2-put/1-date_time
 *
 */
void ServiceManager::HandlePutDateTime(
    const std::shared_ptr<hdr_msgs::srv::DateTime::Request> request,
    std::shared_ptr<hdr_msgs::srv::DateTime::Response> response) {
  try {
    auto [result, success] = driver_->PutDateTime(request->year, request->mon, request->day,
                                                  request->hour, request->min, request->sec);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to put date time: %s", e.what());
  }
}

/**
 * @param request  Log filter parameters: number of items, category, ID range, time range.
 * @param response Contains the JSON log result and success status.
 *
 * @brief Handles retrieval of system logs from the robot controller.
 *
 * @details
 * This service allows filtered log queries based on category, ID, and time range.
 * 🔗 API Reference:
 * https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/7-log_manager/1-get/1-search
 *
 */
void ServiceManager::HandleGetLogManager(
    const std::shared_ptr<hdr_msgs::srv::LogManager::Request> request,
    std::shared_ptr<hdr_msgs::srv::LogManager::Response> response) {
  try {
    // Validate required parameters
    if (request->n_item <= 0) {
      response->success = false;
      response->message = "{\"error\": \"Invalid n_item value: must be positive\"}";
      return;
    }

    if (request->cat_p.empty()) {
      response->success = false;
      response->message = "{\"error\": \"cat_p is required and cannot be empty\"}";
      return;
    }

    // Convert empty optional values to std::nullopt
    std::optional<uint64_t> id_min = std::nullopt;
    std::optional<uint64_t> id_max = std::nullopt;
    std::optional<std::string> ts_min = std::nullopt;
    std::optional<std::string> ts_max = std::nullopt;

    // Only set values if they are valid
    if (request->id_min > 0) {
      id_min = request->id_min;
    }

    if (request->id_max > 0) {
      id_max = request->id_max;
    }

    if (!request->ts_min.empty()) {
      ts_min = request->ts_min;
    }

    if (!request->ts_max.empty()) {
      ts_max = request->ts_max;
    }

    // Call the driver function with properly handled optional parameters
    auto [result, success] =
        driver_->GetLogManager(request->n_item, request->cat_p, id_min, id_max, ts_min, ts_max);

    response->success = success;
    response->message = result.dump();

    RCLCPP_DEBUG(node_->get_logger(), "Log manager query completed with success=%s",
                 success ? "true" : "false");
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "{\"error\": \"" + std::string(e.what()) + "\"}";
    RCLCPP_ERROR(node_->get_logger(), "Failed to get log manager: %s", e.what());
  }
}