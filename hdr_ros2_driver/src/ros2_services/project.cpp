#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param request The request object (empty for this service).
 * @param response The response object containing success status and message (RGEN result).
 *
 * @brief Handles the request to retrieve the project RGEN.
 *
 * @details
 * This service fetches the current RGEN (robot generation) for the project.
 * It queries the robot controller for its generation number.
 * 🔗 API Reference:
 * - [Get Project
 * RGEN](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/1-get/1-rgen)
 *
 */
void ServiceManager::HandleGetProjectRgen(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    // Fetch project RGEN from the driver
    auto [result, success] = driver_->GetProjectRgen();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    // Handle exception and log error
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get project RGEN: %s", e.what());
  }
}

/**
 * @param request The request object (empty for this service).
 * @param response The response object containing success status and job information.
 *
 * @brief Handles the request to retrieve project jobs information.
 *
 * @details
 * This service retrieves the details about the jobs associated with the project.
 * It queries the robot controller for its job data.
 * 🔗 API Reference:
 * - [Get Project Jobs
 * Info](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/1-get/2-jobs_info)
 *
 */
void ServiceManager::HandleGetProjectJobsInfo(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    // Fetch project jobs info from the driver
    auto [result, success] = driver_->GetProjectJobsInfo();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    // Handle exception and log error
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get project jobs info: %s", e.what());
  }
}

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success flag and a message indicating success or
 * failure.
 *
 * @brief Handles the request to reload updated jobs for the project.
 *
 * @details
 * This service reloads the updated jobs for the project. It triggers the reloading process
 * in the robot controller to ensure the jobs are synchronized with the latest configurations.
 * 🔗 API Reference:
 * - [Reload Updated
 * Jobs](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/2-post/1-reload_updated_jobs)
 *
 */
void ServiceManager::HandlePostProjectReloadUpdateJobs(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    // Reload the updated jobs in the driver
    auto [result, success] = driver_->PostProjectReloadUpdateJobs();
    response->success = success;
    response->message =
        success ? "Successfully reloaded updated jobs" : "Failed to reload updated jobs";
  } catch (const std::exception& e) {
    // Handle exception and log error
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to reload updated jobs: %s", e.what());
  }
}

/**
 * @param request The request object containing the path of the job to be deleted.
 * @param response The response object with success flag and message.
 *
 * @brief Handles the request to delete a job from the project.
 *
 * @details
 * This service deletes a specified job from the project based on the provided path.
 * The job is removed from the project, and the changes are applied to the robot controller.
 * 🔗 API Reference:
 * - [Delete
 * Job](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/2-post/2-jobs-delete_job)
 *
 */
void ServiceManager::HandlePostProjectDeleteJob(
    const std::shared_ptr<hdr_msgs::srv::FilePath::Request> request,
    std::shared_ptr<hdr_msgs::srv::FilePath::Response> response) {
  try {
    // Delete the specified job from the project using the driver
    auto [result, success] = driver_->PostProjectDeleteJob(request->path);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    // Handle exception and log error
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to delete job: %s", e.what());
  }
}