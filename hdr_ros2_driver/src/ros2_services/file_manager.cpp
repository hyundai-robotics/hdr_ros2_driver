#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param request The request object containing the path of the directory.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get the files in a directory.
 *
 * @details
 * This service retrieves a list of files located in the specified directory.
 * 🔗 API Reference:
 * [Get
 * Files](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/1-files)
 *
 */
void ServiceManager::HandleGetFiles(const std::shared_ptr<hdr_msgs::srv::FilePath::Request> request,
                                    std::shared_ptr<hdr_msgs::srv::FilePath::Response> response) {
  try {
    auto [result, success] = driver_->GetFiles(request->path);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get files: %s", e.what());
  }
}

/**
 * @param request The request object containing the path of the file.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get information about a file.
 *
 * @details
 * This service retrieves detailed information about the specified file.
 * 🔗 API Reference:
 * [Get File
 * Info](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/2-file_info)
 *
 */
void ServiceManager::HandleGetFileInfo(
    const std::shared_ptr<hdr_msgs::srv::FilePath::Request> request,
    std::shared_ptr<hdr_msgs::srv::FilePath::Response> response) {
  try {
    auto [result, success] = driver_->GetFileInfo(request->path);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get file info: %s", e.what());
  }
}

/**
 * @param request The request object containing the directory path and flags for file inclusion.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to get a list of files in a directory.
 *
 * @details
 * This service retrieves a list of files in the specified directory.
 * 🔗 API Reference:
 * [Get File
 * List](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/3-file_list)
 *
 */
void ServiceManager::HandleGetFileList(
    const std::shared_ptr<hdr_msgs::srv::FileList::Request> request,
    std::shared_ptr<hdr_msgs::srv::FileList::Response> response) {
  try {
    auto [result, success] =
        driver_->GetFileList(request->path, request->incl_file, request->incl_dir);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to get file list: %s", e.what());
  }
}

/**
 * @param request The request object containing the path of the file.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to check the existence of a file.
 *
 * @details
 * This service checks whether a specific file exists at the provided path.
 * 🔗 API Reference:
 * [Check File
 * Existence](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/4-file_exist)
 *
 */
void ServiceManager::HandleGetFileExist(
    const std::shared_ptr<hdr_msgs::srv::FilePath::Request> request,
    std::shared_ptr<hdr_msgs::srv::FilePath::Response> response) {
  try {
    auto [result, success] = driver_->GetFileExist(request->path);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to check file existence: %s", e.what());
  }
}

/**
 * @param request The request object containing the source and target paths for renaming.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to rename a file.
 *
 * @details
 * This service renames the specified file from one path to another.
 * 🔗 API Reference:
 * [Rename
 * File](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/1-rename_file)
 *
 */
void ServiceManager::HandlePostRenameFile(
    const std::shared_ptr<hdr_msgs::srv::FileRename::Request> request,
    std::shared_ptr<hdr_msgs::srv::FileRename::Response> response) {
  try {
    auto [result, success] = driver_->PostRenameFile(request->pathname_from, request->pathname_to);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to rename file: %s", e.what());
  }
}

/**
 * @param request The request object containing the path for the new directory.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to create a directory.
 *
 * @details
 * This service creates a new directory at the specified path.
 * 🔗 API Reference:
 * [Create
 * Directory](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/2-mkdir)
 *
 */
void ServiceManager::HandlePostMkdir(
    const std::shared_ptr<hdr_msgs::srv::FilePath::Request> request,
    std::shared_ptr<hdr_msgs::srv::FilePath::Response> response) {
  try {
    auto [result, success] = driver_->PostMkdir(request->path);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to create directory: %s", e.what());
  }
}

/**
 * @param request The request object containing the source and target file paths.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to send a file.
 *
 * @details
 * This service sends a file from a source to a target location.
 * 🔗 API Reference:
 * [Send
 * Files](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/3-files)
 *
 */
void ServiceManager::HandlePostFiles(
    const std::shared_ptr<hdr_msgs::srv::FileSend::Request> request,
    std::shared_ptr<hdr_msgs::srv::FileSend::Response> response) {
  try {
    auto [result, success] = driver_->PostFiles(request->target_file, request->source_file);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to send file: %s", e.what());
  }
}

/**
 * @param request The request object containing the path of the file to be deleted.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to delete a file.
 *
 * @details
 * This service deletes the file specified in the request.
 * 🔗 API Reference:
 * [Delete
 * File](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/3-delete/1-files)
 *
 */
void ServiceManager::HandlePostDeleteFile(
    const std::shared_ptr<hdr_msgs::srv::FilePath::Request> request,
    std::shared_ptr<hdr_msgs::srv::FilePath::Response> response) {
  try {
    auto [result, success] = driver_->PostDeleteFile(request->path);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to delete file: %s", e.what());
  }
}