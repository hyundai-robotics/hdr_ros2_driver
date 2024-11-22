#include "hr_ros2_driver/functions.hpp"
#include <fstream>
#include <filesystem>

namespace file_manager {

void file_get_files(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                    std::shared_ptr<api_msgs::srv::FilePath::Response> response)
{
    if (request->path.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected string.";
        return;
    }

    std::string params = "pathname=" + request->path;
    auto [posts, status_code] = client->get_str("/file_manager/files", params);
    std::cout << "File get files: " << posts << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = posts;
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void file_get_info(const std::shared_ptr<RequestLib>& client,
                   const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                   std::shared_ptr<api_msgs::srv::FilePath::Response> response)
{
    if (request->path.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected string.";
        return;
    }

    std::string params = "pathname=" + request->path;
    auto [posts, status_code] = client->get("/file_manager/file_info", params);
    std::cout << "File get info: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = posts.dump();
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void file_get_exist(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                    std::shared_ptr<api_msgs::srv::FilePath::Response> response)
{
    if (request->path.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected string.";
        return;
    }

    std::string params = "pathname=" + request->path;
    auto [posts, status_code] = client->get("/file_manager/file_exist", params);
    std::cout << "File get exist: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = posts.dump();
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void file_post_mkdir(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                     std::shared_ptr<api_msgs::srv::FilePath::Response> response)
{
    if (request->path.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected string.";
        return;
    }

    nlohmann::json body = {{"path", request->path}};
    auto [posts, status_code] = client->post("/file_manager/mkdir", "", body);
    std::cout << "File post mkdir: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = posts.dump();
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void file_post_delete(const std::shared_ptr<RequestLib>& client,
                      const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                      std::shared_ptr<api_msgs::srv::FilePath::Response> response)
{
    if (request->path.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected string.";
        return;
    }

    auto [posts, status_code] = client->del("/file_manager/files/" + request->path);
    std::cout << "File post delete: " << status_code << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = status_code;
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void file_get_list(const std::shared_ptr<RequestLib>& client,
                   const std::shared_ptr<api_msgs::srv::FileList::Request> request,
                   std::shared_ptr<api_msgs::srv::FileList::Response> response)
{
    if (request->path.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected non-empty string.";
        return;
    }

    std::string params = "path=" + request->path + 
                         "&incl_file=" + std::string(request->incl_file ? "true" : "false") +
                         "&incl_dir=" + std::string(request->incl_dir ? "true" : "false");         

    auto [posts, status_code] = client->get("/file_manager/file_list", params);
    std::cout << "File get list: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = posts.dump();
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void file_post_rename_file(const std::shared_ptr<RequestLib>& client,
                           const std::shared_ptr<api_msgs::srv::FileRename::Request> request,
                           std::shared_ptr<api_msgs::srv::FileRename::Response> response)
{
    if (request->pathname_from.empty() || request->pathname_to.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected non-empty strings.";
        return;
    }

    nlohmann::json param = {
        {"pathname_from", request->pathname_from},
        {"pathname_to", request->pathname_to}
    };

    auto [_, status_code] = client->post("/file_manager/rename_file", "", param);
    std::cout << "File post rename: " << status_code << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = "Expected status code: " + std::to_string(status_code);
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void file_post_files(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<api_msgs::srv::FileSend::Request> request,
                     std::shared_ptr<api_msgs::srv::FileSend::Response> response)
{
    if (request->target_file.empty() || request->source_file.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected non-empty strings.";
        return;
    }

    if (!std::filesystem::exists(request->source_file)) {
        response->success = false;
        response->message = "Invalid path: source_file does not exist.";
        return;
    }

    std::ifstream file(request->source_file, std::ios::binary);
    if (!file) {
        response->success = false;
        response->message = "Failed to open source file.";
        return;
    }

    std::string file_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    auto [posts, status_code] = client->post_file("/file_manager/files" + request->target_file, "", file_content);
    std::cout << "File post files: " << status_code << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

} // namespace file_manage