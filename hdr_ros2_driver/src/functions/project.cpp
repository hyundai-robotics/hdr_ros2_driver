#include "hdr_ros2_driver/functions.hpp"
#include <iostream>
#include <nlohmann/json.hpp>

namespace project {

void project_get_rgen(const std::shared_ptr<RequestLib>& client,
                      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, status_code] = client->get("/project/rgen");
    std::cout << "Project RGEN: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void project_get_jobs_info(const std::shared_ptr<RequestLib>& client,
                           const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, status_code] = client->get("/project/jobs_info");
    std::cout << "Project Jobs Info: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void project_post_reload_update_jobs(const std::shared_ptr<RequestLib>& client,
                                     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    nlohmann::json body = nlohmann::json::object();

    auto [posts, status_code] = client->post("/project/reload_updated_jobs", "", body);
    std::cout << "Reload Updated Jobs: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void project_post_delete_job(const std::shared_ptr<RequestLib>& client,
                             const std::shared_ptr<hdr_msgs::srv::FilePath::Request> request,
                             std::shared_ptr<hdr_msgs::srv::FilePath::Response> response)
{
    if (request->path.empty()) {
        response->success = false;
        response->message = "Invalid data type: expected non-empty string.";
        return;
    }

    nlohmann::json body = {{"fname", request->path}};
    auto [posts, status_code] = client->post("/project/jobs/delete_job", "", body);
    std::cout << "Delete Job: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = posts.dump();
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

} // namespace project