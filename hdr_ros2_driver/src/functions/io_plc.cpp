#include "hdr_ros2_driver/functions.hpp"
#include <iostream>
#include <set>
#include <map>
#include <string>
#include <algorithm>
#include <regex>
#include <nlohmann/json.hpp>
#include "hdr_msgs/srv/ioplc_get.hpp"


namespace io_plc {

void get_relay_value(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<hdr_msgs::srv::IoplcGet::Request> request,
                     std::shared_ptr<hdr_msgs::srv::IoplcGet::Response> response)
{
    std::set<std::string> allowed_relay_types = {"di", "do", "x", "y", "m", "s", "r", "k"};
    std::set<std::string> allowed_obj_types = {"fb", "fn"};
    std::map<std::string, std::pair<int, int>> obj_idx_ranges = {
        {"fb", {0, 9}},
        {"fn", {0, 63}}
    };

    std::string relay_type = request->relay_type;
    std::transform(relay_type.begin(), relay_type.end(), relay_type.begin(), ::tolower);
    std::string obj_type = request->obj_type;
    std::transform(obj_type.begin(), obj_type.end(), obj_type.begin(), ::tolower);
    int obj_idx = request->obj_idx;
    int st = request->st;
    int len = request->len;

    // Validate relay_type
    if (allowed_relay_types.find(relay_type) == allowed_relay_types.end()) {
        response->success = false;
        response->message = "Invalid relay_type. Allowed types are: di, do, x, y, m, s, r, k";
        return;
    }

    // Validate obj_type and obj_idx only for specific relay_types
    if (relay_type == "di" || relay_type == "do" || relay_type == "x" || relay_type == "y") {
        if (allowed_obj_types.find(obj_type) == allowed_obj_types.end()) {
            response->success = false;
            response->message = "Invalid obj_type. Allowed types are: fb, fn";
            return;
        }

        if (obj_idx_ranges.find(obj_type) != obj_idx_ranges.end()) {
            auto [min_idx, max_idx] = obj_idx_ranges[obj_type];
            if (obj_idx < min_idx || obj_idx > max_idx) {
                response->success = false;
                response->message = "Invalid obj_idx for " + obj_type + ". Must be between " + 
                                    std::to_string(min_idx) + " and " + std::to_string(max_idx);
                return;
            }
        }
    }

    // Construct URL
    std::string url;
    if (relay_type == "di" || relay_type == "do" || relay_type == "x" || relay_type == "y") {
        url = "/project/plc/" + obj_type + std::to_string(obj_idx) + "_" + relay_type + "/val_s32";
    } else {
        url = "/project/plc/" + relay_type + "/val_s32";
    }

    try {
        nlohmann::json param = {{"st", st}, {"len", len}};
        auto [posts, status_code] = client->get(url, param.dump());
        
        std::cout << "IO PLC get relay value: " << posts.dump(4) << std::endl;

        response->success = true;
        response->message = posts.dump();
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Error occurred: " + std::string(e.what());
    }
}

void post_relay_value(const std::shared_ptr<RequestLib>& client,
                      const std::shared_ptr<hdr_msgs::srv::IoplcPost::Request> request,
                      std::shared_ptr<hdr_msgs::srv::IoplcPost::Response> response)
{
    std::string name = request->name;
    double value = request->value;

    // Validate the name format
    std::regex pattern(R"(^fb\d+\.(dof|di|do|dib|dob|x|xb)\d+$)");
    if (!std::regex_match(name, pattern)) {
        std::cout << "Invalid name format: " << name << std::endl;
        response->success = false;
        response->message = "Invalid name format";
        return;
    }

    // Create the payload
    nlohmann::json payload = {
        {"name", name},
        {"value", value}
    };

    // Post the request to the specified endpoint
    try {
        auto [posts, status_code] = client->post("/project/plc/set_relay_value", "", payload);
        if (status_code == 200) {
            std::cout << "Relay value set successfully." << std::endl;
            response->success = true;
            response->message = "Relay value set successfully";
        } else {
            std::cout << "Failed to set relay value. Status code: " << status_code << std::endl;
            response->success = false;
            response->message = "Failed to set relay value. Status code: " + std::to_string(status_code);
        }
    } catch (const std::exception& e) {
        std::cout << "An error occurred: " << e.what() << std::endl;
        response->success = false;
        response->message = "An error occurred: " + std::string(e.what());
    }
}

} // namespace io_plc