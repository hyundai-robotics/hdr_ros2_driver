#include "hdr_ros2_driver/functions.hpp"

namespace control {

void control_op_cnd(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, _] = client->get("/project/control/op_cnd");
    std::cout << "Control op_cnd: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void control_ucs_nos(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, _] = client->get("/project/control/ucss/ucs_nos");
    std::cout << "Control ucs_nos: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void control_ios_di(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
                    std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response)
{
    std::set<std::string> allowed_types = {"di", "dib", "diw", "dil", "dif"};
    int blk_no_min = 0, blk_no_max = 9;
    
    if (allowed_types.find(request->type) == allowed_types.end()) {
        response->success = false;
        response->message = "Invalid request type.";
        return;
    }

    if (request->blk_no < blk_no_min || request->blk_no > blk_no_max) {
        response->success = false;
        response->message = "Block number out of range.";
        return;
    }

    std::string params = "type=" + request->type + 
                        "&blk_no=" + std::to_string(request->blk_no) + 
                        "&sig_no=" + std::to_string(request->sig_no);

    auto [posts, status_code] = client->get("/project/control/ios/dio/di_val", params);
    std::cout << "Control ios_di: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->val = posts.value("val", 0);  // "val" 키가 없으면 기본값 0 반환
    } else {
        response->success = false;
        response->message = "Request failed with status code: " + std::to_string(status_code);
    }
}

void control_ios_do(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
                    std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response)
{
    std::set<std::string> allowed_types = {"do", "dob", "dow", "dol", "dof"};
    int blk_no_min = 0, blk_no_max = 9;
    
    if (allowed_types.find(request->type) == allowed_types.end()) {
        response->success = false;
        response->message = "Invalid request type.";
        return;
    }

    if (request->blk_no < blk_no_min || request->blk_no > blk_no_max) {
        response->success = false;
        response->message = "Block number out of range.";
        return;
    }

    std::string params = "type=" + request->type + 
                        "&blk_no=" + std::to_string(request->blk_no) + 
                        "&sig_no=" + std::to_string(request->sig_no);

    auto [posts, status_code] = client->get("/project/control/ios/dio/do_val", params);
    std::cout << "Control ios_do: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->val = posts.value("val", 0);  // "val" 키가 없으면 기본값 0 반환
    } else {
        response->success = false;
        response->message = "Request failed with status code: " + std::to_string(status_code);
    }
}


void control_ios_si(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
                    std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response)
{
    std::set<std::string> allowed_types = {"si", "sib", "siw", "sil", "sif"};
    
    if (allowed_types.find(request->type) == allowed_types.end()) {
        response->success = false;
        response->message = "Invalid request type.";
        return;
    }

    if (request->sig_no < 0 || request->sig_no > std::numeric_limits<int>::max() ||
        std::floor(request->sig_no) != request->sig_no) {
        response->success = false;
        response->message = "Invalid signal number.";
        return;
    }

    std::string params = "type=" + request->type + 
                        "&sig_no=" + std::to_string(request->sig_no);

    auto [posts, status_code] = client->get("/project/control/ios/sio/si_val", params);
    std::cout << "Control ios_si: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->val = posts.value("val", 0);  // "val" 키가 없으면 기본값 0 반환
    } else {
        response->success = false;
        response->message = "Request failed with status code: " + std::to_string(status_code);
    }
}

void control_ios_so(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
                    std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response)
{
    std::set<std::string> allowed_types = {"so", "sob", "sow", "sol", "sof"};
    
    if (allowed_types.find(request->type) == allowed_types.end()) {
        response->success = false;
        response->message = "Invalid request type.";
        return;
    }

    if (request->sig_no < 0 || request->sig_no > std::numeric_limits<int>::max() ||
        std::floor(request->sig_no) != request->sig_no) {
        response->success = false;
        response->message = "Invalid signal number.";
        return;
    }

    std::string params = "type=" + request->type + 
                        "&sig_no=" + std::to_string(request->sig_no);

    auto [posts, status_code] = client->get("/project/control/ios/sio/so_val", params);
    std::cout << "Control ios_so: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->val = posts.value("val", 0);  // "val" 키가 없으면 기본값 0 반환
    } else {
        response->success = false;
        response->message = "Request failed with status code: " + std::to_string(status_code);
    }
}

void control_post_ios_do(const std::shared_ptr<RequestLib>& client,
                         const std::shared_ptr<hdr_msgs::srv::IoRequest::Request> request,
                         std::shared_ptr<hdr_msgs::srv::IoRequest::Response> response)
{
    std::set<std::string> allowed_types = {"do", "dob", "dow", "dol", "dof"};
    int blk_no_min = 0, blk_no_max = 9;
    
    if (allowed_types.find(request->type) == allowed_types.end()) {
        response->success = false;
        response->message = "Invalid request type.";
        return;
    }

    if (request->blk_no < blk_no_min || request->blk_no > blk_no_max) {
        response->success = false;
        response->message = "Block number out of range.";
        return;
    }

    if (request->sig_no < 0 || request->sig_no > std::numeric_limits<int>::max() ||
        std::floor(request->sig_no) != request->sig_no) {
        response->success = false;
        response->message = "Invalid signal number.";
        return;
    }
    nlohmann::json params = {
        {"type", request->type},
        {"blk_no", request->blk_no},
        {"sig_no", request->sig_no},
        {"val", request->val}
    };

    auto [posts, status_code] = client->post("/project/control/ios/dio/do_val", "", params);
    std::cout << "Control post_ios_do: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

void control_put_op_cnd(const std::shared_ptr<RequestLib>& client,
                        const std::shared_ptr<hdr_msgs::srv::OpCnd::Request> request,
                        std::shared_ptr<hdr_msgs::srv::OpCnd::Response> response)
{
    nlohmann::json body = {
        {"playback_mode", request->playback_mode},
        {"step_goback_max_spd",request->step_goback_max_spd},
        {"ucrd_num", request->ucrd_num}
    };

    auto [_, status_code] = client->put("/project/control/op_cnd", "", body);
    std::cout << "Control put_op_cnd: " << status_code << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = "Unexpected status code: " + std::to_string(status_code);
    }
}

} // namespace control