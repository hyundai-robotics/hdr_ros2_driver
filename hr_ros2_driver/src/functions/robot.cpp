#include "hr_ros2_driver/functions.hpp"

namespace robot {

void robot_motor_state(const std::shared_ptr<RequestLib>& client,
                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, _] = client->get("/project/robot/motor_on_state");
    std::cout << "Robot motor state: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void robot_cur_tool(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, _] = client->get("/project/robot/cur_tool_data");
    std::cout << "Robot current tool: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void robot_tools(const std::shared_ptr<RequestLib>& client,
                 const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, _] = client->get("/project/robot/tools");
    std::cout << "Robot tools: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void robot_tools_t(const std::shared_ptr<RequestLib>& client,
                   const std::shared_ptr<api_msgs::srv::Number::Request> request,
                   std::shared_ptr<api_msgs::srv::Number::Response> response)
{
    auto [posts, status_code] = client->get("/project/robot/tools/t_" + std::to_string(request->data));

    if (status_code == 200) {
        response->success = true;
        response->message = posts.dump();
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void robot_tool_no(const std::shared_ptr<RequestLib>& client,
                   const std::shared_ptr<api_msgs::srv::Number::Request> request,
                   std::shared_ptr<api_msgs::srv::Number::Response> response)
{
    int tool_min = 0, tool_max = 31;

    if (request->data < tool_min || request->data > tool_max) {
        response->success = false;
        response->message = "Invalid tool number: out of range.";
        return;
    }

    nlohmann::json param = {{"val", request->data}};
    auto [posts, status_code] = client->post("/project/robot/tool_no", "", param);
    std::cout << "Robot tool_no: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void robot_crd_sys(const std::shared_ptr<RequestLib>& client,
                   const std::shared_ptr<api_msgs::srv::Number::Request> request,
                   std::shared_ptr<api_msgs::srv::Number::Response> response)
{
    int crd_min = -1, crd_max = 3;

    if (request->data < crd_min || request->data > crd_max) {
        response->success = false;
        response->message = "Invalid coordinate system number: out of range.";
        return;
    }

    nlohmann::json param = {{"val", request->data}};
    auto [posts, status_code] = client->post("/project/robot/crd_sys", "", param);
    std::cout << "Robot crd_sys: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void robot_motor_on_off(const std::shared_ptr<RequestLib>& client,
                        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    std::string endpoint = request->data ? "/project/robot/motor_on" : "/project/robot/motor_off";
    auto [posts, status_code] = client->post(endpoint, "", nlohmann::json::object());
    std::cout << "Robot motor " << (request->data ? "on" : "off") << ": " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void robot_on_off(const std::shared_ptr<RequestLib>& client,
                  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    std::string endpoint = request->data ? "/project/robot/start" : "/project/robot/stop";
    auto [posts, status_code] = client->post(endpoint, "", nlohmann::json::object());
    std::cout << "Robot " << (request->data ? "start" : "stop") << ": " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void robot_po_cur(const std::shared_ptr<RequestLib>& client,
                  const std::shared_ptr<api_msgs::srv::PoseCur::Request> request,
                  std::shared_ptr<api_msgs::srv::PoseCur::Response> response)
{
    int task_min = 0, task_max = 7;
    int crd_min = -1, crd_max = 3;

    if (request->task_no < task_min || request->task_no > task_max) {
        response->success = false;
        response->message = "Invalid task_no: out of range.";
        return;
    }

    if (request->crd < crd_min || request->crd > crd_max) {
        response->success = false;
        response->message = "Invalid crd: out of range.";
        return;
    }

    nlohmann::json param = {
        {"task_no", request->task_no},
        {"crd", request->crd},
        {"ucrd_no", request->ucrd_no},
        {"mechinfo", request->mechinfo}
    };

    auto [posts, status_code] = client->get("/project/robot/po_cur", param.dump());
    // std::cout << "Robot po_cur: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

std::tuple<nlohmann::json, int> robot_po_cur_timer(const std::shared_ptr<RequestLib>& client, std::vector<int64_t>& pose_param)
{
    int task_min = 0, task_max = 7;
    int crd_min = -1, crd_max = 3;
    int ucrd_no_min = 0, ucrd_no_max = 15;  // 사용자 좌표계 번호 범위 가정
    int mechinfo_min = 0, mechinfo_max = 1;  // mechinfo 범위 가정

    // 파라미터 개수 체크
    if (pose_param.size() != 4) {
        std::cerr << "Invalid number of parameters. Expected 4, got " 
                  << pose_param.size() << std::endl;
        return std::make_tuple("Invalid parameter count", 400);
    }

    // task_no 검증
    if (pose_param[0] < task_min || pose_param[0] > task_max) {
        std::cerr << "Invalid task_no: " << pose_param[0] 
                  << ". Must be between " << task_min 
                  << " and " << task_max << std::endl;
        return std::make_tuple("Invalid task_no", 400);
    }

    // crd 검증
    if (pose_param[1] < crd_min || pose_param[1] > crd_max) {
        std::cerr << "Invalid crd: " << pose_param[1] 
                  << ". Must be between " << crd_min 
                  << " and " << crd_max << std::endl;
        return std::make_tuple("Invalid crd", 400);
    }

    // ucrd_no 검증 (crd가 사용자 좌표계일 때만)
    if (pose_param[1] == 2 && (pose_param[2] < ucrd_no_min || pose_param[2] > ucrd_no_max)) {
        std::cerr << "Invalid ucrd_no: " << pose_param[2] 
                  << ". Must be between " << ucrd_no_min 
                  << " and " << ucrd_no_max << std::endl;
        return std::make_tuple("Invalid ucrd_no", 400);
    }

    // mechinfo 검증
    if (pose_param[3] < mechinfo_min || pose_param[3] > mechinfo_max) {
        std::cerr << "Invalid mechinfo: " << pose_param[3] 
                  << ". Must be between " << mechinfo_min 
                  << " and " << mechinfo_max << std::endl;
        return std::make_tuple("Invalid mechinfo", 400);
    }

    try {
        nlohmann::json param = {
            {"task_no", pose_param[0]},
            {"crd", pose_param[1]},
            {"ucrd_no", pose_param[2]},
            {"mechinfo", pose_param[3]}
        };

        auto [posts, status_code] = client->get("/project/robot/po_cur", param.dump());

        return std::make_tuple(posts, status_code);
    }
    catch (const std::exception& e) {
        std::cerr << "Error processing request: " << e.what() << std::endl;
        return std::make_tuple("Internal server error", 500);
    }
}

void robot_emergency_stop(const std::shared_ptr<RequestLib>& client,
                  const std::shared_ptr<api_msgs::srv::Emergency::Request> request,
                  std::shared_ptr<api_msgs::srv::Emergency::Response> response)
{

    // 유효성 검사
    if (request->step_no < 1 || request->step_no > 999) {
        response->success = false;
        response->message = "Invalid step_no: out of range (1-999).";
        return;
    }
    if (request->stop_at < 1 || request->stop_at > 100) {
        response->success = false;
        response->message = "Invalid stop_at: out of range (1-100).";
        return;
    }
    if (request->stop_at_corner != 0 && request->stop_at_corner != 1) {
        response->success = false;
        response->message = "Invalid stop_at_corner: must be 0 or 1.";
        return;
    }
    if (request->category < 0 || request->category > 2) {
        response->success = false;
        response->message = "Invalid category: must be 0, 1, or 2.";
        return;
    }

    nlohmann::json param = {
        {"step_no", request->step_no},
        {"stop_at", request->stop_at},
        {"stop_at_corner", request->stop_at_corner},
        {"category", request->category}
    };
    // Print the entire JSON object
    std::cout << param << std::endl;
    
    auto [posts, status_code] = client->post("/project/robot/emergency_stop", "", param);
    std::cout << "Robot emergency_stop: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

} // namespace robot