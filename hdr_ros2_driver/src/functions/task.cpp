#include "hdr_ros2_driver/functions.hpp"

namespace task {

void task_post_cur_prog_cnt(const std::shared_ptr<RequestLib>& client,
                            const std::shared_ptr<hdr_msgs::srv::ProgramCnt::Request> request,
                            std::shared_ptr<hdr_msgs::srv::ProgramCnt::Response> response)
{
    nlohmann::json param = {
        {"pno", request->pno},
        {"sno", request->sno},
        {"fno", request->fno},
        {"ext_sel", request->ext_sel}
    };

    auto [posts, status_code] = client->post("/project/context/tasks[0]/cur_prog_cnt", "", param);

    if (status_code == 200) {
        response->success = true;
        response->message = posts.dump();
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void task_post_set_cur_pc_idx(const std::shared_ptr<RequestLib>& client,
                              const std::shared_ptr<hdr_msgs::srv::Number::Request> request,
                              std::shared_ptr<hdr_msgs::srv::Number::Response> response)
{
    nlohmann::json param = {{"idx", request->data}};
    auto [_, status_code] = client->post("/project/context/tasks[0]/set_cur_pc_idx", "", param);

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void task_post_release_wait(const std::shared_ptr<RequestLib>& client,
                            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [_, status_code] = client->post("/project/context/tasks[0]/release_wait", "", nlohmann::json::object());

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void task_post_reset(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    nlohmann::json body = nlohmann::json::object(); 
    
    auto [posts, status_code] = client->post("/project/context/tasks/reset", "", body);
    std::cout << "task_post_reset: " << posts.dump(4) << std::endl;

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void task_post_reset_t(const std::shared_ptr<RequestLib>& client,
                       const std::shared_ptr<hdr_msgs::srv::Number::Request> request,
                       std::shared_ptr<hdr_msgs::srv::Number::Response> response)
{

    nlohmann::json body = nlohmann::json::object(); 

    auto [_, status_code] = client->post("/project/context/tasks[" + std::to_string(request->data) + "]/reset" , "", body );

    if (status_code == 200) {
        response->success = true;
        response->message = std::to_string(status_code);
    } else {
        response->success = false;
        response->message = std::to_string(status_code);
    }
}

void task_post_assign_var(const std::shared_ptr<RequestLib>& client,
                          const std::shared_ptr<hdr_msgs::srv::ProgramVar::Request> request,
                          std::shared_ptr<hdr_msgs::srv::ProgramVar::Response> response)
{
    std::set<std::string> allowed_types = {"local", "global", ""};

    if (allowed_types.find(request->scope) == allowed_types.end()) {
        response->success = false;
        response->message = "Invalid request type.";
        return;
    }

    nlohmann::json param;
    bool is_json = false;

    try {
        nlohmann::json::parse(request->expr);
        is_json = true;
    } catch (nlohmann::json::parse_error& e) {
        is_json = false;
    }

    if (is_json) {
        param = {
            {"name", request->name},
            {"scope", request->scope},
            {"json", request->expr},
            {"save", request->save}
        };

        auto [posts, status_code] = client->post("/project/context/tasks[0]/assign_var_json", "", param);

        if (status_code == 200) {
            response->success = true;
            response->message = posts.dump();
        } else {
            response->success = false;
            response->message = std::to_string(status_code);
        }
    } else {
        param = {
            {"name", request->name},
            {"scope", request->scope},
            {"expr", request->expr},
            {"save", request->save}
        };

        auto [posts, status_code] = client->post("/project/context/tasks[0]/assign_var_expr", "", param);

        if (status_code == 200) {
            response->success = true;
            response->message = posts.dump();
        } else {
            response->success = false;
            response->message = std::to_string(status_code);
        }
    }
}

void task_post_solve_expr(const std::shared_ptr<RequestLib>& client,
                          const std::shared_ptr<hdr_msgs::srv::ProgramVar::Request> request,
                          std::shared_ptr<hdr_msgs::srv::ProgramVar::Response> response)
{
    std::set<std::string> allowed_scopes = {"local", "global", ""};

    if (allowed_scopes.find(request->scope) == allowed_scopes.end()) {
        response->success = false;
        response->message = "Invalid scope. Allowed values are: local, global, or empty string.";
        return;
    }

    if (request->expr.empty()) {
        response->success = false;
        response->message = "Expression cannot be empty.";
        return;
    }

    nlohmann::json param = {
        {"expr", request->expr},
        {"scope", request->scope}
    };

    try {
        auto [posts, status_code] = client->post("/project/context/tasks[0]/solve_expr", "", param);
        
        std::cout << "Solve expression response: " << posts.dump(4) << std::endl;

        if (status_code == 200) {
            response->success = true;
            response->message = posts.dump();
        } else {
            response->success = false;
            response->message = "Unexpected status code: " + std::to_string(status_code);
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Error occurred: " + std::string(e.what());
    }
}

/// @brief 
/// @param client 
/// @param request 
/// @param response 
void task_post_execute_move(const std::shared_ptr<RequestLib>& client,
                            const std::shared_ptr<hdr_msgs::srv::ExecuteMove::Request> request,
                            std::shared_ptr<hdr_msgs::srv::ExecuteMove::Response> response) {
                                
    // Construct the path_parameter with the task number
    std::string path_parameter = "/project/context/tasks[" + std::to_string(request->task_no) + "]/execute_move";

    // Example of a valid command
    std::string valid_command_example = "Example of a valid command: move SP,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]";

    // Validate the move command
    const std::string& command = request->stmt;

    // Check command start
    if (command.substr(0, 5) != "move ") {
        response->success = false;
        response->message = "Invalid command start: " + command + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    // Find the first comma which separates the move type from the parameters
    size_t first_comma = command.find(',');
    if (first_comma == std::string::npos) {
        response->success = false;
        response->message = "Missing comma after move type: " + command + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    std::string move_type = command.substr(5, first_comma - 5);
    std::string rest = command.substr(first_comma + 1);

    std::vector<std::string> valid_commands = {"P", "L", "C", "SP", "SL", "SC"};
    if (std::find(valid_commands.begin(), valid_commands.end(), move_type) == valid_commands.end()) {
        response->success = false;
        response->message = "Invalid move type: " + move_type + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    size_t params_end = rest.find(' ');
    if (params_end == std::string::npos) {
        response->success = false;
        response->message = "Missing parameter end: " + rest + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    std::string params = rest.substr(0, params_end);
    std::string list_part = rest.substr(params_end + 1);

    std::vector<std::string> param_list;
    size_t start = 0;
    size_t end = params.find(',');
    while (end != std::string::npos) {
        param_list.push_back(params.substr(start, end - start));
        start = end + 1;
        end = params.find(',', start);
    }
    param_list.push_back(params.substr(start));

    if (param_list.size() != 3) {
        response->success = false;
        response->message = "Invalid number of parameters: " + params + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    std::regex speed_pattern(R"(spd=\d+(mm/sec|cm/min|sec|%))");
    if (!std::regex_match(param_list[0], speed_pattern)) {
        response->success = false;
        response->message = "Invalid speed parameter: " + param_list[0] + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    std::regex accu_pattern(R"(accu=[0-7])");
    if (!std::regex_match(param_list[1], accu_pattern)) {
        response->success = false;
        response->message = "Invalid accuracy parameter: " + param_list[1] + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    std::regex tool_pattern(R"(tool=(\d|[12]\d|3[01]))");
    if (!std::regex_match(param_list[2], tool_pattern)) {
        response->success = false;
        response->message = "Invalid tool parameter: " + param_list[2] + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    std::regex list_pattern(R"(\[\d+, \d+, \d+, \d+, \d+, \d+\])");
    if (!std::regex_match(list_part, list_pattern)) {
        response->success = false;
        response->message = "Invalid list parameter: " + list_part + ". " + valid_command_example;
        std::cerr << response->message << std::endl;
        return;
    }

    // Build the JSON body
    nlohmann::json body;
    body["stmt"] = request->stmt;

    std::cout << "JSON request body: " << body.dump() << std::endl;

    try {
        std::cout << "Full request details:" << std::endl;
        std::cout << "URL: " << path_parameter << std::endl;
        std::cout << "Body: " << body.dump() << std::endl;

        std::pair<nlohmann::json, int> result = client->post(path_parameter, "", body);
        nlohmann::json response_json = result.first;
        int status_code = result.second;

        std::cout << "Execute move response status code: " << status_code << std::endl;
        std::cout << "Raw response body: " << response_json.dump() << std::endl;

        if (status_code == 200) {
            response->success = true;
            response->message = "Move command executed successfully. Response: " + response_json.dump();
        } else {
            response->success = false;
            response->message = "Request failed with status code: " + std::to_string(status_code) +
                                ". Response: " + response_json.dump();
        }

        std::cout << "\nFinal Status Code: " << status_code << std::endl;
        std::cout << "Final Response Text: " << response_json.dump() << std::endl;
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Error occurred: " + std::string(e.what());
        std::cerr << response->message << std::endl;
    }
}
}