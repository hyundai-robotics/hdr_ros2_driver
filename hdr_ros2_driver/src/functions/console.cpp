#include "hdr_ros2_driver/functions.hpp"

namespace console {

int post_execute_cmd(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<hdr_msgs::srv::ExecuteCmd::Request> request,
                     std::shared_ptr<hdr_msgs::srv::ExecuteCmd::Response> response) {
    // Extract commands from the request
    std::vector<std::string> commands = request->cmd_line;

    // Define valid fixed commands
    std::unordered_set<std::string> validCommands = {"rl.stop", "rl.reinit", "rl.i end", "rl.start", "rl.exit"};

    // Define regex patterns for various parameters
    std::regex move_pattern(R"(rl\.i\s+move\s+(P|L|C|SP|SL|SC),spd=\d+(?:\.\d+)?(mm/sec|cm/min|sec|%),accu=[0-7],tool=(\d|[12]\d|3[01])\s+\[.*\])");
    std::regex speed_pattern(R"(spd=\d+(?:\.\d+)?(mm/sec|cm/min|sec|%))");
    std::regex accu_pattern(R"(accu=[0-7])");
    std::regex tool_pattern(R"(tool=(\d|[12]\d|3[01]))");
    std::regex list_pattern(R"(\[\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*\])");

    // Example of a valid command
    std::string valid_command_example = "Example of a valid command: rl.i move P,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]";

    // Iterate through each command
    for (const auto& cmd : commands) {
        if (validCommands.find(cmd) != validCommands.end()) {
            // Valid fixed command
            continue;
        } else if (std::regex_match(cmd, move_pattern)) {
            // The entire command matches the pattern, no need for further parsing
            continue;
        } else {
            // Invalid command
            response->success = false;
            response->message = "Invalid command: " + cmd + ". " + valid_command_example;
            std::cerr << response->message << std::endl;
            return 400;
        }
    }

    // If all commands are valid, send the HTTP POST requests
    for (const auto& cmd : commands) {
        try {
            nlohmann::json body = {{"cmd_line", cmd}};
            auto http_response = client->post("/console/execute_cmd", "", body);
            std::cout << "Execute command response: " << http_response.first.dump(4) << std::endl;

            if (http_response.second != 200) {
                response->success = false;
                response->message = "Unexpected status code: " + std::to_string(http_response.second) + " for command: " + cmd;
                return http_response.second;
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Error occurred: " + std::string(e.what()) + " for command: " + cmd;
            return 500;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // All commands executed successfully
    response->success = true;
    response->message = "All commands executed successfully";
    return 200;
}

}