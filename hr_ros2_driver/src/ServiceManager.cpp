
// ServiceManager.cpp
#include "hr_ros2_driver/ServiceManager.hpp"

ServiceManager::ServiceManager(rclcpp::Node* node)
    : node_(node), io_context_()
{
    io_threads_.emplace_back([this]() { io_context_.run(); });
}

ServiceManager::~ServiceManager()
{
    io_context_.stop();
    for (auto& thread : io_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

template <typename T>
void ServiceManager::setup_service(const std::string &name, ServiceCallback<T> callback)
{
    auto service_callback = [this, callback](
                                const std::shared_ptr<typename T::Request> request,
                                std::shared_ptr<typename T::Response> response)
    {
        callback(client_, request, response);
    };

    services.push_back(node_->create_service<T>(name, service_callback));
}

// Function to set up all services
void ServiceManager::setup_services()
{
    services_get_parameter();
    publisher_set();
    
    // Version related services
    setup_service<std_srvs::srv::Trigger>("/api_agent/get/api_ver", version::api_version);
    setup_service<std_srvs::srv::Trigger>("/api_agent/get/sysver", version::sysver_version);

    // Project related services
    setup_service<std_srvs::srv::Trigger>("/api_agent/project/get/rgen", project::project_get_rgen);
    setup_service<std_srvs::srv::Trigger>("/api_agent/project/get/jobs_info", project::project_get_jobs_info);
    setup_service<std_srvs::srv::Trigger>("/api_agent/project/post/reload_updated_jobs", project::project_post_reload_update_jobs);
    setup_service<api_msgs::srv::FilePath>("/api_agent/project/post/delete_job", project::project_post_delete_job);

    // Control related services
    setup_service<std_srvs::srv::Trigger>("/api_agent/control/get/op_cnd", control::control_op_cnd);
    setup_service<std_srvs::srv::Trigger>("/api_agent/control/get/ucs_nos", control::control_ucs_nos);
    setup_service<api_msgs::srv::IoRequest>("/api_agent/control/get/ios_di", control::control_ios_di);
    setup_service<api_msgs::srv::IoRequest>("/api_agent/control/get/ios_do", control::control_ios_do);
    setup_service<api_msgs::srv::IoRequest>("/api_agent/control/get/ios_si", control::control_ios_si);
    setup_service<api_msgs::srv::IoRequest>("/api_agent/control/get/ios_so", control::control_ios_so);
    setup_service<api_msgs::srv::IoRequest>("/api_agent/control/post/ios_do", control::control_post_ios_do);
    setup_service<api_msgs::srv::OpCnd>("/api_agent/control/put/op_cnd", control::control_put_op_cnd);

    // Robot related services
    setup_service<std_srvs::srv::Trigger>("/api_agent/robot/get/motor_state", robot::robot_motor_state);
    setup_service<std_srvs::srv::Trigger>("/api_agent/robot/get/cur_tool_data", robot::robot_cur_tool);
    setup_service<std_srvs::srv::Trigger>("/api_agent/robot/get/tools", robot::robot_tools);
    setup_service<api_msgs::srv::Number>("/api_agent/robot/get/tools_t", robot::robot_tools_t);
    setup_service<api_msgs::srv::Number>("/api_agent/robot/post/tool_no", robot::robot_tool_no);
    setup_service<api_msgs::srv::Number>("/api_agent/robot/post/crd_sys", robot::robot_crd_sys);
    setup_service<std_srvs::srv::SetBool>("/api_agent/robot/post/motor_control", robot::robot_motor_on_off);
    setup_service<std_srvs::srv::SetBool>("/api_agent/robot/post/robot_control", robot::robot_on_off);
    setup_service<api_msgs::srv::PoseCur>("/api_agent/robot/get/po_cur", robot::robot_po_cur);
    setup_service<api_msgs::srv::Emergency>("/api_agent/robot/post/emergency_stop", robot::robot_emergency_stop);

    // Clock related services
    setup_service<std_srvs::srv::Trigger>("/api_agent/clock/get/date_time", etc::etc_date_time);
    setup_service<api_msgs::srv::DateTime>("/api_agent/clock/put/date_time", etc::etc_put_date_time);

    // Task related services
    setup_service<std_srvs::srv::Trigger>("/api_agent/task/post/release_wait", task::task_post_release_wait);
    setup_service<std_srvs::srv::Trigger>("/api_agent/task/post/reset", task::task_post_reset);
    setup_service<api_msgs::srv::Number>("/api_agent/task/post/set_cur_pc_idx", task::task_post_set_cur_pc_idx);
    setup_service<api_msgs::srv::Number>("/api_agent/task/post/reset_t", task::task_post_reset_t);
    setup_service<api_msgs::srv::ProgramCnt>("/api_agent/task/post/cur_prog_cnt", task::task_post_cur_prog_cnt);
    setup_service<api_msgs::srv::ProgramVar>("/api_agent/task/post/assign_var", task::task_post_assign_var);
    setup_service<api_msgs::srv::ProgramVar>("/api_agent/task/post/solve_expr", task::task_post_solve_expr);
    setup_service<api_msgs::srv::ExecuteMove>("/api_agent/task/post/execute_move", task::task_post_execute_move);
    
    // File manager related services
    setup_service<api_msgs::srv::FilePath>("/api_agent/file/get/files", file_manager::file_get_files);
    setup_service<api_msgs::srv::FilePath>("/api_agent/file/get/file_info", file_manager::file_get_info);
    setup_service<api_msgs::srv::FilePath>("/api_agent/file/get/file_exist", file_manager::file_get_exist);
    setup_service<api_msgs::srv::FilePath>("/api_agent/file/post/mkdir", file_manager::file_post_mkdir);
    setup_service<api_msgs::srv::FilePath>("/api_agent/file/delete/files", file_manager::file_post_delete);
    setup_service<api_msgs::srv::FileList>("/api_agent/file/get/file_list", file_manager::file_get_list);
    setup_service<api_msgs::srv::FileRename>("/api_agent/file/post/rename_file", file_manager::file_post_rename_file);
    setup_service<api_msgs::srv::FileSend>("/api_agent/file/post/files", file_manager::file_post_files);

    // Log manager service
    setup_service<api_msgs::srv::LogManager>("/api_agent/log/get/manager", etc::etc_get_log_manager);

    // I/O PLC services
    setup_service<api_msgs::srv::IoplcGet>("/api_agent/plc/get/relay_value", io_plc::get_relay_value);
    setup_service<api_msgs::srv::IoplcPost>("/api_agent/plc/post/relay_value", io_plc::post_relay_value);

    // Console services
    setup_service<api_msgs::srv::ExecuteCmd>("/api_agent/console/post/execute_cmd", console::post_execute_cmd);

    RCLCPP_INFO(node_->get_logger(), "All services have been set up");
}


void ServiceManager::services_get_parameter()
{
    // Declare and get string parameters as before
    node_->declare_parameter<std::string>("openapi_ip", "192.168.1.150");
    node_->declare_parameter<int>("openapi_port", 8888);

    std::string openapi_ip = node_->get_parameter("openapi_ip").as_string();
    int openapi_port = node_->get_parameter("openapi_port").as_int();
    RCLCPP_INFO(node_->get_logger(), "Server IP: %s", openapi_ip.c_str());
    RCLCPP_INFO(node_->get_logger(), "Server Port: %d", openapi_port);

    std::string open_api_url = "http://" + openapi_ip + ":" + std::to_string(openapi_port);
    RCLCPP_INFO(node_->get_logger(), "OPEN API URL: %s", open_api_url.c_str());

    client_ = std::make_shared<RequestLib>(open_api_url);
}

void ServiceManager::publisher_set()
{
    // Declare and initialize motor_pos_on_off parameter
    node_->declare_parameter<bool>("motor_pose_state", false);
    bool motor_pose_state = node_->get_parameter("motor_pose_state").as_bool();
    
    // Set up motor_pos_interval_ms parameter
    node_->declare_parameter<int>("motor_pose_interval_ms", 100);
    motor_pos_interval_ms_ = node_->get_parameter("motor_pose_interval_ms").as_int();

    node_->declare_parameter<std::string>("motor_pose_topic","/api_agent/joint_position");
    std::string motor_pose_topic_ = node_->get_parameter("motor_pose_topic").as_string();
    

    node_->declare_parameter<std::vector<int64_t>>("pose_param", {0, -1, 0, 0});
    node_->get_parameter_or("pose_param", pose_param, std::vector<int64_t>{0, -1, 0, 0});

    
    desired_joint_order_ = node_->get_parameter("desired_joint_order").as_string_array();
    
    if(motor_pose_state){
        robot_pos_timer_ = node_->create_wall_timer(std::chrono::milliseconds(motor_pos_interval_ms_),
                                                std::bind(&ServiceManager::robot_pos_timer_callback, this));
        joint_pose_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(motor_pose_topic_, 1);
    }
}

// Callback function for motor position timer
void ServiceManager::robot_pos_timer_callback()
{
    auto current_interval_ms = node_->get_parameter("motor_pose_interval_ms").as_int();

    // Detect parameter value change and update timer period
    if (current_interval_ms != motor_pos_interval_ms_)
    {
        motor_pos_interval_ms_ = current_interval_ms;
        robot_pos_timer_->cancel();
        robot_pos_timer_ = node_->create_wall_timer(std::chrono::milliseconds(motor_pos_interval_ms_),
                                                   std::bind(&ServiceManager::robot_pos_timer_callback, this));
        RCLCPP_INFO(node_->get_logger(), "Update interval changed to: %d milliseconds", motor_pos_interval_ms_);
    }
    
    std::tuple<nlohmann::json, int> pose_data = robot::robot_po_cur_timer(client_, pose_param);

    int status_code = std::get<1>(pose_data);
    nlohmann::json json_data = std::get<0>(pose_data);

    if (status_code == 200) {
        try {
            // J로 시작하는 값들을 저장할 벡터  
            std::vector<std::pair<std::string, double>> joint_values;
            
            // 직접 객체에서 j로 시작하는 키를 찾음
            for (const auto& [key, value] : json_data.items()) {
                if (key[0] == 'j' && key.length() > 1 && std::isdigit(key[1])) {
                    // degree를 radian으로 변환 (degree * π/180)
                    double radian = value.get<double>() * M_PI / 180.0;
                    joint_values.push_back({key, radian});
                }
            }
            
            // 키 값으로 정렬 (j1, j2, j3 순서로)
            std::sort(joint_values.begin(), joint_values.end(),
                [](const auto& a, const auto& b) {
                    return std::stoi(a.first.substr(1)) < std::stoi(b.first.substr(1));
                });
            
            sensor_msgs::msg::JointState joint_state_msg_;
            joint_state_msg_.name = desired_joint_order_;
            joint_state_msg_.position.resize(desired_joint_order_.size(), 0.0);
            joint_state_msg_.velocity.resize(desired_joint_order_.size(), 0.0);
            joint_state_msg_.effort.resize(desired_joint_order_.size(), 0.0);

            // joint_values의 값들을 joint_state_msg_의 position에 할당
            for (size_t i = 0; i < joint_values.size() && i < desired_joint_order_.size(); ++i) {
                joint_state_msg_.position[i] = joint_values[i].second;
            }

            // 현재 시간 설정
            joint_state_msg_.header.stamp = node_->now();

            // 메시지 발행
            joint_pose_publisher_->publish(joint_state_msg_);

        } catch (const nlohmann::json::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
        }
    }

}