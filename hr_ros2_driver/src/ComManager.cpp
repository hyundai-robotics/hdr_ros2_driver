#include "hr_ros2_driver/ComManager.hpp"

ComManager::ComManager(rclcpp::Node *node, std::shared_ptr<UDPManager> udp_manager)
    : node_(node), udp_manager_(udp_manager), is_goal_canceled_(false)
{
    RCLCPP_INFO(node_->get_logger(), "ComManager initialized");
}

ComManager::~ComManager()
{
    RCLCPP_INFO(node_->get_logger(), "ComManager destroyed");
}

void ComManager::setup()
{
    // Set joint names from the desired joint order
    node_->declare_parameter<std::vector<std::string>>("desired_joint_order",
                                                       {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "conveyor_to_robot_base"});
    desired_joint_order_ = node_->get_parameter("desired_joint_order").as_string_array();

    joint_state_msg_.name = desired_joint_order_;
    joint_state_msg_.position.resize(desired_joint_order_.size(), 0.0);
    joint_state_msg_.velocity.resize(desired_joint_order_.size(), 0.0);
    joint_state_msg_.effort.resize(desired_joint_order_.size(), 0.0);

    // Publisher for joint states
    std::string joint_states_topic;
    node_->declare_parameter<std::string>("joint_states_topic", "joint_states");
    node_->get_parameter("joint_states_topic", joint_states_topic);
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);

    // Corrected: capture 'this' to access class members
    udp_manager_->start_receive([this](const std::string &data)
                                { this->process_received_data(data); });

    std::string action_name;
    node_->declare_parameter<std::string>("action_name", "follow_joint_trajectory");
    node_->get_parameter("action_name", action_name);
    action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        node_,
        action_name,
        std::bind(&ComManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ComManager::handle_cancel, this, std::placeholders::_1),
        std::bind(&ComManager::handle_accepted, this, std::placeholders::_1));
}

void ComManager::process_received_data(const std::string &data)
{
    // std::cout << "Received data: " << data << std::endl;

    // Remove the square brackets if present
    std::string cleaned_data = data;
    if (!cleaned_data.empty() && cleaned_data.front() == '[')
    {
        cleaned_data.erase(0, 1); // Remove '['
    }
    if (!cleaned_data.empty() && cleaned_data.back() == ']')
    {
        cleaned_data.pop_back(); // Remove ']'
    }

    // Parse the received data (assuming comma-separated values)
    std::vector<double> parsed_positions;
    std::stringstream ss(cleaned_data);
    std::string value;

    while (std::getline(ss, value, ','))
    {
        try
        {
            // Convert from degree to radian
            double position_in_degrees = std::stod(value);
            double position_in_radians = position_in_degrees * M_PI / 180.0;
            parsed_positions.push_back(position_in_radians);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to parse value '%s': %s", value.c_str(), e.what());
            return; // Exit if there's a parsing error
        }
    }

    // Ensure the number of parsed positions matches the desired number of joints
    size_t num_joints = parsed_positions.size();
    if (num_joints >= 6)
    {
        // Populate the message with the received positions
        joint_state_msg_.position = parsed_positions;
        joint_state_msg_.name.assign(desired_joint_order_.begin(), desired_joint_order_.begin() + num_joints);
        joint_state_msg_.velocity.resize(num_joints, 0.0);
        joint_state_msg_.effort.resize(num_joints, 0.0);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Received data has insufficient number of axes: %lu (expected at least 6)", num_joints);
        return; // Skip further processing if the data is incorrect
    }

    static int count = 0;
    auto now = node_->get_clock()->now();
    joint_state_msg_.header.stamp = now;

    joint_state_publisher_->publish(joint_state_msg_);

    if (count % 100 == 0)
    {
        for (size_t i = 0; i < joint_state_msg_.name.size(); i++)
        {
            // 라디안을 각도로 변환 (rad * 180/π)
            double position_deg = joint_state_msg_.position[i] * 180.0 / M_PI;

            RCLCPP_INFO(node_->get_logger(),"%s: pos=%.3f deg",joint_state_msg_.name[i].c_str(),position_deg);
        }
        RCLCPP_INFO(node_->get_logger(), "\n");
    }

    count++;

    // 카운터 오버플로우 방지
    if (count >= 1000000)
    {
        count = 0;
    }
}

rclcpp_action::GoalResponse ComManager::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(), "Received goal request");
    (void)uuid;
    // You can add goal validation logic here
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ComManager::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    // is_goal_canceled_ = true;
    std::string stop_command = "stop";
    udp_manager_->send_data(stop_command);
    udp_manager_->send_data(stop_command);
    udp_manager_->send_data(stop_command);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ComManager::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    std::thread{std::bind(&ComManager::execute, this, goal_handle)}.detach();
}
void ComManager::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

    if (goal_handle->is_canceling())
    {
        RCLCPP_INFO(node_->get_logger(), "Goal canceled");
        goal_handle->canceled(result);
        return;
    }

    // Collect all points and their timing information from trajectory
    struct TrajectoryCommand {
        std::string command;
        rclcpp::Duration time_from_start;
    };
    std::vector<TrajectoryCommand> trajectory_commands;
    
    // Store the start time
    auto trajectory_start_time = node_->now();

    for (const auto &point : goal->trajectory.points)
    {
        // 각 joint name의 인덱스를 저장할 맵 생성
        std::map<std::string, size_t> joint_name_to_index;
        for (size_t i = 0; i < goal->trajectory.joint_names.size(); ++i)
        {
            joint_name_to_index[goal->trajectory.joint_names[i]] = i;
        }


        
        // desired_joint_order_에 따라 위치값 정렬
        std::vector<double> positions_in_degrees;
        for (const auto &desired_joint : desired_joint_order_)
        {
            auto it = joint_name_to_index.find(desired_joint);
            if (it != joint_name_to_index.end())
            {
                size_t index = it->second;
                double position_in_degrees = point.positions[index] * 180.0 / M_PI;
                positions_in_degrees.push_back(position_in_degrees);
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Joint %s not found in trajectory", desired_joint.c_str());
                positions_in_degrees.push_back(0.0);
            }
        }

        // Create position command string
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << "[";
        for (size_t i = 0; i < positions_in_degrees.size(); ++i)
        {
            ss << positions_in_degrees[i];
            if (i < positions_in_degrees.size() - 1)
            {
                ss << ",";
            }
        }
        ss << "]";
        
        // Store command with its timing
        trajectory_commands.push_back({
            ss.str(),
            rclcpp::Duration(point.time_from_start)
        });
        
        // Log information
        double time_secs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    }

    RCLCPP_INFO(node_->get_logger(), "Starting trajectory execution with %zu points", trajectory_commands.size());

    std::string stop_command = "stop";
    udp_manager_->send_data(stop_command);
    RCLCPP_INFO(node_->get_logger(), "1 seconds waiting stop command");
    rclcpp::sleep_for(std::chrono::seconds(1));



    // Execute trajectory points based on their timing
    for (const auto &command : trajectory_commands)
    {
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(node_->get_logger(), "Goal canceled during execution");
            goal_handle->canceled(result);
            return;
        }

        // Calculate the time to wait before sending this point
        auto current_time = node_->now();
        auto time_elapsed = current_time - trajectory_start_time;
        auto time_to_wait = command.time_from_start - time_elapsed;

        // If we're behind schedule, log a warning but continue
        if (time_to_wait.seconds() < 0)
        {
            RCLCPP_WARN(node_->get_logger(), "Behind schedule by %.3f seconds", -time_to_wait.seconds());
        }
        else
        {
            // Convert Duration to nanoseconds for sleep_for
            auto sleep_duration = std::chrono::nanoseconds(time_to_wait.nanoseconds());
            rclcpp::sleep_for(sleep_duration);
        }

        RCLCPP_INFO(node_->get_logger(), "Sending command at %.3f seconds: %s", time_elapsed.seconds(), command.command.c_str());
        udp_manager_->send_data(command.command);
    }

    if (goal_handle->is_canceling())
    {
        goal_handle->canceled(result);
        return;
    }

    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
}