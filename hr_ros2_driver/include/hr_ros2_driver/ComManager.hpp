// JointTrajectoryServer.hpp
#ifndef JOINT_TRAJECTORY_SERVER_HPP
#define JOINT_TRAJECTORY_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "hr_ros2_driver/UDPManager.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath> // For M_PI

class ComManager
{
public:
    ComManager(rclcpp::Node* node, std::shared_ptr<UDPManager> udp_manager);
    ~ComManager();

    void setup();
private:
    rclcpp::Node* node_;
    void process_received_data(const std::string &data);

    //Managers
    std::shared_ptr<UDPManager> udp_manager_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    sensor_msgs::msg::JointState joint_state_msg_;

    std::vector<std::string> desired_joint_order_;
    bool is_goal_canceled_;

    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
};

#endif // JOINT_TRAJECTORY_SERVER_HPP