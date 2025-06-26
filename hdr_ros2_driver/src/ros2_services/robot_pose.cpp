#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @brief Publishes the robot's current joint positions.
 *
 * @details
 * Retrieves the current joint positions from the driver and publishes them as a
 * `sensor_msgs::msg::JointState` message to the configured topic. This is periodically triggered by
 * a timer.
 *
 */
void ServiceManager::HandlePublishRobotPose() {
  auto positions = driver_->GetRobotPosition();
  current_position_ = positions;

  if (!positions.empty()) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
    joint_state.header.stamp = node_->now();
    joint_state.position = positions;

    robot_pose_pub_->publish(joint_state);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get robot pose");
  }
}

/**
 * @param uuid UUID of the goal
 * @param goal Shared pointer to the trajectory goal
 *
 * @return rclcpp_action::GoalResponse Always returns ACCEPT_AND_EXECUTE
 *
 * @brief Handles incoming goal requests for robot trajectory execution.
 *
 * @details
 * This method validates incoming trajectory goals. In the current implementation,
 * all valid goals are accepted immediately and executed.
 *
 */
rclcpp_action::GoalResponse ServiceManager::HandleRobotPoseActionGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal>) {
  RCLCPP_INFO(node_->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
/**
 * @param goal_handle Handle to the active goal
 *
 * @return rclcpp_action::CancelResponse Always returns ACCEPT
 *
 * @brief Handles cancellation requests for active trajectory execution goals.
 *
 * @details
 * This function allows for graceful cancellation of the goal execution.
 *
 */
rclcpp_action::CancelResponse ServiceManager::HandleRobotPoseActionCancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>) {
  RCLCPP_INFO(node_->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}
/**
 * @param goal_handle Handle to the accepted goal
 *
 * @brief Called when a new trajectory goal is accepted.
 *
 * @details
 * This method spawns a new thread to execute the trajectory asynchronously.
 *
 */
void ServiceManager::HandleRobotPoseActionAccepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
        goal_handle) {
  std::thread{std::bind(&ServiceManager::HandleRobotPoseActionExecute, this, goal_handle)}.detach();
}

/**
 * @param goal_handle Handle to the goal being executed
 *
 * @brief Executes the robot trajectory goal.
 *
 * @details
 * Iterates through each point in the trajectory and sends joint commands to the driver.
 * Optionally waits for each point to be reached within a small tolerance window.
 * Handles cancellation requests mid-execution and ensures success/failure result is set.
 *
 */
void ServiceManager::HandleRobotPoseActionExecute(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
        goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  const auto& points = goal->trajectory.points;
  if (points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Received empty trajectory");
    goal_handle->abort(result);
    return;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const auto& point = points[i];

    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "Goal canceled");
      goal_handle->canceled(result);
      return;
    }

    std::stringstream ss;
    for (const auto& pos : point.positions)
      ss << pos << " ";
    RCLCPP_INFO(node_->get_logger(), "Sending joint positions: [%s]", ss.str().c_str());

    driver_->SetRobotPosition(point.positions);

    if (i + 1 < points.size()) {
      const auto& next = points[i + 1];

      int64_t sec_diff = static_cast<int64_t>(next.time_from_start.sec) -
                         static_cast<int64_t>(point.time_from_start.sec);
      int64_t nsec_diff = static_cast<int64_t>(next.time_from_start.nanosec) -
                          static_cast<int64_t>(point.time_from_start.nanosec);
      int64_t total_nsec = sec_diff * 1'000'000'000LL + nsec_diff;

      if (total_nsec > 0)
        rclcpp::sleep_for(std::chrono::nanoseconds(total_nsec));
    }
  }

  goal_handle->succeed(result);
  RCLCPP_INFO(node_->get_logger(), "Goal completed successfully");
}