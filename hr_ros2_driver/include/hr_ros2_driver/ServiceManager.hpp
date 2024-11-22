#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "hr_ros2_driver/Requestlib.hpp"
#include "hr_ros2_driver/functions.hpp"

class ServiceManager {
public:
    explicit ServiceManager(rclcpp::Node* node);
    ~ServiceManager();

    void setup_services();
    std::shared_ptr<RequestLib> get_client() const { return client_; }

private:
    // Type definitions
    template<typename T>
    using ServiceCallback = std::function<void(
        const std::shared_ptr<RequestLib>&,
        const std::shared_ptr<typename T::Request>,
        std::shared_ptr<typename T::Response>
    )>;

    // Service setup methods
    void services_get_parameter();
    template<typename T>
    void setup_service(const std::string& name, ServiceCallback<T> callback);

    // Publisher methods
    void publisher_set();
    void robot_pos_timer_callback();

    // Core components
    rclcpp::Node* node_;
    std::shared_ptr<RequestLib> client_;
    boost::asio::io_context io_context_;
    
    // Service and thread management
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::vector<std::thread> io_threads_;

    // Publishers and timers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pose_publisher_;
    rclcpp::TimerBase::SharedPtr robot_pos_timer_;

    // Configuration parameters
    std::vector<std::string> desired_joint_order_;
    std::vector<int64_t> pose_param;
    int motor_pos_interval_ms_;
};