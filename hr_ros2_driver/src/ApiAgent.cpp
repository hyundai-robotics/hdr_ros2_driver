#include "hr_ros2_driver/ApiAgent.hpp"

ApiAgent::ApiAgent() : Node("hr_ros2_driver")
{
    RCLCPP_INFO(this->get_logger(), "Initializing ApiAgent node");

    udp_manager_ = std::make_shared<UDPManager>(this);
    udp_manager_->start();

    com_manager_ = std::make_unique<ComManager>(this,udp_manager_);
    com_manager_->setup();

    service_manager_ = std::make_shared<ServiceManager>(this);
    service_manager_->setup_services();

    RCLCPP_INFO(this->get_logger(), "ApiAgent node initialized");
}

ApiAgent::~ApiAgent()
{
    if (udp_manager_) {
        udp_manager_->stop();
    }
}