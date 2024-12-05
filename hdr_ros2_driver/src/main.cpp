#include "hdr_ros2_driver/main.hpp"

HDRDriver::HDRDriver() : Node("hdr_ros2_driver")
{
    RCLCPP_INFO(this->get_logger(), "Initializing HDRDriver node");

    udp_manager_ = std::make_shared<UDPManager>(this);
    udp_manager_->start();

    com_manager_ = std::make_unique<ComManager>(this,udp_manager_);
    com_manager_->setup();

    service_manager_ = std::make_shared<ServiceManager>(this);
    service_manager_->setup_services();

    RCLCPP_INFO(this->get_logger(), "HDRDriver node initialized");
}

HDRDriver::~HDRDriver()
{
    if (udp_manager_) {
        udp_manager_->stop();
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HDRDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}