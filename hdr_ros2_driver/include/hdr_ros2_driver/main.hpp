// main.hpp
#ifndef HDR_ROS2_DRIVER_HDRDRIVER_HPP
#define HDR_ROS2_DRIVER_HDRDRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include "hdr_ros2_driver/ServiceManager.hpp"
#include "hdr_ros2_driver/UDPManager.hpp"
#include "hdr_ros2_driver/functions.hpp"
#include "hdr_ros2_driver/ComManager.hpp"
#include <thread>

class HDRDriver : public rclcpp::Node
{
public:
    HDRDriver();
    ~HDRDriver();
private:
    std::shared_ptr<ServiceManager> service_manager_;
    std::shared_ptr<UDPManager> udp_manager_;
    std::unique_ptr<ComManager> com_manager_;
};

#endif