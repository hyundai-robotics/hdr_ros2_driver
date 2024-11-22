// ApiAgent.hpp
#ifndef HR_ROS2_DRIVER_APIAGENT_HPP
#define HR_ROS2_DRIVER_APIAGENT_HPP

#include <rclcpp/rclcpp.hpp>
#include "hr_ros2_driver/ServiceManager.hpp"
#include "hr_ros2_driver/UDPManager.hpp"
#include "hr_ros2_driver/functions.hpp"
#include "hr_ros2_driver/ComManager.hpp"
#include <thread>

class ApiAgent : public rclcpp::Node
{
public:
    ApiAgent();
    ~ApiAgent();
private:
    std::shared_ptr<ServiceManager> service_manager_;
    std::shared_ptr<UDPManager> udp_manager_;
    std::unique_ptr<ComManager> com_manager_;
};

#endif