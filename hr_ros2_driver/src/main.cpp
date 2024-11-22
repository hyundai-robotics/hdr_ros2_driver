#include <rclcpp/rclcpp.hpp>
#include "hr_ros2_driver/ApiAgent.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApiAgent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}