#include "hdr_ros2_driver/hdr_ros2_node.hpp"

/**
 * @brief ROS2 wrapper node for HDR client driver.
 * This node initializes the HDR client using ROS2 parameters and exposes
 * HDR APIs via ROS2 services through a `ServiceManager`.
 * @file hdr_ros2_driver.cpp
 *
 */

/**
 * @throws std::exception Rethrows any exceptions that occur during initialization
 *
 * @brief Constructor for HDRROS2Node class
 * Initializes the ROS2 node, declares parameters, creates the HDR driver,
 * and sets up the service manager.
 * Parameters declared:
 * - `openapi_ip` (string): IP address of the robot controller (default: `"192.168.1.150"`)
 * - `openapi_port` (int): Port number for OpenAPI server (default: `8888`)
 *
 */
HDRROS2Node::HDRROS2Node() : Node("hdr_ros2_driver") {
  RCLCPP_INFO(this->get_logger(), "Initializing ROS2 Node");

  // Declare all required ROS2 parameters with default values
  this->declare_parameter("openapi_ip", "192.168.1.150");  // Default robot IP address
  this->declare_parameter("openapi_port", 8888);           // Default OpenAPI HTTP port

  try {
    // Initialize HDR driver with parameters from ROS2 parameter server
    driver_ = std::make_unique<hdrcl::HdrDriver>(this->get_parameter("openapi_ip").as_string(),
                                                 this->get_parameter("openapi_port").as_int());

    // Create service manager and set up all ROS2 services
    service_manager_ = std::make_unique<ServiceManager>(this, driver_.get());
    service_manager_->SetupAllServices();

    RCLCPP_INFO(this->get_logger(), "Driver initialized successfully");
  } catch (const std::exception& e) {
    // Log error and rethrow to signal initialization failure
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize driver: %s", e.what());
    throw;
  }
}

/**
 * @param argc Argument count
 * @param argv Argument vector
 *
 * @return int Exit status code
 *
 * @brief Main function that launches the HDR ROS2 node.
 * Initializes ROS2, creates an HDRROS2Node instance, and spins the node.
 * Shuts down ROS upon completion.
 *
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HDRROS2Node>();

  // Create a multi-threaded executor with default thread count (number of cores)
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
