#ifndef HDR_ROS2_DRIVER_HDR_ROS2_NODE_HPP_
#define HDR_ROS2_DRIVER_HDR_ROS2_NODE_HPP_

#include "hdr_ros2_driver/service_manager.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief ROS2 Node wrapper for HdrDriver and ServiceManager integration.
 * This class initializes:
 * - A `hdrcl::HdrDriver` instance for communication with the HDR OpenAPI system.
 * - A `ServiceManager` to expose HDR API endpoints via ROS2 services.
 * It serves as the main entry point for HDR robot integration with ROS2.
 *
 * @class HDRROS2Node
 */
class HDRROS2Node : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   * Initializes parameters and binds all services through the service manager.
   * Throws exception if HdrDriver fails to initialize.
   *
   */
  HDRROS2Node();

  /**
   * @brief Destructor
   *
   */
  virtual ~HDRROS2Node() = default;

 private:
  // HDR driver instance for HTTP communication
  std::unique_ptr<hdrcl::HdrDriver> driver_;
  // Service manager to handle all ROS2 services
  std::unique_ptr<ServiceManager> service_manager_;
};

#endif  // HDR_ROS2_DRIVER_HDR_ROS2_NODE_HPP_