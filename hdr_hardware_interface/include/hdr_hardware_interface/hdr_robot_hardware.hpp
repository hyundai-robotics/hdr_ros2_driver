/**
 * @brief Declaration of ::hdr_hardware_interface::HdrRobotHardware.
 * This class is the bridge between the HD Hyundai Robotics controller (accessed
 * via the *HDR Driver* HTTP ) and the ros2_control framework.
 * It exposes *position* state/command interfaces for each joint
 * and implements the standard SystemInterface lifecycle callbacks.
 *
 * @see hdr_robot_hardware.cpp for the implementation details.
 *
 * @file hdr_robot_hardware.hpp
 * @author HD Hyundai Robotics
 * @copyright 2025 HD Hyundai Robotics
 */
#ifndef HDR_HARDWARE_INTERFACE_HDR_ROBOT_HARDWARE_HPP_
#define HDR_HARDWARE_INTERFACE_HDR_ROBOT_HARDWARE_HPP_

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hdr_client_driver/hdr_client_driver.h"  // Open API driver
#include "hdr_hardware_interface/util.hpp"        // helper utilities
#include "rclcpp/rclcpp.hpp"

namespace hdr_hardware_interface {

/**
 * @brief ros2_control *SystemInterface* implementation for HD Hyundai robots.
 *
 * @class HdrRobotHardware
 */
class HdrRobotHardware final : public hardware_interface::SystemInterface {
 public:
  // ────────────────────────────────────────────────────────────────────────────
  // Lifecycle entry points (SystemInterface overrides)
  // ────────────────────────────────────────────────────────────────────────────

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& system_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  // ────────────────────────────────────────────────────────────────────────────
  // Run‑time data — joint state & command buffers
  // ────────────────────────────────────────────────────────────────────────────
  std::vector<double> joint_positions_;        ///< Current joint positions [rad]
  std::vector<double> position_commands_;      ///< Target joint positions  [rad]
  std::vector<double> position_commands_old_;  ///< Previously sent targets
  int read_counter_ = 0;                       ///< Cycle counter for optimized API calls

  // ────────────────────────────────────────────────────────────────────────────
  // Robot state management
  // ────────────────────────────────────────────────────────────────────────────
  enum class RobotMode {
    MANUAL,     ///< cur_mode 0, 1
    AUTOMATIC,  ///< cur_mode 3, 4 + is_remote_mode 0
    REMOTE      ///< cur_mode 3, 4 + is_remote_mode 1
  };

  // Current states
  int is_remote_mode_ = 0;                    ///< Remote mode flag from robot
  int cur_mode_ = 0;                          ///< Current mode from robot
  RobotMode robot_mode_ = RobotMode::MANUAL;  ///< Processed robot mode
  int emergency_state_ = 0;                   ///< Emergency stop state (0=off, 1=on)
  int motor_state_ = 0;                       ///< Motor power state (0=off, 1=on)

  // Previous states for change detection
  RobotMode prev_mode_ = RobotMode::MANUAL;  ///< Previous robot mode
  int prev_emergency_state_ = 0;             ///< Previous emergency state
  int prev_motor_state_ = 0;                 ///< Previous motor state

  // ────────────────────────────────────────────────────────────────────────────
  // Controller management
  // ────────────────────────────────────────────────────────────────────────────
  std::shared_ptr<rclcpp::Node> node_for_services_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;
  bool restarted_controller_ = false;  ///< Flag to prevent duplicate controller restarts

  // ────────────────────────────────────────────────────────────────────────────
  // Driver & controller meta data
  // ────────────────────────────────────────────────────────────────────────────
  std::unique_ptr<hdrcl::HdrDriver> driver_;  ///< Low‑level HTTP driver

  double robot_sw_version_api_ = 0.0;  ///< API firmware version
  double robot_sw_version_sys_ = 0.0;  ///< System firmware version
  int pub_hz_ = 0;                     ///< Joint States publisher hz

  bool driver_initialized_ = false;  ///< True once driver_ has been created
  bool first_pass_ = true;           ///< First activation pass flag
  bool initialized_ = false;         ///< True after initial position sync

  // ────────────────────────────────────────────────────────────────────────────
  // Configuration parameters (filled during @ref OnConfigure)
  // ────────────────────────────────────────────────────────────────────────────
  std::string openapi_ip_ = "192.168.1.150";  ///< Controller IP address
  int openapi_port_ = 8888;                   ///< Controller HTTP port
  std::string robot_model_ = "hdf7_9";        ///< Expected robot model string
};

}  // namespace hdr_hardware_interface

#endif  // HDR_HARDWARE_INTERFACE_HDR_ROBOT_HARDWARE_HPP_
