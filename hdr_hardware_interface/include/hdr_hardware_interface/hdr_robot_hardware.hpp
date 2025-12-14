/**
 * @brief Declaration of ::hdr_hardware_interface::HdrRobotHardware.
 *
 * This class is the bridge between the HD Hyundai Robotics controller (accessed
 * via the *HDR Driver* HTTP and socket Stream) and the ros2_control framework.
 * It exposes *position, velocity, and effort* state interfaces and *position*
 * command interfaces for each joint, implementing the standard SystemInterface
 * lifecycle callbacks.
 *
 * @see hdr_robot_hardware.cpp for the implementation details.
 *
 * @file hdr_robot_hardware.hpp
 * @author HD Hyundai Robotics
 * @copyright 2025 HD Hyundai Robotics
 */

#ifndef HDR_HARDWARE_INTERFACE_HDR_ROBOT_HARDWARE_HPP_
#define HDR_HARDWARE_INTERFACE_HDR_ROBOT_HARDWARE_HPP_

#include <atomic>
#include <chrono>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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

  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  // ────────────────────────────────────────────────────────────────────────────
  // Configuration helpers (called during on_configure)
  // ────────────────────────────────────────────────────────────────────────────

  /**
   * @brief Loads configuration parameters and initializes the driver.
   * @return `true` on success, `false` on failure.
   */
  bool LoadParametersAndInitializeDriver();

  /**
   * @brief Validates robot operational state and model configuration.
   * @return `true` on success, `false` on failure.
   */
  bool ValidateRobotStateAndModel();

  // ────────────────────────────────────────────────────────────────────────────
  // Activation helpers (called during on_activate)
  // ────────────────────────────────────────────────────────────────────────────

  /**
   * @brief Initializes robot operation with motor power and job program.
   * @return `true` on success, `false` on failure.
   */
  bool InitializeRobotOperation();

  /**
   * @brief Sets up periodic state monitoring and controller management.
   * @return `true` on success, `false` on failure.
   */
  bool SetupStateMonitoring();

  /**
   * @brief Synchronizes initial joint positions with robot.
   * @return `true` on success, `false` on failure.
   */
  bool SyncInitialPosition();

  // ────────────────────────────────────────────────────────────────────────────
  // State monitoring callback
  // ────────────────────────────────────────────────────────────────────────────

  /**
   * @brief Periodic callback for state monitoring and controller switching.
   */
  void StateMonitoringCallback();

  // ────────────────────────────────────────────────────────────────────────────
  // Deactivation/Cleanup/Shutdown helpers
  // ────────────────────────────────────────────────────────────────────────────

  enum class CleanupLevel {
    DEACTIVATE,  ///< Stop state monitoring only (timer/executor)
    CLEANUP,     ///< Stop state monitoring + polling/sending
    SHUTDOWN     ///< Full cleanup + emergency stop
  };

  /**
   * @brief Common cleanup routine for deactivation, cleanup, and shutdown.
   * @param level The cleanup level to perform.
   * @return `true` on success, `false` if any errors occurred (non-fatal).
   */
  bool CleanupResources(CleanupLevel level);

  // ────────────────────────────────────────────────────────────────────────────
  // Helper functions for robot state management
  // ────────────────────────────────────────────────────────────────────────────

  /**
   * @brief Updates robot state variables from the robot controller.
   * @return Pair of (state json object, success flag). If failed, returns empty json and false.
   */
  std::pair<nlohmann::json, bool> UpdateRobotState();

  /**
   * @brief Checks if robot is in REMOTE mode (cur_mode 3 or 4 and is_remote_mode 1).
   * @return `true` if in REMOTE mode, `false` otherwise.
   */
  bool IsRemoteMode() const;

  // ────────────────────────────────────────────────────────────────────────────
  // Run‑time data — joint state & command buffers
  // ────────────────────────────────────────────────────────────────────────────

  std::vector<double> joint_positions_;        ///< Current joint positions [rad]
  std::vector<double> joint_velocities_;       ///< Current joint velocities [rad/s]
  std::vector<double> joint_efforts_;          ///< Current joint efforts/torques
  std::vector<double> position_commands_;      ///< Target joint positions  [rad]
  std::vector<double> position_commands_old_;  ///< Previously sent targets

  // ────────────────────────────────────────────────────────────────────────────
  // Robot state management
  // ────────────────────────────────────────────────────────────────────────────

  enum class RobotMode {
    MANUAL,     ///< cur_mode 0, 1
    AUTOMATIC,  ///< cur_mode 3, 4 + is_remote_mode 0
    REMOTE      ///< cur_mode 3, 4 + is_remote_mode 1
  };

  int is_remote_mode_ = 0;                    ///< Remote mode flag from robot (0/1)
  int cur_mode_ = 0;                          ///< Current mode from robot (0,1,3,4)
  int is_playback_mode_ = 0;                  ///< Playback state (0=Stopped, 1=Playing)
  RobotMode robot_mode_ = RobotMode::MANUAL;  ///< Processed robot mode
  int motor_state_ = 1;                       ///< Motor power state (0=ON, 1=OFF, 2=BUSY)

  // ────────────────────────────────────────────────────────────────────────────
  // Controller management
  // ────────────────────────────────────────────────────────────────────────────

  std::shared_ptr<rclcpp::Node> node_for_services_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_client_;

  bool controller_active_ = false;  ///< Flag indicating if joint_trajectory_controller is active
  std::atomic<bool> switch_in_progress_{false};  ///< Flag to prevent concurrent controller switches

  rclcpp::TimerBase::SharedPtr state_timer_;        ///< Timer for periodic state monitoring
  rclcpp::executors::SingleThreadedExecutor exec_;  ///< Executor for service callbacks
  std::thread spin_thread_;                         ///< Thread for executor spinning

  // ────────────────────────────────────────────────────────────────────────────
  // Driver & controller meta data
  // ────────────────────────────────────────────────────────────────────────────

  std::unique_ptr<hdrcl::HdrDriver> driver_;  ///< Low‑level HTTP and socket Stream driver
  double robot_sw_version_api_ = 0.0;         ///< API firmware version
  double robot_sw_version_sys_ = 0.0;         ///< System firmware version
  int pub_hz_ = 0;                            ///< Polling frequency in Hz

  bool driver_initialized_ = false;  ///< True once driver_ has been created
  bool first_pass_ = true;           ///< First activation pass flag
  bool initialized_ = false;         ///< True after initial position sync

  // ────────────────────────────────────────────────────────────────────────────
  // Configuration parameters (filled during on_configure)
  // ────────────────────────────────────────────────────────────────────────────

  std::string openapi_ip_ = "192.168.1.150";  ///< Controller IP address
  std::string robot_model_ = "hdf7_9";        ///< Expected robot model string
  int command_buffer_size_ = 5;               ///< Trajectory buffer size for smooth execution
};

}  // namespace hdr_hardware_interface

#endif  // HDR_HARDWARE_INTERFACE_HDR_ROBOT_HARDWARE_HPP_
