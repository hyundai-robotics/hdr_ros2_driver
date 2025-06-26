/**
 * @brief Implementation of ::hdr_hardware_interface::HDRRobotHardware for
 *        ROS2 control (ros2_control) integration.
 * This file provides the concrete implementation of the HDRRobotHardware class,
 * which bridges the HD Hyundai Robotics Open API (HTTP) and the
 * ros2_control framework.  The class exposes joint state and command interfaces
 * and orchestrates driver initialisation, robot activation/de‑activation, and
 * cyclic read/write calls.
 * Do **NOT** change functional behaviour here unless you also validate the
 * corresponding firmware versions on the real controller.  Minor refactors for
 * readability, logging, or documentation are welcome.
 *
 * @file hdr_robot_hardware.cpp
 * @author HD Hyundai Robotics
 */
#include "hdr_hardware_interface/hdr_robot_hardware.hpp"

namespace hdr_hardware_interface {

/**
 * @param[in] system_info  Hardware description supplied by ros2_control.
 *
 * @return `CallbackReturn::SUCCESS` when ready, otherwise
 *         `CallbackReturn::ERROR`.
 *
 * @brief Validates joint interfaces and allocates internal buffers.
 * This method is called exactly *once* when the component is first loaded by
 * the controller manager. If the joint interface layout in the URDF/xacro
 * does not match the expected layout (i.e., 'position' interface must exist
 * for both *state* and *command*), the initialization will fail early so the
 * integrator can correct the robot description before run-time.
 *
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_init(
    const hardware_interface::HardwareInfo& system_info) {
  if (hardware_interface::SystemInterface::on_init(system_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = system_info;
  size_t num_joints = info_.joints.size();
  joint_positions_.resize(num_joints, 0.0);
  position_commands_.resize(num_joints, 0.0);
  position_commands_old_.resize(num_joints, 0.0);

  driver_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "Initializing hardware interface with %zu joints", num_joints);

  // Validate every joint entry
  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // Command interfaces
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("HdrRobotHardware"),
                   "Joint '%s' must expose POSITION command interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // State interfaces
    if (joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("HdrRobotHardware"),
                   "Joint '%s' must expose POSITION state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (num_joints < 1) {
      RCLCPP_FATAL(rclcpp::get_logger("HdrRobotHardware"),
                   "No joints found in the robot description");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                "HdrRobotHardware initialized with %zu joints.", num_joints);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
// Interface exposure
// ──────────────────────────────────────────────────────────────────────────────
/**
 * @return A vector of state interfaces.
 *
 * @brief Exports state and command interfaces for the hardware.
 * This method is called when the controller manager loads the hardware
 * interface. It exports the state and command interfaces for each joint.
 *
 */
std::vector<hardware_interface::StateInterface> HdrRobotHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                "Exporting State Interface for Joint[%zu]: %s", i, info_.joints[i].name.c_str());

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
  }
  // Software version meta data (exposed as additional state variables)
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "get_robot_sw_version", "api_version", &robot_sw_version_api_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "get_robot_sw_version", "sys_version", &robot_sw_version_sys_));

  return state_interfaces;
}
/**
 * @return A vector of command interfaces.
 *
 * @brief Exports command interfaces for the hardware.
 * This method is called when the controller manager loads the hardware
 * interface. It exports the command interfaces for each joint.
 *
 */
std::vector<hardware_interface::CommandInterface> HdrRobotHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }
  return command_interfaces;
}
// ──────────────────────────────────────────────────────────────────────────────
// Lifecycle — configuration & activation
// ──────────────────────────────────────────────────────────────────────────────
/**
 * @param previous_state The previous state of the hardware interface.
 *
 * @return CallbackReturn::SUCCESS on success, otherwise an error code.
 *
 * @brief Configures the hardware interface.
 * This method is called when the controller is configured. It initializes
 * the hardware interface and prepares it for operation.
 *
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_configure(
    const rclcpp_lifecycle::State& /*unused*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "OnConfigure please wait");

  openapi_ip_ = std::get<std::string>(util::GetParam(info_.hardware_parameters, "openapi_ip",
                                                     std::string("192.168.1.150"), "string"));
  openapi_port_ =
      std::get<int>(util::GetParam(info_.hardware_parameters, "openapi_port", 8888, "int"));
  robot_model_ = std::get<std::string>(
      util::GetParam(info_.hardware_parameters, "robot_model", std::string("hdf7_9"), "string"));

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Parameters: ip=%s:%d, model=%s",
              openapi_ip_.c_str(), openapi_port_, robot_model_.c_str());

  std::fill(joint_positions_.begin(), joint_positions_.end(), 0.0);

  try {
    driver_ = std::make_unique<hdrcl::HdrDriver>(openapi_ip_, openapi_port_);

    driver_initialized_ = true;

    robot_sw_version_api_ = driver_->GetApiVersion();
    robot_sw_version_sys_ = driver_->GetSysVersion();

    if (robot_sw_version_sys_ < util::kMinSupportedSysVer) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                   "Unsupported API version: %.4f. Minimum required version is %.4f",
                   robot_sw_version_sys_, util::kMinSupportedSysVer);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto [state, ok] = driver_->GetProjectRgen();

    if (!ok) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to get robot information");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check if robot is in remote mode
    int is_remote_mode = state["is_remote_mode"].get<int>();
    int cur_mode = state["cur_mode"].get<int>();

    // Remote mode validation: cur_mode should be 3 or 4, and is_remote_mode should be 1
    if (!((cur_mode == 3 || cur_mode == 4) && is_remote_mode == 1)) {
      std::string mode_str = (cur_mode == 0 || cur_mode == 1) ? "MANUAL"
                             : (cur_mode == 3 || cur_mode == 4)
                                 ? (is_remote_mode == 1 ? "REMOTE" : "AUTOMATIC")
                                 : "UNKNOWN";

      RCLCPP_ERROR(
          rclcpp::get_logger("HdrRobotHardware"),
          "Robot is not in remote mode. Current mode: %s (cur_mode: %d, is_remote_mode: %d)",
          mode_str.c_str(), cur_mode, is_remote_mode);
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string actual_model = state["robot_model"].get<std::string>();
    bool match = false;

    auto it = util::kAllowedMap.find(robot_model_);
    if (it != util::kAllowedMap.end()) {
      match = std::any_of(it->second.begin(), it->second.end(), [&](const std::string& allowed) {
        return util::CompareIgnoreCase(actual_model, allowed);
      });
    }
    if (!match && !util::CompareIgnoreCase(robot_model_, actual_model)) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                   "Robot model mismatch (expected %s, got %s)", robot_model_.c_str(),
                   actual_model.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                "Robot model verified (%s), API %.2f / SYS %.4f", actual_model.c_str(),
                robot_sw_version_api_, robot_sw_version_sys_);

    if (auto param = info_.hardware_parameters.find("update_rate");
        param != info_.hardware_parameters.end()) {
      pub_hz_ = std::stoi(param->second);
    } else {
      pub_hz_ = 50;  // default fallback
    }

    driver_->StartPolling(pub_hz_);

    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Configuration failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

/**
 * @param previous_state The previous state of the hardware interface.
 *
 * @return CallbackReturn::SUCCESS on success, otherwise an error code.
 *
 * @brief Activates the hardware interface.
 * This method is called when the controller is activated. It initializes the
 * robot and prepares it for operation.
 *
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_activate(
    const rclcpp_lifecycle::State& /*unused*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Activating hardware...");

  if (!driver_ || !driver_initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "HDR driver is not initialized yet. Activation failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_for_services_ = std::make_shared<rclcpp::Node>("hdr_robot_hw_node");

  switch_controller_client_ =
      node_for_services_->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");

  if (!driver_->PostRobotMotorPower(true).second) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to activate motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // First-time position sync
  if (first_pass_ && !initialized_) {
    const int max_position_retries = 3;
    const std::chrono::milliseconds position_retry_delay(100);
    bool position_initialized = false;

    for (int attempt = 1; attempt <= max_position_retries; ++attempt) {
      const auto positions = driver_->GetRobotPosition();
      if (!positions.empty()) {
        joint_positions_ = positions;
        position_commands_ = positions;
        position_commands_old_ = positions;

        first_pass_ = false;
        initialized_ = true;
        position_initialized = true;

        RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                    "First-pass init: command = current robot position (attempt %d/%d).", attempt,
                    max_position_retries);
        break;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                    "Initial joint position read failed or empty (attempt %d/%d).", attempt,
                    max_position_retries);

        if (attempt < max_position_retries) {
          std::this_thread::sleep_for(position_retry_delay);
        }
      }
    }

    if (!position_initialized) {
      RCLCPP_ERROR(
          rclcpp::get_logger("HdrRobotHardware"),
          "Failed to read initial joint positions after %d attempts. Hardware activation failed.",
          max_position_retries);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @param previous_state The previous state of the hardware interface.
 *
 * @return CallbackReturn::SUCCESS on success, otherwise an error code.
 *
 * @brief Deactivates the hardware interface.
 * This method is called when the controller is deactivated. It stops the
 * robot and releases any resources.
 *
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*unused*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Deactivating hardware...");

  if (!driver_ || !driver_initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "HDR driver is not initialized yet. Deactivation failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto result = driver_->PostRobotMotorPower(false).second;
  if (!result) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to deactivate robot motor.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  driver_->StopPolling();

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────────────────────
// Cyclic read / write
// ──────────────────────────────────────────────────────────────────────────────
/**
 * @param time The current time (currently unused).
 * @param period The time since the last read (currently unused).
 *
 * @return hardware_interface::return_type::OK on success,
 *         hardware_interface::return_type::ERROR on failure.
 *
 * @brief Read joint positions from the robot and update system state.
 *
 * @details
 * This function performs the following operations:
 * - Reads current joint positions from the robot driver
 * - Updates robot mode state (every 11 cycles)
 * - Updates emergency stop status (every 13 cycles)
 * - Updates motor power status (every 17 cycles)
 * - Manages controller activation/deactivation based on state changes
 * - Handles mode transitions between MANUAL, AUTOMATIC, and REMOTE
 * - Ensures safety during emergency stop and motor power transitions
 *
 * Controller management logic:
 * - REMOTE → MANUAL/AUTO: Deactivates joint_trajectory_controller
 * - MANUAL/AUTO → REMOTE: Activates joint_trajectory_controller (if conditions met)
 * - Emergency activated: Immediately deactivates all controllers
 * - Motor off: Immediately deactivates all controllers
 * - Emergency released + Motor on: Reactivates controllers if in REMOTE mode
 * - Motor on: Reactivates controllers if emergency off and in REMOTE mode
 *
 * The function uses a cyclic counter (0-142) to optimize API calls.
 *
 * @note This function is called periodically by the ROS2 control framework.
 */
hardware_interface::return_type HdrRobotHardware::read(const rclcpp::Time& /*unused*/,
                                                       const rclcpp::Duration& /*unused*/) {
  if (!driver_ || !driver_initialized_) {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                "Driver not initialized. Returning default positions.");
    return hardware_interface::return_type::ERROR;
  }

  auto positions = driver_->GetRobotPosition();
  if (positions.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "Failed to read joint positions from driver.");
    return hardware_interface::return_type::ERROR;
  }

  joint_positions_ = positions;
  read_counter_++;

  // ============================================================================
  // State Updates (each with different cycles)
  // ============================================================================

  // Robot mode state update (every 11 cycles)
  if (read_counter_ % 11 == 0) {
    auto [robot_state, ok] = driver_->GetProjectRgen();
    if (ok) {
      is_remote_mode_ = robot_state["is_remote_mode"].get<int>();
      cur_mode_ = robot_state["cur_mode"].get<int>();

      if (cur_mode_ == 0 || cur_mode_ == 1) {
        robot_mode_ = RobotMode::MANUAL;
      } else if (cur_mode_ == 3 || cur_mode_ == 4) {
        robot_mode_ = is_remote_mode_ == 1 ? RobotMode::REMOTE : RobotMode::AUTOMATIC;
      }
    }
  }

  // Emergency state update (every 13 cycles)
  if (read_counter_ % 13 == 0) {
    auto [emg_state, ok] = driver_->GetEmergencyStop();
    if (ok) {
      emergency_state_ = emg_state["val"].get<int>();
    }
  }

  // Motor state update (every 17 cycles)
  if (read_counter_ % 17 == 0) {
    auto [motor_state, ok] = driver_->GetRobotMotorState();
    if (ok) {
      motor_state_ = motor_state["val"].get<int>();
    }
  }

  // ============================================================================
  // Controller Management Logic (condition-based processing)
  // ============================================================================
  bool should_deactivate = false;
  bool should_activate = false;
  std::string reason;

  // 1. Emergency activation - highest priority safety measure
  if (prev_emergency_state_ == 0 && emergency_state_ == 1) {
    should_deactivate = true;
    reason = "Emergency stop activated";
    restarted_controller_ = false;
    prev_emergency_state_ = emergency_state_;
  }
  // 2. Motor power off - deactivate controllers
  else if (prev_motor_state_ == 0 && motor_state_ == 1) {  // 0(on) -> 1(off)
    should_deactivate = true;
    reason = "Motor power turned off";
    restarted_controller_ = false;
    prev_motor_state_ = motor_state_;
  }
  // 3. Not in REMOTE mode - always deactivate controllers
  else if (robot_mode_ != RobotMode::REMOTE && restarted_controller_) {
    should_deactivate = true;
    reason = std::string("Not in REMOTE mode (current: ") +
             (robot_mode_ == RobotMode::MANUAL ? "MANUAL" : "AUTOMATIC") + ")";
    restarted_controller_ = false;
  }
  // 4. Emergency release and motor on - check if we should reactivate controllers
  else if ((prev_emergency_state_ == 1 && emergency_state_ == 0) ||
           (prev_motor_state_ == 1 && motor_state_ == 0)) {  // 1(off) -> 0(on)
    // Sync to current position first
    position_commands_ = joint_positions_;
    position_commands_old_ = joint_positions_;

    // Only activate if both emergency is off AND motor is on AND in REMOTE mode
    if (emergency_state_ == 0 && motor_state_ == 0 && robot_mode_ == RobotMode::REMOTE) {
      should_activate = true;
      if (prev_emergency_state_ == 1) {
        reason = "Emergency stop released and motor on - reactivating controllers";
      } else {
        reason = "Motor power turned on - reactivating controllers in REMOTE mode";
      }
      restarted_controller_ = true;
    }

    prev_emergency_state_ = emergency_state_;
    prev_motor_state_ = motor_state_;
  }
  // 5. Mode change to REMOTE (when emergency off and motor on)
  else if (emergency_state_ == 0 && motor_state_ == 0 && robot_mode_ == RobotMode::REMOTE &&
           prev_mode_ != RobotMode::REMOTE && !restarted_controller_) {
    should_activate = true;
    reason = std::string("Mode changed to REMOTE from ") +
             (prev_mode_ == RobotMode::MANUAL ? "MANUAL" : "AUTOMATIC");
    // Sync command to current position
    position_commands_ = joint_positions_;
    position_commands_old_ = joint_positions_;
    restarted_controller_ = true;
  }

  prev_mode_ = robot_mode_;

  // Controller service call
  if (should_deactivate || should_activate) {
    if (switch_controller_client_ &&
        switch_controller_client_->wait_for_service(std::chrono::seconds(1))) {
      auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

      if (should_deactivate) {
        req->deactivate_controllers = {"joint_trajectory_controller"};
        RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"), "Deactivating controllers: %s",
                    reason.c_str());
      } else {
        req->activate_controllers = {"joint_trajectory_controller"};
        RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Activating controllers: %s",
                    reason.c_str());
      }

      req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
      req->activate_asap = true;
      req->timeout.sec = 1;
      req->timeout.nanosec = 0;

      switch_controller_client_->async_send_request(req);
    }
  }

  if (read_counter_ >= 142) {
    read_counter_ = 0;
  }

  return hardware_interface::return_type::OK;
}
/**
 * @param time The current time (currently unused).
 * @param period The time since the last write (currently unused).
 *
 * @return hardware_interface::return_type::OK on success,
 *         hardware_interface::return_type::ERROR on failure.
 *
 * @brief Write joint commands to the robot with intelligent filtering.
 *
 * @details
 * This function implements smart command filtering to reduce unnecessary
 * network traffic to the robot controller:
 *
 * Filtering conditions:
 * - Skips sending if robot is in MANUAL mode
 * - Skips sending if emergency stop is active
 * - Skips sending if motor is off
 * - Sends only when commands change (reduces traffic after trajectory completion)
 * // - Forces sending every FORCE_SEND_INTERVAL seconds to prevent power saving mode (disabled)
 *
 * Constants:
 * - POSITION_EPSILON (1e-3): Threshold for detecting robot movement
 * - COMMAND_EPSILON (1e-3): Threshold for detecting command changes
 * // - FORCE_SEND_INTERVAL (0.2s): Maximum interval between sends to prevent power saving
 * (disabled)
 *
 * @note Only sends commands in REMOTE mode when emergency stop is inactive and motor is on.
 *       Eliminates redundant transmissions after trajectory completion.
 *       // Always sends commands every 200ms to prevent robot from entering power saving mode
 * (disabled).
 */
hardware_interface::return_type HdrRobotHardware::write(const rclcpp::Time& /*unused*/,
                                                        const rclcpp::Duration& /*unused*/) {
  constexpr double POSITION_EPSILON = 1e-3;
  constexpr double COMMAND_EPSILON = 1e-3;
  // constexpr double FORCE_SEND_INTERVAL = 0.2;

  if (!driver_ || !driver_initialized_) {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"), "Driver not initialized — skipping write");
    return hardware_interface::return_type::ERROR;
  }

  // static rclcpp::Time last_write_time = rclcpp::Clock().now();

  if (robot_mode_ == RobotMode::MANUAL) {
    position_commands_ = joint_positions_;
    position_commands_old_ = joint_positions_;
    // last_write_time = rclcpp::Clock().now();
    return hardware_interface::return_type::OK;
  }

  if (robot_mode_ == RobotMode::REMOTE && emergency_state_ == 0 && motor_state_ == 0) {
    bool is_moving = false;
    for (size_t i = 0; i < position_commands_.size(); ++i) {
      if (std::abs(position_commands_[i] - joint_positions_[i]) > POSITION_EPSILON) {
        is_moving = true;
        break;
      }
    }

    bool is_same_command = true;
    for (size_t i = 0; i < position_commands_.size(); ++i) {
      if (std::abs(position_commands_[i] - position_commands_old_[i]) > COMMAND_EPSILON) {
        is_same_command = false;
        break;
      }
    }

    // rclcpp::Duration elapsed = rclcpp::Clock().now() - last_write_time;
    // bool force_send = elapsed.seconds() > FORCE_SEND_INTERVAL;

    if (!is_moving && is_same_command /* && !force_send */) {
      return hardware_interface::return_type::OK;
    }

    if (!driver_->SetRobotPosition(position_commands_)) {
      return hardware_interface::return_type::ERROR;
    }

    position_commands_old_ = position_commands_;
    // last_write_time = rclcpp::Clock().now();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hdr_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hdr_hardware_interface::HdrRobotHardware,
                       hardware_interface::SystemInterface)
