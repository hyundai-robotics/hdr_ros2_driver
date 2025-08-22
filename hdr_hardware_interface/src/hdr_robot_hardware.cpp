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

  command_port_ =
      std::get<int>(util::GetParam(info_.hardware_parameters, "command_port", 8000, "int"));

  command_start_time_ = std::get<double>(
      util::GetParam(info_.hardware_parameters, "command_start_time", -1.0, "double"));

  command_buffer_size_ =
      std::get<int>(util::GetParam(info_.hardware_parameters, "command_buffer_size", 5, "int"));

  pub_hz_ = std::get<int>(util::GetParam(info_.hardware_parameters, "update_rate", 100, "int"));

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "Loaded parameters: update_rate=%d, start_time=%.3f, command_port=%d, "
              "buffer_size=%d",
              pub_hz_, command_start_time_, command_port_, command_buffer_size_);

  std::fill(joint_positions_.begin(), joint_positions_.end(), 0.0);

  try {
    driver_ = std::make_unique<hdrcl::HdrDriver>(openapi_ip_, openapi_port_, "UDP", command_port_);
    driver_initialized_ = true;

    // API/System version validation
    robot_sw_version_api_ = driver_->GetApiVersion();
    robot_sw_version_sys_ = driver_->GetSysVersion();
    if (robot_sw_version_sys_ < util::kMinSupportedSysVer) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                   "Unsupported API version: %.4f. Minimum required version is %.4f",
                   robot_sw_version_sys_, util::kMinSupportedSysVer);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Robot state validation
    const auto [state, state_ok] = driver_->GetProjectRgen();
    if (!state_ok) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to get robot information");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Remote mode validation
    int is_remote_mode = state["is_remote_mode"].get<int>();
    int cur_mode = state["cur_mode"].get<int>();
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

    // Robot model validation
    const std::string actual_model = state["robot_model"].get<std::string>();
    auto it = util::kAllowedMap.find(robot_model_);
    bool match = (it != util::kAllowedMap.end())
                     ? std::any_of(it->second.begin(), it->second.end(),
                                   [&](const std::string& allowed) {
                                     return util::CompareIgnoreCase(actual_model, allowed);
                                   })
                     : util::CompareIgnoreCase(robot_model_, actual_model);

    if (!match) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                   "Robot model mismatch (expected %s, got %s)", robot_model_.c_str(),
                   actual_model.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                "Robot model verified (%s), API %.2f / SYS %.4f", actual_model.c_str(),
                robot_sw_version_api_, robot_sw_version_sys_);

    // Start polling
    if (!driver_->StartPolling(pub_hz_)) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to start robot polling at %d Hz",
                   pub_hz_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    controller_active_ = true;

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

  node_for_services_ = std::make_shared<rclcpp::Node>("hdr_robot_hw_node",
                                                      rclcpp::NodeOptions()
                                                          .enable_rosout(false)
                                                          .start_parameter_services(false)
                                                          .start_parameter_event_publisher(false));

  switch_controller_client_ =
      node_for_services_->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");

  if (!driver_->PostRobotMotorPower().second) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to activate motor");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!driver_->RelayExternalStopClear().second) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to initialize remote state");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set job program
  if (!driver_->SetJobProgram(command_port_, command_start_time_, pub_hz_, command_buffer_size_)) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "Failed to set job program (port=%d, start_time=%.1f, hz=%d, buffer=%d)",
                 command_port_, command_start_time_, pub_hz_, command_buffer_size_);
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

  auto result = driver_->PostRobotEmergencyStop().second;
  if (!result) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to deactivate robot.");
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
 * - Validates driver initialization status
 * - Reads current joint positions from the robot driver
 * - Updates all robot states from rgen API (every 17 cycles)
 * - Manages controller activation/deactivation based on state changes
 * - Handles mode transitions between MANUAL, AUTOMATIC, and REMOTE
 * - Ensures safety during playback mode and motor power transitions
 *
 * Controller management logic:
 * - All conditions met (playback active, REMOTE mode, motor on): Activates
 * joint_trajectory_controller
 * - Any condition fails: Deactivates joint_trajectory_controller immediately
 * - Playback stopped: Immediately deactivates all controllers
 * - Motor off or busy: Immediately deactivates all controllers
 * - Mode changed from REMOTE: Deactivates joint_trajectory_controller
 *
 * State values:
 * - is_playback_mode_: 0 = Stopped, 1 = Playing
 * - motor_state_: 0 = ON, 1 = OFF, 2 = Busy (extracted from enable_state bit 0)
 * - robot_mode_: Mapped from cur_mode_ and is_remote_mode_
 *
 * The function uses a cyclic counter (0-1699) to optimize API calls. The counter resets at 1700
 * to prevent overflow and maintain proper alignment of the periodic state updates (multiple of 17).
 *
 * @note This function is called periodically by the ROS2 control framework.
 * @note All robot states are now retrieved from a single rgen API call instead of separate calls.
 */
hardware_interface::return_type HdrRobotHardware::read(const rclcpp::Time& time,
                                                       const rclcpp::Duration& /*period*/) {
  // Driver initialization check
  if (!driver_ || !driver_initialized_) {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"), "Driver not initialized");
    return hardware_interface::return_type::ERROR;
  }

  // Read joint positions
  auto positions = driver_->GetRobotPosition();
  if (positions.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to read joint positions");
    return hardware_interface::return_type::ERROR;
  }
  joint_positions_ = positions;

  // Update robot state every 500ms
  static rclcpp::Time last_state_update = time;
  if ((time - last_state_update) >= rclcpp::Duration(std::chrono::milliseconds(500))) {
    auto [robot_state, ok] = driver_->GetProjectRgen();
    if (ok) {
      is_remote_mode_ = robot_state["is_remote_mode"].get<int>();
      cur_mode_ = robot_state["cur_mode"].get<int>();
      is_playback_mode_ = robot_state["is_playback"].get<int>();
      motor_state_ = robot_state["enable_state"].get<int>() & 0x01;

      // Determine robot mode
      if (cur_mode_ == 0 || cur_mode_ == 1) {
        robot_mode_ = RobotMode::MANUAL;
      } else if (cur_mode_ == 3 || cur_mode_ == 4) {
        robot_mode_ = (is_remote_mode_ == 1) ? RobotMode::REMOTE : RobotMode::AUTOMATIC;
      }
      last_state_update = time;
    }
  }

  // Check activation conditions
  bool playback_ok = (is_playback_mode_ == 1);
  bool mode_ok = (robot_mode_ == RobotMode::REMOTE);
  bool motor_ok = (motor_state_ == 0);
  bool all_conditions_met = playback_ok && mode_ok && motor_ok;

  // Track condition changes for controller switching
  static bool prev_conditions = false;
  static bool switch_in_progress = false;
  static rclcpp::Time last_switch_time = time;

  bool conditions_changed = (prev_conditions != all_conditions_met);
  bool should_activate = !controller_active_ && all_conditions_met && conditions_changed;
  bool should_deactivate = controller_active_ && !all_conditions_met && conditions_changed;

  // Log condition changes
  static bool prev_playback = true, prev_mode = true, prev_motor = true;
  if (prev_playback != playback_ok || prev_mode != mode_ok || prev_motor != motor_ok) {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                "CONDITIONS - Playback:%s, Mode:%s, Motor:%s, Controller:%s",
                playback_ok ? "OK" : "FAIL", mode_ok ? "OK" : "FAIL", motor_ok ? "OK" : "FAIL",
                controller_active_ ? "ACTIVE" : "INACTIVE");
    prev_playback = playback_ok;
    prev_mode = mode_ok;
    prev_motor = motor_ok;
  }

  // Reset switch progress flag after timeout
  if (switch_in_progress &&
      (time - last_switch_time) >= rclcpp::Duration(std::chrono::milliseconds(200))) {
    switch_in_progress = false;
  }

  // Prevent duplicate requests during switch operation
  if (switch_in_progress) {
    prev_conditions = all_conditions_met;
    return hardware_interface::return_type::OK;
  }

  // Execute controller switch if needed
  if (should_activate || should_deactivate) {
    if (!switch_controller_client_ ||
        !switch_controller_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Controller service unavailable");
      prev_conditions = all_conditions_met;
      return hardware_interface::return_type::OK;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->activate_asap = true;
    request->timeout.sec = 0;
    request->timeout.nanosec = 500000000;

    // Set switch progress flags
    switch_in_progress = true;
    last_switch_time = time;

    if (should_deactivate) {
      const char* reason = !playback_ok ? "Robot not in playback mode"
                           : !mode_ok   ? "Not in REMOTE mode"
                                        : "Motor power off or busy";

      RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"), "Deactivating controllers: %s", reason);

      controller_active_ = false;
      request->deactivate_controllers = {"joint_trajectory_controller"};

      switch_controller_client_->async_send_request(
          request,
          [this](
              rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
            try {
              auto response = future.get();
              RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                          response->ok ? "Controller deactivated successfully"
                                       : "Controller deactivation failed");
            } catch (const std::exception& e) {
              RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                           "Deactivation callback exception: %s", e.what());
            }
          });

    } else if (should_activate) {
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                  "Activating controllers: All conditions met");

      // Sync commands with current positions
      position_commands_ = joint_positions_;
      position_commands_old_ = joint_positions_;

      controller_active_ = true;
      request->activate_controllers = {"joint_trajectory_controller"};

      switch_controller_client_->async_send_request(
          request,
          [this](
              rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
            try {
              auto response = future.get();
              if (response->ok) {
                RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                            "Controller activated successfully");
              } else {
                RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"), "Controller activation failed");
                controller_active_ = false;  // Rollback on failure
              }
            } catch (const std::exception& e) {
              RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                           "Activation callback exception: %s", e.what());
              controller_active_ = false;  // Rollback on exception
            }
          });
    }
  }

  prev_conditions = all_conditions_met;
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
 * - Skips sending if playback is not active
 * - Skips sending if motor is off
 * - Sends only when commands change (reduces traffic after trajectory completion)
 *
 * Constants:
 * - POSITION_EPSILON (1e-3): Threshold for detecting robot movement
 * - COMMAND_EPSILON (1e-3): Threshold for detecting command changes
 *
 * @note Only sends commands in REMOTE mode when playback is active and motor is on.
 *       Eliminates redundant transmissions after trajectory completion.
 */
hardware_interface::return_type HdrRobotHardware::write(const rclcpp::Time& /*unused*/,
                                                        const rclcpp::Duration& /*unused*/) {
  constexpr double POSITION_EPSILON = 1e-3;
  constexpr double COMMAND_EPSILON = 1e-3;

  if (!driver_ || !driver_initialized_) {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"), "Driver not initialized — skipping write");
    return hardware_interface::return_type::ERROR;
  }

  if (robot_mode_ == RobotMode::MANUAL) {
    position_commands_ = joint_positions_;
    position_commands_old_ = joint_positions_;
    return hardware_interface::return_type::OK;
  }

  if (robot_mode_ == RobotMode::REMOTE && is_playback_mode_ == 1 && motor_state_ == 0) {
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

    if (!is_moving && is_same_command) {
      return hardware_interface::return_type::OK;
    }

    if (!driver_->SetRobotPosition(position_commands_)) {
      return hardware_interface::return_type::ERROR;
    }

    position_commands_old_ = position_commands_;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hdr_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hdr_hardware_interface::HdrRobotHardware,
                       hardware_interface::SystemInterface)
