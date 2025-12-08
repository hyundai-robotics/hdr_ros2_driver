/**
 * @brief Implementation of ::hdr_hardware_interface::HDRRobotHardware for
 *        ROS2 control (ros2_control) integration.
 *
 * This file provides the concrete implementation of the HDRRobotHardware class,
 * which bridges the HD Hyundai Robotics Open API (socket Stream) and the
 * ros2_control framework. The class exposes joint state and command interfaces
 * and orchestrates driver initialisation, robot activation/de‑activation, and
 * cyclic read/write calls.
 *
 * Do **NOT** change functional behaviour here unless you also validate the
 * corresponding firmware versions on the real controller. Minor refactors for
 * readability, logging, or documentation are welcome.
 *
 * @file hdr_robot_hardware.cpp
 * @author HD Hyundai Robotics
 */
#include "hdr_hardware_interface/hdr_robot_hardware.hpp"

namespace hdr_hardware_interface {

namespace {
constexpr double POSITION_EPSILON = 1e-3;
constexpr double COMMAND_EPSILON = 1e-3;
constexpr int MAX_POSITION_RETRIES = 3;
constexpr int STATE_TIMER_PERIOD_MS = 200;
constexpr int POSITION_RETRY_DELAY_MS = 100;
constexpr int SERVICE_WAIT_TIMEOUT_MS = 1;
constexpr int64_t SWITCH_TIMEOUT_NANOSEC = 500000000;
}  // namespace

// ══════════════════════════════════════════════════════════════════════════════
// Initialization
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @param[in] system_info Hardware description supplied by ros2_control.
 *
 * @return `CallbackReturn::SUCCESS` when ready, otherwise `CallbackReturn::ERROR`.
 *
 * @brief Validates joint interfaces and allocates internal buffers.
 *
 * This method is called exactly *once* when the component is first loaded by
 * the controller manager. If the joint interface layout in the URDF/xacro
 * does not match the expected layout (i.e., 'position' interface must exist
 * for both *state* and *command*), the initialization will fail early so the
 * integrator can correct the robot description before run-time.
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_init(
    const hardware_interface::HardwareInfo& system_info) {
  if (hardware_interface::SystemInterface::on_init(system_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = system_info;
  const size_t num_joints = info_.joints.size();

  if (num_joints < 1) {
    RCLCPP_FATAL(rclcpp::get_logger("HdrRobotHardware"),
                 "No joints found in the robot description");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_positions_.resize(num_joints, 0.0);
  joint_velocities_.resize(num_joints, 0.0);
  joint_efforts_.resize(num_joints, 0.0);
  position_commands_.resize(num_joints, 0.0);
  position_commands_old_.resize(num_joints, 0.0);

  driver_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "Initializing hardware interface with %zu joints", num_joints);

  for (const auto& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("HdrRobotHardware"),
                   "Joint '%s' must expose POSITION command interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position = false;

    for (const auto& state_if : joint.state_interfaces) {
      if (state_if.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      }
    }

    if (!has_position) {
      RCLCPP_FATAL(rclcpp::get_logger("HdrRobotHardware"),
                   "Joint '%s' does not have POSITION state interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "HdrRobotHardware initialized with %zu joints.", num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ══════════════════════════════════════════════════════════════════════════════
// Interface exposure
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @return A vector of state interfaces.
 *
 * @brief Exports state interfaces for the hardware.
 *
 * This method is called when the controller manager loads the hardware
 * interface. It exports the state interfaces for each joint (position, velocity, effort)
 * and additional software version metadata as state variables.
 */
std::vector<hardware_interface::StateInterface> HdrRobotHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                "Exporting State Interface for Joint[%zu]: %s", i, info_.joints[i].name.c_str());

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
  }

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
 *
 * This method is called when the controller manager loads the hardware
 * interface. It exports the command interfaces for each joint.
 */
std::vector<hardware_interface::CommandInterface> HdrRobotHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }
  return command_interfaces;
}

// ══════════════════════════════════════════════════════════════════════════════
// Lifecycle — configuration & activation
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @param[in] previous_state The previous state of the hardware interface.
 *
 * @return `CallbackReturn::SUCCESS` on success, otherwise an error code.
 *
 * @brief Configures the hardware interface.
 *
 * This method is called when the controller is configured. It loads parameters,
 * initializes the driver, validates firmware versions and robot state, verifies
 * the robot model, and starts polling.
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "OnConfigure please wait");

  if (!LoadParametersAndInitializeDriver()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!ValidateRobotStateAndModel()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!driver_->DoHandshake(util::kStreamSvrVer, 5000)) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to perform socket Stream handshake");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "socket Stream handshake completed successfully");

  if (!driver_->StartPolling(pub_hz_)) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to start robot polling at %d Hz",
                 pub_hz_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!driver_->StartSending()) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to start async send thread");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "Async send thread started for non-blocking command transmission");

  // Create service node for controller switching
  node_for_services_ = std::make_shared<rclcpp::Node>("hdr_robot_hw_node",
                                                      rclcpp::NodeOptions()
                                                          .enable_rosout(false)
                                                          .start_parameter_services(false)
                                                          .start_parameter_event_publisher(false));

  switch_controller_client_ =
      node_for_services_->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "Service node and controller switching client created");

  controller_active_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @return `true` on success, `false` on failure.
 *
 * @brief Loads configuration parameters and initializes the driver.
 *
 * Retrieves all hardware parameters from the robot description, creates the
 * driver instance, and validates firmware versions against minimum requirements.
 */
bool HdrRobotHardware::LoadParametersAndInitializeDriver() {
  openapi_ip_ = std::get<std::string>(util::GetParam(info_.hardware_parameters, "openapi_ip",
                                                     std::string("192.168.1.150"), "string"));
  robot_model_ = std::get<std::string>(
      util::GetParam(info_.hardware_parameters, "robot_model", std::string("hdf7_9"), "string"));
  command_buffer_size_ =
      std::get<int>(util::GetParam(info_.hardware_parameters, "command_buffer_size", 5, "int"));
  pub_hz_ = std::get<int>(util::GetParam(info_.hardware_parameters, "update_rate", 100, "int"));

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "Loaded parameters: openapi_ip=%s, robot_model=%s, update_rate=%d Hz, "
              "command_buffer_size=%d",
              openapi_ip_.c_str(), robot_model_.c_str(), pub_hz_, command_buffer_size_);

  std::fill(joint_positions_.begin(), joint_positions_.end(), 0.0);
  std::fill(joint_velocities_.begin(), joint_velocities_.end(), 0.0);
  std::fill(joint_efforts_.begin(), joint_efforts_.end(), 0.0);

  try {
    driver_ = std::make_unique<hdrcl::HdrDriver>(openapi_ip_);
    driver_initialized_ = true;

    robot_sw_version_api_ = driver_->GetApiVersion();
    robot_sw_version_sys_ = driver_->GetSysVersion();

    if (robot_sw_version_sys_ < util::kMinSupportedSysVer) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                   "Unsupported system version: %.4f. Minimum required: %.4f",
                   robot_sw_version_sys_, util::kMinSupportedSysVer);
      return false;
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Configuration failed: %s", e.what());
    return false;
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// Helper functions for robot state management
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @return Pair of (state json object, success flag). First element contains
 *         the complete robot state as JSON. Second element is true on success,
 *         false on failure. If failed, returns empty json and false.
 *
 * @brief Updates robot state variables from the robot controller.
 *
 * Fetches the latest robot state via GetProjectRgen API and updates internal
 * state variables including is_remote_mode_, cur_mode_, is_playback_mode_,
 * motor_state_, and robot_mode_. Also returns the full state JSON object
 * for additional processing.
 */
std::pair<nlohmann::json, bool> HdrRobotHardware::UpdateRobotState() {
  const auto [state, state_ok] = driver_->GetProjectRgen();
  if (!state_ok) {
    RCLCPP_DEBUG(rclcpp::get_logger("HdrRobotHardware"),
                 "GetProjectRgen failed - API call returned error. Response: %s",
                 state.dump().c_str());
    return {nlohmann::json{}, false};
  }

  is_remote_mode_ = state["is_remote_mode"].get<int>();
  cur_mode_ = state["cur_mode"].get<int>();
  is_playback_mode_ = state["is_playback"].get<int>();
  motor_state_ = state["enable_state"].get<int>() & 0x01;

  if (cur_mode_ == 0 || cur_mode_ == 1) {
    robot_mode_ = RobotMode::MANUAL;
  } else if (cur_mode_ == 3 || cur_mode_ == 4) {
    robot_mode_ = (is_remote_mode_ == 1) ? RobotMode::REMOTE : RobotMode::AUTOMATIC;
  }

  return {state, true};
}

/**
 * @return `true` if robot is in REMOTE mode, `false` otherwise.
 *
 * @brief Checks if robot is in REMOTE mode.
 *
 * Validates that the robot is in REMOTE mode by checking if cur_mode
 * is 3 or 4 (automatic modes) AND is_remote_mode flag is 1.
 */
bool HdrRobotHardware::IsRemoteMode() const {
  return ((cur_mode_ == 3 || cur_mode_ == 4) && is_remote_mode_ == 1);
}

/**
 * @return `true` on success, `false` on failure.
 *
 * @brief Validates robot operational state and model configuration.
 *
 * Checks that the robot is in REMOTE mode, verifies the robot model matches
 * the configuration, and ensures the robot is ready for operation.
 */
bool HdrRobotHardware::ValidateRobotStateAndModel() {
  const auto [state, state_ok] = UpdateRobotState();
  if (!state_ok) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to get robot information");
    return false;
  }

  if (!IsRemoteMode()) {
    std::string mode_str = (cur_mode_ == 0 || cur_mode_ == 1) ? "MANUAL"
                           : (cur_mode_ == 3 || cur_mode_ == 4)
                               ? (is_remote_mode_ == 1 ? "REMOTE" : "AUTOMATIC")
                               : "UNKNOWN";
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "Robot is not in remote mode. Current mode: %s (cur_mode: %d, is_remote_mode: %d)",
                 mode_str.c_str(), cur_mode_, is_remote_mode_);
    return false;
  }

  // Validate robot model (using state from UpdateRobotState)
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
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
              "Robot model verified (%s), API %.2f / SYS %.4f", actual_model.c_str(),
              robot_sw_version_api_, robot_sw_version_sys_);

  return true;
}

/**
 * @param[in] previous_state The previous state of the hardware interface.
 *
 * @return `CallbackReturn::SUCCESS` on success, otherwise an error code.
 *
 * @brief Activates the hardware interface.
 *
 * This method is called when the controller is activated. It performs the full
 * activation sequence: initializes services, powers on the robot motor, clears
 * safety stops, configures the job program, starts state monitoring, and
 * synchronizes initial joint data.
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Activating hardware...");

  if (!driver_ || !driver_initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "HDR driver is not initialized yet. Activation failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!InitializeRobotOperation()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!SetupStateMonitoring()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!SyncInitialPosition()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @return `true` on success, `false` on failure.
 *
 * @brief Initializes robot operation with motor power and job program.
 *
 * Activates motor power, clears external stops, initializes trajectory buffer,
 * and configures the job program with command parameters.
 */
bool HdrRobotHardware::InitializeRobotOperation() {
  if (!driver_->PostRobotMotorPower().second) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to activate motor");
    return false;
  }

  if (!driver_->RelayExternalStopClear().second) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to clear external stop");
    return false;
  }

  if (!driver_->PostInitJointTrajectory().second) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to initialize joint trajectory");
    return false;
  }

  if (!driver_->SetJobProgram(pub_hz_)) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Failed to set job program (hz=%d)",
                 pub_hz_);
    return false;
  }

  return true;
}

/**
 * @return `true` on success, `false` on failure.
 *
 * @brief Sets up periodic state monitoring and controller management.
 *
 * Starts the executor thread for service callbacks and creates a wall timer
 * that periodically monitors robot state and manages controller activation/
 * deactivation based on playback mode, robot mode, and motor state.
 *
 * This method is exception-safe and will catch RCL errors that may occur
 * during shutdown when attempting to add nodes to the executor.
 */
bool HdrRobotHardware::SetupStateMonitoring() {
  try {
    exec_.add_node(node_for_services_);
    spin_thread_ = std::thread([this]() { exec_.spin(); });

    state_timer_ = node_for_services_->create_wall_timer(
        std::chrono::milliseconds(STATE_TIMER_PERIOD_MS), [this] { StateMonitoringCallback(); });

    return true;
  } catch (const rclcpp::exceptions::RCLError& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "RCL error during state monitoring setup (likely shutdown in progress): %s",
                 e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                 "Exception during state monitoring setup: %s", e.what());
    return false;
  }
}

/**
 * @brief Periodic callback for state monitoring and controller switching.
 *
 * Monitors robot state from rgen API and automatically manages controller
 * activation/deactivation. Activates the joint_trajectory_controller when
 * all conditions are met (playback active, REMOTE mode, motor on) and
 * deactivates it when any condition fails.
 */
void HdrRobotHardware::StateMonitoringCallback() {
  const auto [state, state_ok] = UpdateRobotState();
  if (!state_ok) {
    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("HdrRobotHardware"), *node_for_services_->get_clock(), 30000,
        "UpdateRobotState failed - Check robot connection and API status. Response: %s",
        state.dump().c_str());
    return;
  }

  const bool playback_ok = (is_playback_mode_ == 1);
  const bool mode_ok = (robot_mode_ == RobotMode::REMOTE);
  const bool motor_ok = (motor_state_ == 0);
  const bool all_ok = playback_ok && mode_ok && motor_ok;

  static bool prev_all_ok = false;
  static bool first_call = true;

  if (first_call) {
    first_call = false;
    if (all_ok && controller_active_) {
      prev_all_ok = all_ok;
      return;
    }
  }

  const bool changed = (prev_all_ok != all_ok);
  prev_all_ok = all_ok;

  if (!changed || switch_in_progress_)
    return;

  if (!switch_controller_client_ || !switch_controller_client_->wait_for_service(
                                        std::chrono::milliseconds(SERVICE_WAIT_TIMEOUT_MS))) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HdrRobotHardware"), *node_for_services_->get_clock(),
                         2000, "SwitchController service unavailable");
    return;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  req->activate_asap = true;
  req->timeout.sec = 0;
  req->timeout.nanosec = SWITCH_TIMEOUT_NANOSEC;

  switch_in_progress_ = true;

  if (!all_ok) {
    const char* reason = !playback_ok ? "Playback not active"
                         : !mode_ok   ? "Not in REMOTE mode"
                                      : "Motor off/busy";
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"), "Deactivating controllers: %s", reason);
    controller_active_ = false;
    req->deactivate_controllers = {"joint_trajectory_controller"};

    switch_controller_client_->async_send_request(
        req,
        [this](
            rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
          (void)future;
          switch_in_progress_ = false;
        });

  } else {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Activating controllers: conditions met");

    // Reinitialize trajectory buffer on controller reactivation
    if (!driver_->PostInitJointTrajectory().second) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                   "Failed to reinitialize joint trajectory");
    }

    position_commands_ = joint_positions_;
    position_commands_old_ = joint_positions_;
    controller_active_ = true;
    req->activate_controllers = {"joint_trajectory_controller"};

    switch_controller_client_->async_send_request(
        req,
        [this](
            rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
          try {
            auto res = future.get();
            if (!res->ok) {
              RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                          "Controller activation failed; rolling back");
              controller_active_ = false;
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                         "Activation callback exception: %s", e.what());
            controller_active_ = false;
          }
          switch_in_progress_ = false;
        });
  }
}

/**
 * @return `true` on success, `false` on failure.
 *
 * @brief Synchronizes initial joint data with robot.
 *
 * Attempts multiple retries to read initial joint data from the robot
 * and synchronizes all command buffers to prevent sudden movements on activation.
 */
bool HdrRobotHardware::SyncInitialPosition() {
  if (!first_pass_ || initialized_) {
    return true;
  }

  for (int attempt = 1; attempt <= MAX_POSITION_RETRIES; ++attempt) {
    auto joint_data = driver_->GetJointData();

    if (joint_data.IsValid()) {
      joint_positions_ = joint_data.positions;
      joint_velocities_ = joint_data.velocities;
      joint_efforts_ = joint_data.efforts;
      position_commands_ = joint_data.positions;
      position_commands_old_ = joint_data.positions;

      first_pass_ = false;
      initialized_ = true;

      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                  "First-pass init: command = current robot position (attempt %d/%d).", attempt,
                  MAX_POSITION_RETRIES);
      return true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                  "Initial joint data read failed or invalid (attempt %d/%d).", attempt,
                  MAX_POSITION_RETRIES);

      if (attempt < MAX_POSITION_RETRIES) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POSITION_RETRY_DELAY_MS));
      }
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
               "Failed to read initial joint data after %d attempts. Hardware activation failed.",
               MAX_POSITION_RETRIES);
  return false;
}

/**
 * @param[in] level The cleanup level to perform.
 *
 * @return `true` on success, `false` if any errors occurred (non-fatal).
 *
 * @brief Common cleanup routine for deactivation, cleanup, and shutdown.
 *
 * @details
 * This method performs resource cleanup operations based on the lifecycle
 * transition being performed:
 *
 * CleanupLevel::DEACTIVATE (active → inactive):
 * - Cancel state monitoring timer
 * - Cancel executor and join spin thread
 * - Keep polling/sending running for potential reactivation
 *
 * CleanupLevel::CLEANUP (inactive → unconfigured):
 * - All DEACTIVATE steps
 * - Stop sending thread
 * - Stop polling thread
 *
 * CleanupLevel::SHUTDOWN (any → finalized):
 * - Emergency stop with motor state verification
 * - All CLEANUP steps
 *
 * All operations are exception-safe with detailed logging.
 */
bool HdrRobotHardware::CleanupResources(CleanupLevel level) {
  if (!driver_ || !driver_initialized_) {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                "HDR driver is not initialized. Skipping cleanup steps.");
    return true;
  }

  bool cleanup_success = true;

  // Step 1: Send emergency stop to robot with retry (SHUTDOWN only)
  if (level == CleanupLevel::SHUTDOWN) {
    constexpr int max_estop_retries = 3;
    bool estop_sent = false;

    try {
      for (int attempt = 1; attempt <= max_estop_retries; ++attempt) {
        RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                    "Sending emergency stop to robot (attempt %d/%d)...", attempt,
                    max_estop_retries);

        auto [response, result] = driver_->PostRobotEmergencyStop();
        if (result) {
          RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Emergency stop sent successfully");
          estop_sent = true;
          break;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                      "Failed to send emergency stop (attempt %d): %s", attempt,
                      response.dump().c_str());
          if (attempt < max_estop_retries) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
          }
        }
      }

      if (!estop_sent) {
        RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                     "Failed to send emergency stop after %d attempts", max_estop_retries);
        cleanup_success = false;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Exception during emergency stop: %s",
                   e.what());
      cleanup_success = false;
    }

    // Step 2: Verify motor is actually OFF
    if (estop_sent) {
      constexpr int max_verify_retries = 5;
      bool motor_confirmed_off = false;

      try {
        RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                    "Verifying motor state after emergency stop...");

        for (int attempt = 1; attempt <= max_verify_retries; ++attempt) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          auto [motor_response, motor_ok] = driver_->GetRobotMotorState();
          if (motor_ok && motor_response.contains("state")) {
            int motor_state = motor_response["state"].get<int>();
            // state: 0=ON, 1=OFF, 2=BUSY
            if (motor_state == 1) {
              RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                          "Motor confirmed OFF after %d attempts", attempt);
              motor_confirmed_off = true;
              break;
            } else {
              RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                          "Motor state is %d (expected 1=OFF), retrying... (attempt %d/%d)",
                          motor_state, attempt, max_verify_retries);
            }
          } else {
            RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                        "Failed to get motor state, retrying... (attempt %d/%d)", attempt,
                        max_verify_retries);
          }
        }

        if (!motor_confirmed_off) {
          RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                       "Failed to confirm motor OFF state after %d attempts. "
                       "Motor may still be running!",
                       max_verify_retries);
          cleanup_success = false;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"),
                     "Exception during motor state verification: %s", e.what());
        cleanup_success = false;
      }
    }
  }

  // Step 3: Stop sending thread (CLEANUP and SHUTDOWN only)
  if (level == CleanupLevel::CLEANUP || level == CleanupLevel::SHUTDOWN) {
    try {
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Stopping sending thread...");
      driver_->StopSending();
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Sending stopped successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Exception stopping sending: %s",
                   e.what());
      cleanup_success = false;
    }

    // Step 4: Stop polling thread
    try {
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Stopping polling thread...");
      driver_->StopPolling();
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Polling stopped successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Exception stopping polling: %s",
                   e.what());
      cleanup_success = false;
    }
  }

  // Step 5: Cancel and cleanup state monitoring timer
  try {
    if (state_timer_) {
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Canceling state monitoring timer...");
      state_timer_->cancel();
      state_timer_.reset();
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Timer canceled successfully");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Exception canceling timer: %s", e.what());
    cleanup_success = false;
  }

  // Step 6: Cancel executor
  try {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Canceling executor...");
    exec_.cancel();
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Executor canceled");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Exception canceling executor: %s",
                 e.what());
    cleanup_success = false;
  }

  // Step 7: Join spin thread with timeout detection
  try {
    if (spin_thread_.joinable()) {
      RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Joining spin thread...");
      auto start = std::chrono::steady_clock::now();
      spin_thread_.join();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start);

      if (elapsed.count() > 1000) {
        RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                    "Spin thread took %ld ms to join (expected < 1000ms)", elapsed.count());
      } else {
        RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"),
                    "Spin thread joined successfully (%ld ms)", elapsed.count());
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("HdrRobotHardware"), "Exception joining spin thread: %s",
                 e.what());
    cleanup_success = false;
  }

  // Reset controller state
  controller_active_ = false;
  switch_in_progress_ = false;

  return cleanup_success;
}

// ══════════════════════════════════════════════════════════════════════════════
// Lifecycle — deactivation & shutdown
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @param[in] previous_state The previous state of the hardware interface.
 *
 * @return `CallbackReturn::SUCCESS` on success, otherwise an error code.
 *
 * @brief Deactivates the hardware interface.
 *
 * @details
 * This method is called when the controller is deactivated (active → inactive).
 * It stops state monitoring (timer/executor) but keeps polling and sending
 * active to allow for potential reactivation without full reconfiguration.
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Deactivating hardware...");

  bool success = CleanupResources(CleanupLevel::DEACTIVATE);

  if (success) {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Hardware deactivated successfully");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                "Hardware deactivated with some errors (see logs above)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;  // Allow transition even with errors
}

/**
 * @param[in] previous_state The previous state of the hardware interface.
 *
 * @return `CallbackReturn::SUCCESS` on success, otherwise an error code.
 *
 * @brief Cleans up the hardware interface.
 *
 * @details
 * This method is called when the controller is cleaned up (inactive → unconfigured).
 * It stops polling and sending threads, and cleans up driver resources, allowing
 * for full reconfiguration if needed.
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Cleaning up hardware...");

  bool success = CleanupResources(CleanupLevel::CLEANUP);

  if (success) {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Hardware cleaned up successfully");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                "Hardware cleaned up with some errors (see logs above)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;  // Allow transition even with errors
}

/**
 * @param[in] previous_state The previous state of the hardware interface.
 *
 * @return `CallbackReturn::SUCCESS` on success, otherwise an error code.
 *
 * @brief Shuts down the hardware interface.
 *
 * @details
 * This method is called when the controller is shut down (any state → finalized).
 * It performs complete cleanup including emergency stop to ensure safe robot state.
 */
hardware_interface::CallbackReturn HdrRobotHardware::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Shutting down hardware...");

  bool success = CleanupResources(CleanupLevel::SHUTDOWN);

  if (success) {
    RCLCPP_INFO(rclcpp::get_logger("HdrRobotHardware"), "Hardware shutdown successfully");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("HdrRobotHardware"),
                "Hardware shutdown with some errors (see logs above)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;  // Allow transition even with errors
}

// ══════════════════════════════════════════════════════════════════════════════
// Cyclic read / write
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @param[in] time The current time (currently unused).
 * @param[in] period The time since the last read (currently unused).
 *
 * @return `hardware_interface::return_type::OK` on success,
 *         `hardware_interface::return_type::ERROR` on failure.
 *
 * @brief Read joint data from the robot and update system state.
 *
 * Retrieves the latest joint data (positions, velocities, and efforts) from the driver's cache,
 * which is populated by the polling thread. Updates all joint state interfaces including
 * position, velocity, and effort data.
 */
hardware_interface::return_type HdrRobotHardware::read(const rclcpp::Time& /*time*/,
                                                       const rclcpp::Duration& /*period*/) {
  // Fast path: skip all checks if not initialized
  if (!driver_ || !driver_initialized_) {
    return hardware_interface::return_type::OK;
  }

  auto joint_data = driver_->GetJointData();

  if (joint_data.IsValid()) {
    // Update positions (assume size is correct for performance)
    if (joint_data.positions.size() == joint_positions_.size()) {
      std::copy(joint_data.positions.begin(), joint_data.positions.end(), joint_positions_.begin());
    }

    // Update velocities
    if (joint_data.velocities.size() == joint_velocities_.size()) {
      std::copy(joint_data.velocities.begin(), joint_data.velocities.end(),
                joint_velocities_.begin());
    }

    // Update efforts
    if (joint_data.efforts.size() == joint_efforts_.size()) {
      std::copy(joint_data.efforts.begin(), joint_data.efforts.end(), joint_efforts_.begin());
    }
  }
  return hardware_interface::return_type::OK;
}

/**
 * @param[in] time The current time (currently unused).
 * @param[in] period The time since the last write (currently unused).
 *
 * @return `hardware_interface::return_type::OK` on success,
 *         `hardware_interface::return_type::ERROR` on failure.
 *
 * @brief Write joint commands to the robot with intelligent filtering.
 *
 * Checks the controller_active_ flag set by StateMonitoringCallback to determine
 * if the robot is in a safe operational state. When safe, implements smart command
 * filtering to reduce unnecessary network traffic.
 *
 * Constants:
 * - POSITION_EPSILON (1e-3): Threshold for detecting robot movement
 * - COMMAND_EPSILON (1e-3): Threshold for detecting command changes
 *
 * Look-ahead time calculation:
 * - look_ahead_time = (1/hz) * buffer_size (in seconds)
 * - Example: at 100Hz with buffer_size=5 → 0.01 * 5 = 0.05 seconds
 */
hardware_interface::return_type HdrRobotHardware::write(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/) {
  // Fast path: skip if not initialized
  if (!driver_ || !driver_initialized_) {
    return hardware_interface::return_type::ERROR;
  }

  // Check controller_active_ flag set by StateMonitoringCallback
  // This flag reflects: playback active, REMOTE mode, and motor ON
  if (!controller_active_) {
    position_commands_ = joint_positions_;
    position_commands_old_ = joint_positions_;
    return hardware_interface::return_type::OK;
  }

  // Robot is in safe operational state - send commands
  {
    // Combined loop: check both is_moving and is_same_command in one pass
    bool is_moving = false;
    bool is_same_command = true;

    for (size_t i = 0; i < position_commands_.size(); ++i) {
      if (std::abs(position_commands_[i] - joint_positions_[i]) > POSITION_EPSILON) {
        is_moving = true;
      }
      if (std::abs(position_commands_[i] - position_commands_old_[i]) > COMMAND_EPSILON) {
        is_same_command = false;
      }
      // Early exit if both conditions met
      if (is_moving && !is_same_command) {
        break;
      }
    }

    // Skip sending if robot not moving and command unchanged
    if (!is_moving && is_same_command) {
      return hardware_interface::return_type::OK;
    }

    // Calculate look_ahead_time: (1/hz) * buffer_size
    double look_ahead_time = (1.0 / static_cast<double>(pub_hz_)) * command_buffer_size_;

    // Async send (queued, non-blocking)
    driver_->SetRobotPosition(position_commands_, pub_hz_, look_ahead_time);

    position_commands_old_ = position_commands_;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hdr_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hdr_hardware_interface::HdrRobotHardware,
                       hardware_interface::SystemInterface)
