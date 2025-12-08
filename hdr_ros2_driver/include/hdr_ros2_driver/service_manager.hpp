#ifndef HDR_ROS2_DRIVER_SERVICE_MANAGER_HPP_
#define HDR_ROS2_DRIVER_SERVICE_MANAGER_HPP_

// Common/Core headers
#include "hdr_client_driver/hdr_client_driver.h"
#include "rclcpp/rclcpp.hpp"

// Standard ROS2 services
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// Basic types
#include "hdr_msgs/srv/number.hpp"

// Control related
#include "hdr_msgs/srv/io_request.hpp"
#include "hdr_msgs/srv/op_cnd.hpp"

// Robot related
#include "hdr_msgs/srv/emergency.hpp"
#include "hdr_msgs/srv/joint_trajectory_points.hpp"
#include "hdr_msgs/srv/pose_cur.hpp"

// IO PLC related
#include "hdr_msgs/srv/ioplc_get.hpp"
#include "hdr_msgs/srv/ioplc_post.hpp"

// System/ETC related
#include "hdr_msgs/srv/date_time.hpp"
#include "hdr_msgs/srv/log_manager.hpp"

// File management related
#include "hdr_msgs/srv/file_list.hpp"
#include "hdr_msgs/srv/file_path.hpp"
#include "hdr_msgs/srv/file_rename.hpp"
#include "hdr_msgs/srv/file_send.hpp"

// Program/Task related
#include "hdr_msgs/srv/execute_cmd.hpp"
#include "hdr_msgs/srv/execute_move.hpp"
#include "hdr_msgs/srv/program_cnt.hpp"
#include "hdr_msgs/srv/program_var.hpp"

/**
 * @brief Contains constants for ROS2 service name strings grouped by domain.
 *
 */
namespace service_names {
namespace version {
constexpr auto kApi = "~/get/api_ver";
constexpr auto kSystem = "~/get/system_ver";
}  // namespace version

namespace project {
constexpr auto kRgen = "~/project/get/rgen";
constexpr auto kJobsInfo = "~/project/get/jobs_info";
constexpr auto kReload = "~/project/post/reload_updated_jobs";
constexpr auto kDeleteJob = "~/project/post/delete_job";
}  // namespace project

namespace control {
constexpr auto kOpCnd = "~/control/get/op_cnd";
constexpr auto kIosDio = "~/control/get/ios/dio";
constexpr auto kIosSio = "~/control/get/ios/sio";
constexpr auto kUcsNos = "~/control/get/ucs_nos";
constexpr auto kPostIosDo = "~/control/post/ios/dio";
constexpr auto kPutOpCnd = "~/control/put/op_cnd";
}  // namespace control

namespace robot {
constexpr auto kMotorState = "~/robot/get/motor_state";
constexpr auto kCurTool = "~/robot/get/cur_tool";
constexpr auto kTools = "~/robot/get/tools";
constexpr auto kToolsT = "~/robot/get/tools_t";
constexpr auto kEmergency = "~/robot/get/emergency";
constexpr auto kToolNo = "~/robot/post/tool_no";
constexpr auto kCrdSys = "~/robot/post/crd_sys";
constexpr auto kMotorPower = "~/robot/post/motor_power";
constexpr auto kOperation = "~/robot/post/operation";
constexpr auto kPoCur = "~/robot/get/po_cur";
constexpr auto kEmergencyStop = "~/robot/post/emergency_stop";
constexpr auto kEmergencyStopTest = "~/robot/post/emergency_stop_test";
constexpr auto kJointTrajBuffAvail = "~/robot/get/joint_traj_buff_avail";
constexpr auto kInitJointTrajectory = "~/robot/post/init_joint_trajectory";
constexpr auto kInsertJointTrajectoryPoints = "~/robot/post/insert_joint_trajectory_points";
}  // namespace robot

namespace io_plc {
constexpr auto kGetRelayValue = "~/plc/get/relay_value";
constexpr auto kPostRelayValue = "~/plc/post/relay_value";
}  // namespace io_plc

namespace etc {
constexpr auto kGetDateTime = "~/clock/get/date_time";
constexpr auto kPutDateTime = "~/clock/put/date_time";
constexpr auto kGetLogManager = "~/log/get/manager";
}  // namespace etc

namespace file_manager {
constexpr auto kGetFiles = "~/file/get/files";
constexpr auto kGetFileInfo = "~/file/get/file_info";
constexpr auto kGetFileList = "~/file/get/file_list";
constexpr auto kGetFileExist = "~/file/get/file_exist";
constexpr auto kPostMkdir = "~/file/post/mkdir";
constexpr auto kPostDeleteFile = "~/file/delete/files";
constexpr auto kPostRenameFile = "~/file/post/rename_file";
constexpr auto kPostFiles = "~/file/post/files";
}  // namespace file_manager

namespace task {
constexpr auto kPostCurProgCnt = "~/task/post/cur_prog_cnt";
constexpr auto kPostSetCurPcIdx = "~/task/post/set_cur_pc_idx";
constexpr auto kPostReleaseWait = "~/task/post/release_wait";
constexpr auto kPostReset = "~/task/post/reset";
constexpr auto kPostAssignVar = "~/task/post/assign_var";
constexpr auto kPostSolveExpr = "~/task/post/solve_expr";
constexpr auto kPostExecuteMove = "~/task/post/execute_move";
}  // namespace task

namespace console {
constexpr auto kPostExecuteCmd = "~/console/post/execute_cmd";
constexpr auto kPostRemoteOperation = "~/console/post/operation";
}  // namespace console
}  // namespace service_names

/**
 * @brief Manages all ROS2 service bindings for HdrDriver API calls.
 * Registers services with ROS2, binds them to internal handler functions,
 * and safely invokes driver logic in response to service requests.
 *
 * @class ServiceManager
 *
 */
class ServiceManager {
 public:
  /**
   * @param node ROS2 node for service registration
   * @param driver HdrDriver instance to bind to service callbacks
   *
   * @brief Construct a new Service Manager object
   *
   */
  ServiceManager(rclcpp::Node* node, hdrcl::HdrDriver* driver);

  // Default destructor
  ~ServiceManager() = default;

  // Defines function pointer type for service handlers
  template <typename T>
  using HandlerFunction = void (ServiceManager::*)(const std::shared_ptr<typename T::Request>,
                                                   std::shared_ptr<typename T::Response>);

  /**
   * @param name Service name to register (e.g. "~/robot/get/motor_state")
   * @param handler Member function to call on service invocation
   * @tparam T ROS2 service type
   *
   * @brief Registers a single ROS2 service with a specified handler
   *
   */
  template <typename T>
  void SetupService(const std::string& name, HandlerFunction<T> handler) {
    auto callback = [this, handler](const std::shared_ptr<typename T::Request> request,
                                    std::shared_ptr<typename T::Response> response) {
      try {
        (this->*handler)(request, response);
      } catch (...) {
        try {
          response->success = false;
          response->message = "Service timeout - robot busy";
        } catch (...) {
        }
      }
    };

    services_.push_back(node_->create_service<T>(name, callback));
  }

  /**
   * @brief Registers all HDR-related services to the ROS2 node
   *
   */
  void SetupAllServices();

 protected:
// Macro to declare handler functions for each service type
#define DECLARE_HANDLER(name, type) \
  void Handle##name(const std::shared_ptr<type::Request>, std::shared_ptr<type::Response>)

  // === Declare all service handler functions ===
  // Version handlers
  DECLARE_HANDLER(GetApiVersion, std_srvs::srv::Trigger);
  DECLARE_HANDLER(GetSystemVersion, std_srvs::srv::Trigger);

  // Project handlers
  DECLARE_HANDLER(GetProjectRgen, std_srvs::srv::Trigger);
  DECLARE_HANDLER(GetProjectJobsInfo, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostProjectReloadUpdateJobs, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostProjectDeleteJob, hdr_msgs::srv::FilePath);

  // Control handlers
  DECLARE_HANDLER(GetControlOpCnd, std_srvs::srv::Trigger);
  DECLARE_HANDLER(GetControlUcsNos, std_srvs::srv::Trigger);
  DECLARE_HANDLER(GetControlIosDio, hdr_msgs::srv::IoRequest);
  DECLARE_HANDLER(GetControlIosSio, hdr_msgs::srv::IoRequest);
  DECLARE_HANDLER(PostControlIosDio, hdr_msgs::srv::IoRequest);
  DECLARE_HANDLER(PutControlOpCnd, hdr_msgs::srv::OpCnd);

  // Robot handlers
  DECLARE_HANDLER(GetRobotMotorState, std_srvs::srv::Trigger);
  DECLARE_HANDLER(GetRobotCurTool, std_srvs::srv::Trigger);
  DECLARE_HANDLER(GetRobotTools, std_srvs::srv::Trigger);
  DECLARE_HANDLER(GetRobotToolsT, hdr_msgs::srv::Number);
  DECLARE_HANDLER(GetRobotEmergency, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostRobotToolNo, hdr_msgs::srv::Number);
  DECLARE_HANDLER(PostRobotCrdSys, hdr_msgs::srv::Number);
  DECLARE_HANDLER(PostRobotMotorPower, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostRobotOperation, std_srvs::srv::SetBool);
  DECLARE_HANDLER(GetRobotPoCur, hdr_msgs::srv::PoseCur);
  DECLARE_HANDLER(PostRobotEmergencyStop, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostRobotEmergencyStopTest, hdr_msgs::srv::Emergency);
  DECLARE_HANDLER(GetJointTrajBuffAvail, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostInitJointTrajectory, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostInsertJointTrajectoryPoints, hdr_msgs::srv::JointTrajectoryPoints);

  // IO PLC handlers
  DECLARE_HANDLER(GetRelayValue, hdr_msgs::srv::IoplcGet);
  DECLARE_HANDLER(PostRelayValue, hdr_msgs::srv::IoplcPost);

  // ETC handlers
  DECLARE_HANDLER(GetDateTime, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PutDateTime, hdr_msgs::srv::DateTime);
  DECLARE_HANDLER(GetLogManager, hdr_msgs::srv::LogManager);

  // File manager handlers
  DECLARE_HANDLER(GetFiles, hdr_msgs::srv::FilePath);
  DECLARE_HANDLER(GetFileInfo, hdr_msgs::srv::FilePath);
  DECLARE_HANDLER(GetFileExist, hdr_msgs::srv::FilePath);
  DECLARE_HANDLER(PostMkdir, hdr_msgs::srv::FilePath);
  DECLARE_HANDLER(PostDeleteFile, hdr_msgs::srv::FilePath);
  DECLARE_HANDLER(GetFileList, hdr_msgs::srv::FileList);
  DECLARE_HANDLER(PostRenameFile, hdr_msgs::srv::FileRename);
  DECLARE_HANDLER(PostFiles, hdr_msgs::srv::FileSend);

  // Task handlers
  DECLARE_HANDLER(PostCurProgCnt, hdr_msgs::srv::ProgramCnt);
  DECLARE_HANDLER(PostSetCurPcIdx, hdr_msgs::srv::Number);
  DECLARE_HANDLER(PostReleaseWait, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostReset, std_srvs::srv::Trigger);
  DECLARE_HANDLER(PostAssignVar, hdr_msgs::srv::ProgramVar);
  DECLARE_HANDLER(PostSolveExpr, hdr_msgs::srv::ProgramVar);
  DECLARE_HANDLER(PostExecuteMove, hdr_msgs::srv::ExecuteMove);

  // Console handlers
  DECLARE_HANDLER(PostExecuteCmd, hdr_msgs::srv::ExecuteCmd);

#undef DECLARE_HANDLER

 private:
  // ROS2 node used to register services
  rclcpp::Node* node_;
  // The HdrDriver interface to interact with OpenAPI
  hdrcl::HdrDriver* driver_;

  // List of registered services
  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_;
};

#endif  // HDR_ROS2_DRIVER_SERVICE_MANAGER_HPP_
