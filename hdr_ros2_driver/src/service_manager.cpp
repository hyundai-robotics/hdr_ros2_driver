#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @brief Defines the ServiceManager class which maps ROS2 services to HdrDriver API calls.
 *
 * @file service_manager.cpp
 */

/**
 * @param node Pointer to the ROS2 node used for service registration
 * @param driver Reference to the HdrDriver instance for API calls
 *
 * @brief Constructor for ServiceManager.
 *
 */
ServiceManager::ServiceManager(rclcpp::Node* node, hdrcl::HdrDriver* driver)
    : node_(node), driver_(driver) {
  // Initialize the service manager
  RCLCPP_INFO(node_->get_logger(), "ServiceManager initialized.");
}
/**
 * @brief Sets up all ROS2 services and binds them to their respective handler callbacks.
 * Services are grouped by logical modules:
 * - Version info
 * - Project configuration
 * - Robot control
 * - IO PLC
 * - System time/log manager
 * - File manager
 * - Task execution
 * - Console interface
 *
 */
void ServiceManager::SetupAllServices() {
  // ------------------ Version Services ------------------
  SetupService<std_srvs::srv::Trigger>(service_names::version::kApi,
                                       &ServiceManager::HandleGetApiVersion);
  SetupService<std_srvs::srv::Trigger>(service_names::version::kSystem,
                                       &ServiceManager::HandleGetSystemVersion);

  // ------------------ Project Services ------------------
  SetupService<std_srvs::srv::Trigger>(service_names::project::kRgen,
                                       &ServiceManager::HandleGetProjectRgen);
  SetupService<std_srvs::srv::Trigger>(service_names::project::kJobsInfo,
                                       &ServiceManager::HandleGetProjectJobsInfo);
  SetupService<std_srvs::srv::Trigger>(service_names::project::kReload,
                                       &ServiceManager::HandlePostProjectReloadUpdateJobs);
  SetupService<hdr_msgs::srv::FilePath>(service_names::project::kDeleteJob,
                                        &ServiceManager::HandlePostProjectDeleteJob);

  // ------------------ Control Services ------------------
  SetupService<std_srvs::srv::Trigger>(service_names::control::kOpCnd,
                                       &ServiceManager::HandleGetControlOpCnd);
  SetupService<hdr_msgs::srv::IoRequest>(service_names::control::kIosDio,
                                         &ServiceManager::HandleGetControlIosDio);
  SetupService<hdr_msgs::srv::IoRequest>(service_names::control::kIosSio,
                                         &ServiceManager::HandleGetControlIosSio);
  SetupService<std_srvs::srv::Trigger>(service_names::control::kUcsNos,
                                       &ServiceManager::HandleGetControlUcsNos);
  SetupService<hdr_msgs::srv::IoRequest>(service_names::control::kPostIosDo,
                                         &ServiceManager::HandlePostControlIosDio);
  SetupService<hdr_msgs::srv::OpCnd>(service_names::control::kPutOpCnd,
                                     &ServiceManager::HandlePutControlOpCnd);

  // ------------------ Robot Services ------------------
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kMotorState,
                                       &ServiceManager::HandleGetRobotMotorState);
  SetupService<hdr_msgs::srv::PoseCur>(service_names::robot::kPoCur,
                                       &ServiceManager::HandleGetRobotPoCur);
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kCurTool,
                                       &ServiceManager::HandleGetRobotCurTool);
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kTools,
                                       &ServiceManager::HandleGetRobotTools);
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kEmergency,
                                       &ServiceManager::HandleGetRobotEmergency);
  SetupService<hdr_msgs::srv::Number>(service_names::robot::kToolsT,
                                      &ServiceManager::HandleGetRobotToolsT);
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kMotorPower,
                                       &ServiceManager::HandlePostRobotMotorPower);
  SetupService<std_srvs::srv::SetBool>(service_names::robot::kOperation,
                                       &ServiceManager::HandlePostRobotOperation);
  SetupService<hdr_msgs::srv::Number>(service_names::robot::kToolNo,
                                      &ServiceManager::HandlePostRobotToolNo);
  SetupService<hdr_msgs::srv::Number>(service_names::robot::kCrdSys,
                                      &ServiceManager::HandlePostRobotCrdSys);
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kEmergencyStop,
                                       &ServiceManager::HandlePostRobotEmergencyStop);
  SetupService<hdr_msgs::srv::Emergency>(service_names::robot::kEmergencyStopTest,
                                         &ServiceManager::HandlePostRobotEmergencyStopTest);
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kJointTrajBuffAvail,
                                       &ServiceManager::HandleGetJointTrajBuffAvail);
  SetupService<std_srvs::srv::Trigger>(service_names::robot::kInitJointTrajectory,
                                       &ServiceManager::HandlePostInitJointTrajectory);
  SetupService<hdr_msgs::srv::JointTrajectoryPoints>(
      service_names::robot::kInsertJointTrajectoryPoints,
      &ServiceManager::HandlePostInsertJointTrajectoryPoints);

  // ------------------ IO PLC Services ------------------
  SetupService<hdr_msgs::srv::IoplcGet>(service_names::io_plc::kGetRelayValue,
                                        &ServiceManager::HandleGetRelayValue);
  SetupService<hdr_msgs::srv::IoplcPost>(service_names::io_plc::kPostRelayValue,
                                         &ServiceManager::HandlePostRelayValue);

  // ------------------ ETC Services ------------------
  SetupService<std_srvs::srv::Trigger>(service_names::etc::kGetDateTime,
                                       &ServiceManager::HandleGetDateTime);
  SetupService<hdr_msgs::srv::DateTime>(service_names::etc::kPutDateTime,
                                        &ServiceManager::HandlePutDateTime);
  SetupService<hdr_msgs::srv::LogManager>(service_names::etc::kGetLogManager,
                                          &ServiceManager::HandleGetLogManager);

  // ------------------ File Manager Services ------------------
  SetupService<hdr_msgs::srv::FilePath>(service_names::file_manager::kGetFiles,
                                        &ServiceManager::HandleGetFiles);
  SetupService<hdr_msgs::srv::FilePath>(service_names::file_manager::kGetFileInfo,
                                        &ServiceManager::HandleGetFileInfo);
  SetupService<hdr_msgs::srv::FileList>(service_names::file_manager::kGetFileList,
                                        &ServiceManager::HandleGetFileList);
  SetupService<hdr_msgs::srv::FilePath>(service_names::file_manager::kGetFileExist,
                                        &ServiceManager::HandleGetFileExist);
  SetupService<hdr_msgs::srv::FileRename>(service_names::file_manager::kPostRenameFile,
                                          &ServiceManager::HandlePostRenameFile);
  SetupService<hdr_msgs::srv::FilePath>(service_names::file_manager::kPostMkdir,
                                        &ServiceManager::HandlePostMkdir);
  SetupService<hdr_msgs::srv::FileSend>(service_names::file_manager::kPostFiles,
                                        &ServiceManager::HandlePostFiles);
  SetupService<hdr_msgs::srv::FilePath>(service_names::file_manager::kPostDeleteFile,
                                        &ServiceManager::HandlePostDeleteFile);

  // ------------------ Task Services ------------------
  // SetupService<hdr_msgs::srv::ProgramCnt>(service_names::task::kPostCurProgCnt,
  //                                         &ServiceManager::HandlePostCurProgCnt);
  SetupService<std_srvs::srv::Trigger>(service_names::task::kPostReset,
                                       &ServiceManager::HandlePostReset);
  SetupService<hdr_msgs::srv::ProgramVar>(service_names::task::kPostAssignVar,
                                          &ServiceManager::HandlePostAssignVar);
  SetupService<std_srvs::srv::Trigger>(service_names::task::kPostReleaseWait,
                                       &ServiceManager::HandlePostReleaseWait);
  SetupService<hdr_msgs::srv::Number>(service_names::task::kPostSetCurPcIdx,
                                      &ServiceManager::HandlePostSetCurPcIdx);
  SetupService<hdr_msgs::srv::ProgramVar>(service_names::task::kPostSolveExpr,
                                          &ServiceManager::HandlePostSolveExpr);
  SetupService<hdr_msgs::srv::ExecuteMove>(service_names::task::kPostExecuteMove,
                                           &ServiceManager::HandlePostExecuteMove);

  // ------------------ Console Services ------------------
  SetupService<hdr_msgs::srv::ExecuteCmd>(service_names::console::kPostExecuteCmd,
                                          &ServiceManager::HandlePostExecuteCmd);
}
