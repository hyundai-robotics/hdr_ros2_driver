#pragma once

#include <iostream>
#include <regex>
#include <unordered_set>
#include <set>
#include <map>
#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "api_msgs/srv/file_path.hpp"
#include "api_msgs/srv/file_rename.hpp"
#include "api_msgs/srv/file_list.hpp"
#include "api_msgs/srv/file_send.hpp"
#include "api_msgs/srv/file_path.hpp"

#include "api_msgs/srv/number.hpp"
#include "api_msgs/srv/io_request.hpp"
#include "api_msgs/srv/ioplc_get.hpp"
#include "api_msgs/srv/ioplc_post.hpp"
#include "api_msgs/srv/op_cnd.hpp"
#include "api_msgs/srv/pose_cur.hpp"
#include "api_msgs/srv/program_cnt.hpp"
#include "api_msgs/srv/program_var.hpp"
#include "api_msgs/srv/emergency.hpp"
#include "api_msgs/srv/execute_move.hpp"
#include "api_msgs/srv/execute_cmd.hpp"

#include "api_msgs/srv/date_time.hpp"
#include "api_msgs/srv/log_manager.hpp"

#include "hr_ros2_driver/Requestlib.hpp"

namespace version
{
    void api_version(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void sysver_version(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
}

namespace project
{
    void project_get_rgen(const std::shared_ptr<RequestLib> &client,
                          const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void project_get_jobs_info(const std::shared_ptr<RequestLib> &client,
                               const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void project_post_reload_update_jobs(const std::shared_ptr<RequestLib> &client,
                                         const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void project_post_delete_job(const std::shared_ptr<RequestLib> &client,
                                 const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                                 std::shared_ptr<api_msgs::srv::FilePath::Response> response);
}

namespace control
{

    void control_op_cnd(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void control_ucs_nos(const std::shared_ptr<RequestLib> &client,
                         const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void control_ios_di(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<api_msgs::srv::IoRequest::Request> request,
                        std::shared_ptr<api_msgs::srv::IoRequest::Response> response);

    void control_ios_do(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<api_msgs::srv::IoRequest::Request> request,
                        std::shared_ptr<api_msgs::srv::IoRequest::Response> response);

    void control_ios_si(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<api_msgs::srv::IoRequest::Request> request,
                        std::shared_ptr<api_msgs::srv::IoRequest::Response> response);

    void control_ios_so(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<api_msgs::srv::IoRequest::Request> request,
                        std::shared_ptr<api_msgs::srv::IoRequest::Response> response);

    void control_post_ios_do(const std::shared_ptr<RequestLib> &client,
                             const std::shared_ptr<api_msgs::srv::IoRequest::Request> request,
                             std::shared_ptr<api_msgs::srv::IoRequest::Response> response);

    void control_put_op_cnd(const std::shared_ptr<RequestLib> &client,
                            const std::shared_ptr<api_msgs::srv::OpCnd::Request> request,
                            std::shared_ptr<api_msgs::srv::OpCnd::Response> response);
}

namespace robot
{
    void robot_motor_state(const std::shared_ptr<RequestLib> &client,
                           const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void robot_cur_tool(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void robot_tools(const std::shared_ptr<RequestLib> &client,
                     const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void robot_tools_t(const std::shared_ptr<RequestLib> &client,
                       const std::shared_ptr<api_msgs::srv::Number::Request> request,
                       std::shared_ptr<api_msgs::srv::Number::Response> response);

    void robot_tool_no(const std::shared_ptr<RequestLib> &client,
                       const std::shared_ptr<api_msgs::srv::Number::Request> request,
                       std::shared_ptr<api_msgs::srv::Number::Response> response);

    void robot_crd_sys(const std::shared_ptr<RequestLib> &client,
                       const std::shared_ptr<api_msgs::srv::Number::Request> request,
                       std::shared_ptr<api_msgs::srv::Number::Response> response);

    void robot_motor_on_off(const std::shared_ptr<RequestLib> &client,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void robot_on_off(const std::shared_ptr<RequestLib> &client,
                      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void robot_po_cur(const std::shared_ptr<RequestLib>& client,
                  const std::shared_ptr<api_msgs::srv::PoseCur::Request> request,
                  std::shared_ptr<api_msgs::srv::PoseCur::Response> response);

    std::tuple<nlohmann::json, int> robot_po_cur_timer(const std::shared_ptr<RequestLib>& client, std::vector<int64_t>& pose_param);
    
    void robot_emergency_stop(const std::shared_ptr<RequestLib>& client,
                  const std::shared_ptr<api_msgs::srv::Emergency::Request> request,
                  std::shared_ptr<api_msgs::srv::Emergency::Response> response);
                        
}

namespace etc
{
    void etc_date_time(const std::shared_ptr<RequestLib> &client,
                       const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void etc_put_date_time(const std::shared_ptr<RequestLib>& client,
                       const std::shared_ptr<api_msgs::srv::DateTime::Request> request,
                       std::shared_ptr<api_msgs::srv::DateTime::Response> response);

    void etc_get_log_manager(const std::shared_ptr<RequestLib>& client,
                         const std::shared_ptr<api_msgs::srv::LogManager::Request> request,
                         std::shared_ptr<api_msgs::srv::LogManager::Response> response);
}

namespace task
{
    void task_post_cur_prog_cnt(const std::shared_ptr<RequestLib> &client,
                                const std::shared_ptr<api_msgs::srv::ProgramCnt::Request> request,
                                std::shared_ptr<api_msgs::srv::ProgramCnt::Response> response);

    void task_post_set_cur_pc_idx(const std::shared_ptr<RequestLib> &client,
                                  const std::shared_ptr<api_msgs::srv::Number::Request> request,
                                  std::shared_ptr<api_msgs::srv::Number::Response> response);

    void task_post_release_wait(const std::shared_ptr<RequestLib> &client,
                                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void task_post_reset(const std::shared_ptr<RequestLib> &client,
                         const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void task_post_reset_t(const std::shared_ptr<RequestLib> &client,
                           const std::shared_ptr<api_msgs::srv::Number::Request> request,
                           std::shared_ptr<api_msgs::srv::Number::Response> response);

    void task_post_assign_var(const std::shared_ptr<RequestLib> &client,
                              const std::shared_ptr<api_msgs::srv::ProgramVar::Request> request,
                              std::shared_ptr<api_msgs::srv::ProgramVar::Response> response);
    
    void task_post_solve_expr(const std::shared_ptr<RequestLib>& client,
                          const std::shared_ptr<api_msgs::srv::ProgramVar::Request> request,
                          std::shared_ptr<api_msgs::srv::ProgramVar::Response> response);

    void task_post_execute_move(const std::shared_ptr<RequestLib>& client,
                            const std::shared_ptr<api_msgs::srv::ExecuteMove::Request> request,
                            std::shared_ptr<api_msgs::srv::ExecuteMove::Response> response);
}

namespace file_manager
{
    void file_get_files(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                        std::shared_ptr<api_msgs::srv::FilePath::Response> response);

    void file_get_info(const std::shared_ptr<RequestLib> &client,
                       const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                       std::shared_ptr<api_msgs::srv::FilePath::Response> response);

    void file_get_exist(const std::shared_ptr<RequestLib> &client,
                        const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                        std::shared_ptr<api_msgs::srv::FilePath::Response> response);

    void file_post_mkdir(const std::shared_ptr<RequestLib> &client,
                         const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                         std::shared_ptr<api_msgs::srv::FilePath::Response> response);

    void file_post_delete(const std::shared_ptr<RequestLib> &client,
                          const std::shared_ptr<api_msgs::srv::FilePath::Request> request,
                          std::shared_ptr<api_msgs::srv::FilePath::Response> response);

    void file_get_list(const std::shared_ptr<RequestLib>& client,
                   const std::shared_ptr<api_msgs::srv::FileList::Request> request,
                   std::shared_ptr<api_msgs::srv::FileList::Response> response);

    void file_post_rename_file(const std::shared_ptr<RequestLib>& client,
                           const std::shared_ptr<api_msgs::srv::FileRename::Request> request,
                           std::shared_ptr<api_msgs::srv::FileRename::Response> response);
    
    void file_post_files(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<api_msgs::srv::FileSend::Request> request,
                     std::shared_ptr<api_msgs::srv::FileSend::Response> response);
}

namespace io_plc
{
    void get_relay_value(const std::shared_ptr<RequestLib>& client,
                     const std::shared_ptr<api_msgs::srv::IoplcGet::Request> request,
                     std::shared_ptr<api_msgs::srv::IoplcGet::Response> response);

    void post_relay_value(const std::shared_ptr<RequestLib>& client,
                      const std::shared_ptr<api_msgs::srv::IoplcPost::Request> request,
                      std::shared_ptr<api_msgs::srv::IoplcPost::Response> response);
}

namespace console
{
    int post_execute_cmd(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<api_msgs::srv::ExecuteCmd::Request> request,
                    std::shared_ptr<api_msgs::srv::ExecuteCmd::Response> response);
}

#undef DECLARE_SERVICE_FUNCTION