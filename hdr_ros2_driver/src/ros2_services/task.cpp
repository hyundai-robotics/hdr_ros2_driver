#include "hdr_ros2_driver/service_manager.hpp"

/**
 * @param request The request object containing the program count details.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to post the current program count.
 *
 * @details
 * This service updates the current program count in the robot controller. It handles
 * program number, step number, function number, and external selection as input.
 * 🔗 API Reference:
 * [Post Current Program
 * Count](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/1-cur_prog_cnt)
 *
 */
void ServiceManager::HandlePostCurProgCnt(
    const std::shared_ptr<hdr_msgs::srv::ProgramCnt::Request> request,
    std::shared_ptr<hdr_msgs::srv::ProgramCnt::Response> response) {
  try {
    auto [result, success] =
        driver_->PostCurProgCnt(request->pno, request->sno, request->fno, request->ext_sel);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post current program count: %s", e.what());
  }
}

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to reset the robot controller.
 *
 * @details
 * This service resets the robot controller to its initial state, effectively restarting it.
 * 🔗 API Reference:
 * [Post Reset](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/2-reset)
 *
 */
void ServiceManager::HandlePostReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->PostReset();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post reset: %s", e.what());
  }
}

/**
 * @param request The request object containing the variable details (name, scope, expression, save
 * flag).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to assign a variable for the task.
 *
 * @details
 * This service assigns a variable to the task in the robot controller using the provided
 * expression.
 * 🔗 API Reference:
 * [Post Assign Variable
 * Expression](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/3-assign_var_expr)
 * [Post Assign Variable Json]
 * (https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/4-assign_var_json)
 *
 */
void ServiceManager::HandlePostAssignVar(
    const std::shared_ptr<hdr_msgs::srv::ProgramVar::Request> request,
    std::shared_ptr<hdr_msgs::srv::ProgramVar::Response> response) {
  try {
    auto [result, success] =
        driver_->PostAssignVar(request->name, request->scope, request->expr, request->save);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post assign var: %s", e.what());
  }
}

/**
 * @param request The request object (empty for this service).
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to release wait condition.
 *
 * @details
 * This service releases the wait condition in the robot controller and resumes operations.
 * 🔗 API Reference:
 * [Post Release
 * Wait](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/5-release_wait)
 *
 */
void ServiceManager::HandlePostReleaseWait(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  try {
    auto [result, success] = driver_->PostReleaseWait();
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post release wait: %s", e.what());
  }
}

/**
 * @param request The request object containing the PC index value.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to set the current PC index.
 *
 * @details
 * This service updates the current PC (program counter) index in the robot controller.
 * 🔗 API Reference:
 * [Post Set Current PC
 * Index](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/6-set_cur_pc_idx)
 *
 */
void ServiceManager::HandlePostSetCurPcIdx(
    const std::shared_ptr<hdr_msgs::srv::Number::Request> request,
    std::shared_ptr<hdr_msgs::srv::Number::Response> response) {
  try {
    auto [result, success] = driver_->PostSetCurPcIdx(request->data);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to set current PC index: %s", e.what());
  }
}

/**
 * @param request The request object containing the scope and expression to solve.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to solve an expression.
 *
 * @details
 * This service solves a mathematical expression on the robot controller based on the given scope
 * and expression.
 * 🔗 API Reference:
 * [Post Solve
 * Expression](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/7-solve_expr)
 *
 */
void ServiceManager::HandlePostSolveExpr(
    const std::shared_ptr<hdr_msgs::srv::ProgramVar::Request> request,
    std::shared_ptr<hdr_msgs::srv::ProgramVar::Response> response) {
  try {
    auto [result, success] = driver_->PostSolveExpr(request->scope, request->expr);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post solve expression: %s", e.what());
  }
}

/**
 * @param request The request object containing the statement and task number for the move
 * execution.
 * @param response The response object with success status and message.
 *
 * @brief Handles the request to execute a move command.
 *
 * @details
 * This service sends an execute move command to the robot controller, which will process and
 * execute the move.
 * 🔗 API Reference:
 * [Post Execute
 * Move](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/8-execute_move)
 *
 */
void ServiceManager::HandlePostExecuteMove(
    const std::shared_ptr<hdr_msgs::srv::ExecuteMove::Request> request,
    std::shared_ptr<hdr_msgs::srv::ExecuteMove::Response> response) {
  try {
    auto [result, success] = driver_->PostExecuteMove(request->stmt, request->task_no);
    response->success = success;
    response->message = result.dump();
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Failed to post execute move: %s", e.what());
  }
}