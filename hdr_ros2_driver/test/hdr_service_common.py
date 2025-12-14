#!/usr/bin/env python3
import os
from datetime import datetime
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

service_type_mapping = {
    "version": [
        ("/hdr_ros2_driver/get/api_ver", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/get/system_ver", "std_srvs/srv/Trigger"),
    ],
    "etc": [
        ("/hdr_ros2_driver/clock/get/date_time", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/clock/put/date_time", "hdr_msgs/srv/DateTime"),
        ("/hdr_ros2_driver/log/get/manager", "hdr_msgs/srv/LogManager"),
    ],
    "control": [
        ("/hdr_ros2_driver/control/get/op_cnd", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/control/get/ucs_nos", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/control/put/op_cnd", "hdr_msgs/srv/OpCnd")
    ],
    "robot": [
        ("/hdr_ros2_driver/robot/post/motor_power", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/robot/post/emergency_stop_test", "hdr_msgs/srv/Emergency"),
        ("/hdr_ros2_driver/robot/get/motor_state", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/robot/get/po_cur", "hdr_msgs/srv/PoseCur"),
        ("/hdr_ros2_driver/robot/get/cur_tool", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/robot/get/tools", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/robot/get/tools_t", "hdr_msgs/srv/Number"),
        ("/hdr_ros2_driver/robot/get/emergency", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/console/post/execute_cmd", "hdr_msgs/srv/ExecuteCmd"),
        ("/hdr_ros2_driver/robot/post/motor_power", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/robot/post/operation", "std_srvs/srv/SetBool"),
        ("/hdr_ros2_driver/robot/get/joint_traj_buff_avail", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/robot/post/init_joint_trajectory", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/robot/post/insert_joint_trajectory_points", "hdr_msgs/srv/JointTrajectoryPoints"),
        ("/hdr_ros2_driver/robot/post/tool_no", "hdr_msgs/srv/Number"),
        ("/hdr_ros2_driver/robot/post/crd_sys", "hdr_msgs/srv/Number"),
        ("/hdr_ros2_driver/robot/post/emergency_stop", "std_srvs/srv/Trigger")
    ],
    "plc": [
        ("/hdr_ros2_driver/plc/get/relay_value", "hdr_msgs/srv/IoplcGet"),
        ("/hdr_ros2_driver/plc/post/relay_value", "hdr_msgs/srv/IoplcPost"),
        ("/hdr_ros2_driver/control/get/ios/dio", "hdr_msgs/srv/IoRequest"),  
        ("/hdr_ros2_driver/control/get/ios/sio", "hdr_msgs/srv/IoRequest"),  
        ("/hdr_ros2_driver/control/post/ios/dio", "hdr_msgs/srv/IoRequest"),
    ],
    "console": [
        ("/hdr_ros2_driver/console/post/execute_cmd", "hdr_msgs/srv/ExecuteCmd")
    ],
    "task": [
        ("/hdr_ros2_driver/task/post/reset", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/task/post/set_cur_pc_idx", "hdr_msgs/srv/Number"),
        ("/hdr_ros2_driver/task/post/release_wait", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/task/post/assign_var", "hdr_msgs/srv/ProgramVar"),
        ("/hdr_ros2_driver/task/post/solve_expr", "hdr_msgs/srv/ProgramVar"),
        ("/hdr_ros2_driver/robot/post/motor_power", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/task/post/execute_move", "hdr_msgs/srv/ExecuteMove"),
    ],
    "file": [
        ("/hdr_ros2_driver/file/post/files", "hdr_msgs/srv/FileSend"),
        ("/hdr_ros2_driver/project/post/reload_updated_jobs", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/file/get/files", "hdr_msgs/srv/FilePath"),
        ("/hdr_ros2_driver/file/get/file_info", "hdr_msgs/srv/FilePath"),
        ("/hdr_ros2_driver/file/get/file_exist", "hdr_msgs/srv/FilePath"),
        ("/hdr_ros2_driver/file/post/mkdir", "hdr_msgs/srv/FilePath"),
        ("/hdr_ros2_driver/file/delete/files", "hdr_msgs/srv/FilePath"),
        ("/hdr_ros2_driver/file/get/file_list", "hdr_msgs/srv/FileList"),
        ("/hdr_ros2_driver/file/post/rename_file", "hdr_msgs/srv/FileRename"),
        ("/hdr_ros2_driver/project/post/reload_updated_jobs", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/project/post/delete_job", "hdr_msgs/srv/FilePath"),
    ],
    "project": [
        ("/hdr_ros2_driver/project/get/rgen", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/project/get/jobs_info", "std_srvs/srv/Trigger"),
        ("/hdr_ros2_driver/project/post/reload_updated_jobs", "std_srvs/srv/Trigger"),
    ],
}

def get_category_services(category: str):
    if category == "all":
        all_services = []
        for service_list in service_type_mapping.values():
            all_services.extend(service_list)
        return all_services
    return service_type_mapping.get(category, [])


def fill_request_fields(service_type_str, req, service_name=""):
    try:
        if "SetBool" in service_type_str:
            req.data = True
        elif "Number" in service_type_str:
            req.data = 0
        elif "IoRequest" in service_type_str:
            if "get/ios/dio" in service_name:
                req.type = "di"
                req.blk_no = 1
                req.sig_no = 1
            elif "post/ios/dio" in service_name:
                req.type = "dob"
                req.blk_no = 2
                req.sig_no = 3
                req.val = -99
            elif "get/ios/sio" in service_name:
                req.type = "si"
                req.sig_no = 3
        elif "OpCnd" in service_type_str:
            req.playback_mode = 1
            req.step_goback_max_spd = 130
            req.ucrd_num = 2
        elif "PoseCur" in service_type_str:
            req.crd = 0
        elif "Emergency" in service_type_str:
            req.step_no = 1
            req.stop_at = 50
            req.stop_at_corner = 0
            req.category = 1
        elif "IoplcGet" in service_type_str:
            req.type = 'm'
            req.st = 32
            req.len = 4
        elif "IoplcPost" in service_type_str:
            req.name = "fb1.do0"
            req.value = 2.718
        elif "DateTime" in service_type_str:
            now = datetime.now()
            req.year = now.year
            req.mon = now.month
            req.day = now.day
            req.hour = now.hour
            req.min = now.minute
            req.sec = now.second
        elif "FileRename" in service_type_str:
            req.pathname_from = "project/jobs/test.job"
            req.pathname_to = "project/jobs/8888.job"
        elif "FileSend" in service_type_str:
            current_path = os.getcwd()
            req.target_file = "project/jobs/test.job"
            req.source_file = current_path + "/test.job"
        elif "FilePath" in service_type_str:
            if "post/mkdir" in service_name or "delete/files" in service_name:
                req.path = "project/jobs/test_dir"
            elif "post/delete_job" in service_name:
                req.path = "8888.job"
            else:
                req.path = "project/jobs/test.job"
        elif "FileList" in service_type_str:
            req.path = "project/jobs"
            req.incl_file = True
            req.incl_dir = False
        elif "ProgramCnt" in service_type_str:
            req.pno = -1
            req.sno = -1
            req.fno = -1
            req.ext_sel = 0
        elif "ProgramVar" in service_type_str:
            if "solve_expr" in service_name:
                req.scope = "global"
                req.expr = "a"
            else:
                req.name = "a"
                req.scope = "global"
                req.expr = "{\"test\": 10}"
                req.save = True
        elif "ExecuteMove" in service_type_str:
            req.stmt = "move SP,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]"
            req.task_no = 0
        elif "ExecuteCmd" in service_type_str:
            req.cmd_line = [
                "rl.stop",
                "rl.reinit",
                "rl.i wait di1",
                "rl.i end"
            ]
            req.period_ms = 100
        elif "LogManager" in service_type_str:
            req.n_item = 5
            req.cat_p = "E,W,P,O"

        elif "JointTrajectoryPoints" in service_type_str:
            # Build a minimal valid JointTrajectory with >= 2 points
            traj = JointTrajectory()
            traj.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

            p1 = JointTrajectoryPoint()
            p1.positions = [0.0, 1.570796, 0.0, 0.0, 0.0, 0.0]
            p1.time_from_start.sec = 0
            p1.time_from_start.nanosec = 0

            p2 = JointTrajectoryPoint()
            p2.positions = [0.01, 1.570796, 0.0, 0.0, 0.0, 0.0]
            p2.time_from_start.sec = 3
            p2.time_from_start.nanosec = 0

            traj.points = [p1, p2]
            req.trajectory = traj
    except Exception as e:
        print(f"Failed to fill fields for {service_type_str}: {e}")
