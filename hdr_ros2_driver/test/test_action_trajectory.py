#!/usr/bin/env python3

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState


class HomePositionClient(Node):
    def __init__(self):
        super().__init__('home_position_client')

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/hdr_ros2_driver/follow_joint_trajectory'
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            '/hdr_ros2_driver/joint_states',
            self.on_joint_states,
            10
        )

        self.current_positions = None
        self.goal_sent = False

        self.home_position = [0.0, 1.571, 0.0, 0.0, 0.0, 0.0]
        self.intermediate_positions = [
            [0.2, 1.571, 0.0, 0.0, 0.0, 0.0],
            [0.4, 1.571, 0.0, 0.0, 0.0, 0.0],
            [0.2, 1.571, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.571, 0.0, 0.0, 0.0, 0.0]
        ]

        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

    def on_joint_states(self, msg):
        if len(msg.name) >= 6 and not self.goal_sent:
            positions = [0.0] * 6
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    try:
                        index = self.joint_names.index(name)
                        if 0 <= index < 6:
                            positions[index] = msg.position[i]
                    except ValueError:
                        pass

            self.current_positions = positions
            self.get_logger().info(f'Received current positions: {self.current_positions}')
            self.goal_sent = True
            self.send_trajectory_goal()

    def send_trajectory_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        if self.current_positions is None:
            self.get_logger().error('No current joint state received.')
            return

        dt_seconds = 0.01
        points_per_segment = 25

        all_positions = [self.current_positions] + self.intermediate_positions + [self.home_position]

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points = []

        time_from_start_nsec = 0

        for i in range(len(all_positions) - 1):
            start = np.array(all_positions[i])
            end = np.array(all_positions[i + 1])
            segment = np.linspace(start, end, num=points_per_segment)

            for pos in segment:
                point = JointTrajectoryPoint()
                point.positions = pos.tolist()

                nanoseconds = int(dt_seconds * 1e9)
                time_from_start_nsec += nanoseconds

                point.time_from_start = Duration(
                    sec=time_from_start_nsec // int(1e9),
                    nanosec=time_from_start_nsec % int(1e9)
                )
                trajectory.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        goal_msg.goal_time_tolerance = Duration(sec=1)

        self.get_logger().info(f'Sending {len(trajectory.points)} trajectory points')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_feedback_received
        )
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            sys.exit(0)
            return

        self.get_logger().info('Goal accepted - moving through points to home position')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.on_result_received)

    def on_result_received(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info('Successfully reached home position')
        else:
            self.get_logger().info(
                f'Failed. Status: {status}, Error code: {result.error_code}'
            )

        self.get_logger().info('Task completed, exiting...')
        sys.exit(0)

    def on_feedback_received(self, feedback_msg):
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'actual') and feedback.actual.positions:
            positions_str = ', '.join(f'{p:.4f}' for p in feedback.actual.positions)
            self.get_logger().info(f'Current positions: {positions_str}')


def main(args=None):
    rclpy.init(args=args)
    client = HomePositionClient()

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        return

    rclpy.shutdown()


if __name__ == '__main__':
    main()
