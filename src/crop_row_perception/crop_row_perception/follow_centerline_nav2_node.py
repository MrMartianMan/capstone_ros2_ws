#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


class FollowCenterlineNav2Node(Node):
    def __init__(self) -> None:
        super().__init__('follow_centerline_nav2_node')

        self.path_topic = '/centerline_path'
        self.controller_action_name = 'follow_path'

        # These IDs must match nav2_params.yaml
        self.controller_id = 'FollowPath'
        self.goal_checker_id = 'goal_checker'

        # Re-send logic
        self.min_path_poses = 3
        self.min_update_period = 0.5
        self.min_goal_shift = 0.10

        self.latest_path: Optional[Path] = None
        self.last_sent_path: Optional[Path] = None
        self.last_send_time = None
        self.active_goal_handle = None

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_cb,
            10
        )

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            self.controller_action_name
        )

        self.timer = self.create_timer(0.2, self.timer_cb)

        self.get_logger().info('### FOLLOW CENTERLINE NAV2 NODE ###')
        self.get_logger().info(f'path_topic={self.path_topic}')
        self.get_logger().info(f'action={self.controller_action_name}')

    def path_cb(self, msg: Path) -> None:
        if len(msg.poses) < self.min_path_poses:
            self.get_logger().warn('Ignoring short path')
            return
        self.latest_path = msg

    def goal_shift(self, a: Path, b: Path) -> float:
        if not a.poses or not b.poses:
            return 999.0

        ax = a.poses[-1].pose.position.x
        ay = a.poses[-1].pose.position.y
        bx = b.poses[-1].pose.position.x
        by = b.poses[-1].pose.position.y

        return math.hypot(ax - bx, ay - by)

    def should_send_new_goal(self) -> bool:
        if self.latest_path is None:
            return False

        if self.last_sent_path is None or self.last_send_time is None:
            return True

        elapsed = (self.get_clock().now() - self.last_send_time).nanoseconds * 1e-9
        if elapsed < self.min_update_period:
            return False

        shift = self.goal_shift(self.latest_path, self.last_sent_path)
        return shift >= self.min_goal_shift

    def timer_cb(self) -> None:
        if self.latest_path is None:
            return

        if not self.follow_path_client.wait_for_server(timeout_sec=0.05):
            self.get_logger().warn('Nav2 FollowPath action server not available yet')
            return

        if not self.should_send_new_goal():
            return

        self.send_goal(self.latest_path)

    def send_goal(self, path: Path) -> None:
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f'Sending path to Nav2: poses={len(path.poses)}, frame={path.header.frame_id}'
        )

        future = self.follow_path_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)

        self.last_sent_path = path
        self.last_send_time = self.get_clock().now()

    def goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath goal rejected')
            return

        self.get_logger().info('FollowPath goal accepted')
        self.active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'FollowPath feedback: distance_to_goal={feedback.distance_to_goal:.3f}, '
            f'speed={feedback.speed:.3f}'
        )

    def result_cb(self, future) -> None:
        try:
            result = future.result().result
            self.get_logger().info(f'FollowPath finished: {result}')
        except Exception as e:
            self.get_logger().warn(f'FollowPath result error: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowCenterlineNav2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
