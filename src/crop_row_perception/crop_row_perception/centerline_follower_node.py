#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

import tf2_ros


class CenterlineFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('centerline_follower_node')

        # Topics
        self.path_topic = '/centerline_path'
        self.cmd_vel_topic = '/cmd_vel'

        # Frames
        self.path_frame = None
        self.robot_frame = 'zed_left_camera_frame'

        # Safer tracking params
        self.lookahead_distance = 0.45
        self.linear_speed = 0.08
        self.max_angular_speed = 0.60
        self.angular_gain = 1.80

        # Stop / timeout
        self.goal_tolerance = 0.12
        self.control_rate_hz = 10.0
        self.path_timeout_sec = 0.8

        # Command smoothing
        self.max_linear_step = 0.02
        self.max_angular_step = 0.08
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        self.latest_path: Optional[Path] = None
        self.latest_path_time = None

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_cb,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info('### TUNED CENTERLINE FOLLOWER NODE ###')
        self.get_logger().info(f'path_topic={self.path_topic}')
        self.get_logger().info(f'cmd_vel_topic={self.cmd_vel_topic}')
        self.get_logger().info(f'robot_frame={self.robot_frame}')

    def path_cb(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            self.get_logger().warn('Received path with too few poses')
            return

        self.latest_path = msg
        self.latest_path_time = self.get_clock().now()
        self.path_frame = msg.header.frame_id

    def stop_robot(self) -> None:
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def get_robot_pose_in_path_frame(self) -> Optional[Tuple[float, float, float]]:
        if self.path_frame is None:
            return None

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y

        q = tf_msg.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return tx, ty, yaw

    def path_points(self) -> List[Tuple[float, float]]:
        if self.latest_path is None:
            return []

        pts = []
        for pose_stamped in self.latest_path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            pts.append((x, y))
        return pts

    def distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def find_lookahead_point(
        self,
        robot_xy: Tuple[float, float],
        path_pts: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        if not path_pts:
            return None

        for pt in path_pts:
            if self.distance(robot_xy, pt) >= self.lookahead_distance:
                return pt

        return path_pts[-1]

    def clamp_step(self, target: float, previous: float, max_step: float) -> float:
        delta = target - previous
        if delta > max_step:
            return previous + max_step
        if delta < -max_step:
            return previous - max_step
        return target

    def control_loop(self) -> None:
        if self.latest_path is None or self.latest_path_time is None:
            self.stop_robot()
            return

        age = (self.get_clock().now() - self.latest_path_time).nanoseconds * 1e-9
        if age > self.path_timeout_sec:
            self.get_logger().warn('Path timed out, stopping robot')
            self.stop_robot()
            return

        robot_pose = self.get_robot_pose_in_path_frame()
        if robot_pose is None:
            self.stop_robot()
            return

        robot_x, robot_y, robot_yaw = robot_pose
        robot_xy = (robot_x, robot_y)

        path_pts = self.path_points()
        if len(path_pts) < 2:
            self.stop_robot()
            return

        goal_xy = path_pts[-1]
        dist_to_goal = self.distance(robot_xy, goal_xy)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached, stopping robot')
            self.stop_robot()
            return

        lookahead_xy = self.find_lookahead_point(robot_xy, path_pts)
        if lookahead_xy is None:
            self.stop_robot()
            return

        dx = lookahead_xy[0] - robot_x
        dy = lookahead_xy[1] - robot_y

        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - robot_yaw

        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        # Raw desired commands
        turn_scale = max(0.25, 1.0 - min(abs(heading_error), 1.0))
        desired_linear = self.linear_speed * turn_scale

        desired_angular = self.angular_gain * heading_error
        desired_angular = max(-self.max_angular_speed, min(self.max_angular_speed, desired_angular))

        # Smooth command changes
        cmd_linear = self.clamp_step(desired_linear, self.prev_linear, self.max_linear_step)
        cmd_angular = self.clamp_step(desired_angular, self.prev_angular, self.max_angular_step)

        self.prev_linear = cmd_linear
        self.prev_angular = cmd_angular

        cmd = Twist()
        cmd.linear.x = cmd_linear
        cmd.angular.z = cmd_angular
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CenterlineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
