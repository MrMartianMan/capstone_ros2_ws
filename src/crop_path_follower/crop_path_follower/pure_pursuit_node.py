#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.path = None
        self.sub = self.create_subscription(Path, '/centerline_path', self.path_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info('pure_pursuit_node started')

    def path_cb(self, msg):
        self.path = msg

    def loop(self):
        cmd = Twist()

        if self.path is None or len(self.path.poses) == 0:
            self.pub.publish(cmd)
            return

        target = None
        for pose in self.path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            d = math.sqrt(x * x + y * y)
            if d >= 1.0:
                target = (x, y)
                break

        if target is None:
            target = (
                self.path.poses[-1].pose.position.x,
                self.path.poses[-1].pose.position.y
            )

        tx, ty = target
        if tx <= 0.05:
            self.pub.publish(cmd)
            return

        cmd.linear.x = 0.3
        cmd.angular.z = max(-1.0, min(1.0, 2.0 * ty))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
