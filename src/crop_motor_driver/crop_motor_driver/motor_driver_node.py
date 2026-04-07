#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.get_logger().info('motor_driver_node started')

    def cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        wheel_base = 0.55

        left = v - (w * wheel_base / 2.0)
        right = v + (w * wheel_base / 2.0)

        self.get_logger().info(f'left={left:.3f}, right={right:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
