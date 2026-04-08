#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# ================= CONSTANTS =================
LINEAR_AXIS  = 7    # D-pad up/down (up=-1.0, down=1.0)
STEER_AXIS   = 6    # D-pad left/right (left=1.0, right=-1.0)
DEADZONE     = 0.0
MAX_LINEAR   = 1.0
MAX_STEER    = 1.0  # will be scaled to degrees in motor node
# ============================================

def apply_deadzone(value):
    """Apply deadzone to ignore small stick drift."""
    return 0.0 if abs(value) < DEADZONE else value

class RemoteControllerNode(Node):
    def __init__(self):
        super().__init__('remote_controller_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("Remote Controller Node Started")

    def joy_callback(self, msg):
        # D-pad up/down → forward/back
        linear_input = apply_deadzone(msg.axes[LINEAR_AXIS])

        # D-pad left/right → steering angle
        steer_input  = apply_deadzone(msg.axes[STEER_AXIS])

        cmd = Twist()
        cmd.linear.x  = linear_input * MAX_LINEAR   # drive forward/back
        cmd.linear.y  = steer_input  * MAX_STEER    # steering angle
        cmd.angular.z = 0.0                          # unused
        self.publisher.publish(cmd)

def main():
    rclpy.init()
    node = RemoteControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
