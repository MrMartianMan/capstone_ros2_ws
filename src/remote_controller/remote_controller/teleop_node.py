#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# ================= BUTTON MAP =================
LINEAR_AXIS  = 7    # D-pad up/down
STEER_AXIS   = 6    # D-pad left/right

BTN_A  = 0
BTN_B  = 1
BTN_X  = 2
BTN_Y  = 3
BTN_LB = 4
BTN_RB = 5
BTN_LT = 6
BTN_RT = 7
# ==============================================

class RemoteControllerNode(Node):
    def __init__(self):
        super().__init__('remote_controller_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_timer(0.05, self.publish_cmd)  # publish at 20Hz always

        self.individual_mode = False
        self.steer_direction = 1.0

        self.lb_prev = 0
        self.rb_prev = 0

        self.last_cmd = Twist()  # stores last command to keep publishing

        self.get_logger().info("Remote Controller Node Started")
        self.get_logger().info("Mode: DEFAULT — X/B control all steer motors")

    def joy_callback(self, msg):
        cmd = Twist()

        # ── D-pad drive ───────────────────────────────────────────
        cmd.linear.x = msg.axes[LINEAR_AXIS]
        cmd.linear.y = msg.axes[STEER_AXIS]

        lb = msg.buttons[BTN_LB]
        rb = msg.buttons[BTN_RB]

        # ── Toggle individual mode — rising edge of LB ────────────
        if lb and not self.lb_prev:
            self.individual_mode = not self.individual_mode
            mode_str = "INDIVIDUAL" if self.individual_mode else "DEFAULT"
            self.get_logger().info(f"Switched to {mode_str} mode")

        # ── RB toggles direction — rising edge only ───────────────
        if rb and not self.rb_prev:
            self.steer_direction = -self.steer_direction
            dir_str = "FORWARD" if self.steer_direction > 0 else "BACKWARD"
            self.get_logger().info(f"Steer direction: {dir_str}")

        self.lb_prev = lb
        self.rb_prev = rb

        if self.individual_mode:
            # ── Individual mode — ABXY control one motor each ─────
            cmd.angular.x = self.steer_direction if msg.buttons[BTN_A] else 0.0  # FL
            cmd.angular.y = self.steer_direction if msg.buttons[BTN_B] else 0.0  # FR
            cmd.angular.z = self.steer_direction if msg.buttons[BTN_X] else 0.0  # RL
            cmd.linear.z  = self.steer_direction if msg.buttons[BTN_Y] else 0.0  # RR
        else:
            # ── Default mode — X forward, B backward all 4 ────────
            if msg.buttons[BTN_X]:
                steer_val = 1.0
            elif msg.buttons[BTN_B]:
                steer_val = -1.0
            else:
                steer_val = 0.0

            cmd.angular.x = steer_val
            cmd.angular.y = steer_val
            cmd.angular.z = steer_val
            cmd.linear.z  = steer_val

        self.last_cmd = cmd

    def publish_cmd(self):
        self.publisher.publish(self.last_cmd)


def main():
    rclpy.init()
    node = RemoteControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
