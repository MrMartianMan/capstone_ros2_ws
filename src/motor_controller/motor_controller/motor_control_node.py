#!/usr/bin/env python3
"""
motor_control_node.py
Subscribes to /cmd_vel and sends VESC CAN commands for a 4-wheel
independent steering robot (8 VESCs total).

CAN ID Assignment:
  FL Drive=1  FL Steer=2
  FR Drive=3  FR Steer=4
  RL Drive=5  RL Steer=6
  RR Drive=7  RR Steer=8

VESC CAN packet format:
  Drive  → SET_RPM    (command 3)  : 4-byte signed int (RPM)
  Steer  → SET_POS    (command 26) : 4-byte signed int (degrees * 1000000)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import struct
import math

# ================= CONSTANTS =================
WHEEL_BASE_X   = 0.6    # meters front-to-rear
WHEEL_BASE_Y   = 0.5    # meters left-to-right
MAX_RPM        = 3000   # max drive motor RPM
MAX_STEER_DEG  = 30.0   # max steering angle in degrees
CAN_INTERFACE  = 'can0'

# VESC CAN commands
VESC_SET_RPM   = 3
VESC_SET_POS   = 26

# CAN IDs
FL_DRIVE = 1;  FL_STEER = 2
FR_DRIVE = 3;  FR_STEER = 4
RL_DRIVE = 5;  RL_STEER = 6
RR_DRIVE = 7;  RR_STEER = 8
# ============================================


def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        # CAN bus setup
        self.bus = None
        self.can_available = False
        self._connect_can()

        # Watchdog — stops motors if no cmd_vel received
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_callback)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Motor Control Node Started")
        if not self.can_available:
            self.get_logger().warn(
                "CAN bus not available — running in simulation mode (printing only)"
            )

    def _connect_can(self):
        try:
            self.bus = can.interface.Bus(
                channel=CAN_INTERFACE,
                bustype='socketcan'
            )
            self.can_available = True
            self.get_logger().info(f'CAN bus connected: {CAN_INTERFACE}')
        except Exception as e:
            self.get_logger().warn(f'CAN not connected: {e}')
            self.can_available = False

    # ─── VESC CAN HELPERS ───────────────────────────────────────────

    def vesc_set_rpm(self, vesc_id, rpm):
        """Send SET_RPM command to a VESC over CAN."""
        rpm_int = int(clamp(rpm, -MAX_RPM, MAX_RPM))
        if self.can_available:
            try:
                arb_id = (vesc_id & 0xFF) | (VESC_SET_RPM << 8)
                data = struct.pack('>i', rpm_int)
                msg = can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=True
                )
                self.bus.send(msg)
            except can.CanError as e:
                self.get_logger().error(f'CAN send error (RPM): {e}')
        else:
            self.get_logger().info(f'[SIM] VESC {vesc_id} RPM: {rpm_int}')

    def vesc_set_position(self, vesc_id, degrees):
        """Send SET_POS command to a VESC over CAN."""
        degrees = clamp(degrees, -MAX_STEER_DEG, MAX_STEER_DEG)
        pos_int = int(degrees * 1000000)
        if self.can_available:
            try:
                arb_id = (vesc_id & 0xFF) | (VESC_SET_POS << 8)
                data = struct.pack('>i', pos_int)
                msg = can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=True
                )
                self.bus.send(msg)
            except can.CanError as e:
                self.get_logger().error(f'CAN send error (POS): {e}')
        else:
            self.get_logger().info(
                f'[SIM] VESC {vesc_id} POS: {degrees:.2f} deg'
            )

    # ─── STEERING GEOMETRY ──────────────────────────────────────────

    def compute_steering_angles(self, linear_x, angular_z):
        """
        Ackermann-style steering angles for 4 wheels.
        Returns (fl, fr, rl, rr) in degrees.
        """
        if abs(angular_z) < 0.01:
            return 0.0, 0.0, 0.0, 0.0

        # Turning radius from center
        radius = linear_x / angular_z if abs(linear_x) > 0.01 else \
                 math.copysign(0.5, angular_z)

        half_x = WHEEL_BASE_X / 2.0
        half_y = WHEEL_BASE_Y / 2.0

        fl = math.degrees(math.atan2(half_x, abs(radius) - half_y))
        fr = math.degrees(math.atan2(half_x, abs(radius) + half_y))
        rl = -fl
        rr = -fr

        if angular_z < 0:
            fl, fr, rl, rr = -fl, -fr, -rl, -rr

        return fl, fr, rl, rr

    # ─── MAIN CALLBACK ──────────────────────────────────────────────

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear  = clamp(msg.linear.x,  -1.0, 1.0)
        angular = clamp(msg.angular.z, -1.0, 1.0)

        # Drive speeds — scale to RPM
        target_rpm = linear * MAX_RPM

        # All drive motors get same RPM (steer handles direction)
        self.vesc_set_rpm(FL_DRIVE, target_rpm)
        self.vesc_set_rpm(FR_DRIVE, target_rpm)
        self.vesc_set_rpm(RL_DRIVE, target_rpm)
        self.vesc_set_rpm(RR_DRIVE, target_rpm)

        # Steering angles
        fl, fr, rl, rr = self.compute_steering_angles(linear, angular)

        self.vesc_set_position(FL_STEER, fl)
        self.vesc_set_position(FR_STEER, fr)
        self.vesc_set_position(RL_STEER, rl)
        self.vesc_set_position(RR_STEER, rr)

    def watchdog_callback(self):
        """Stop all motors if no cmd_vel received within 0.5 seconds."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            for drive_id in [FL_DRIVE, FR_DRIVE, RL_DRIVE, RR_DRIVE]:
                self.vesc_set_rpm(drive_id, 0)
            for steer_id in [FL_STEER, FR_STEER, RL_STEER, RR_STEER]:
                self.vesc_set_position(steer_id, 0.0)

    def destroy_node(self):
        self.get_logger().info('Shutting down — stopping all motors.')
        for drive_id in [FL_DRIVE, FR_DRIVE, RL_DRIVE, RR_DRIVE]:
            self.vesc_set_rpm(drive_id, 0)
        for steer_id in [FL_STEER, FR_STEER, RL_STEER, RR_STEER]:
            self.vesc_set_position(steer_id, 0.0)
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
