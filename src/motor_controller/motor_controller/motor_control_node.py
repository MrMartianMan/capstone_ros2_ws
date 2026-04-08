#!/usr/bin/env python3
"""
motor_control_node.py
ERPM-based differential drive control for 4-wheel robot with 8 VESCs.
D-pad up/down = drive forward/back
D-pad left/right while moving = arc turn (one side faster than other)
D-pad left/right while stopped = zero point turn
Steering motors locked at 0 degrees.

CAN ID Assignment:
  FL Drive=76  FL Steer=97
  FR Drive=7   FR Steer=21
  RL Drive=53  RL Steer=59
  RR Drive=61  RR Steer=37
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import struct

# ================= CONSTANTS =================
WHEEL_BASE_Y  = 0.5      # meters left-to-right
MAX_RPM       = 900      # max ERPM (900 is minimum to start motor)
MIN_RPM       = 450      # minimum ERPM for slower side during arc turn
RAMP_RATE     = 100      # ERPM per cycle to ramp up/down
CAN_INTERFACE = 'can0'   # CAN interface name

# VESC CAN commands
VESC_SET_RPM  = 3        # command 3 = SET_RPM (takes ERPM)
VESC_SET_POS  = 26       # command 26 = SET_POS (for steering)

# CAN IDs — drive motors use ERPM, steer motors use position
FL_DRIVE = 76   # Front Left drive motor
FL_STEER = 97   # Front Left steer motor
FR_DRIVE = 7    # Front Right drive motor
FR_STEER = 21   # Front Right steer motor
RL_DRIVE = 53   # Rear Left drive motor
RL_STEER = 59   # Rear Left steer motor
RR_DRIVE = 61   # Rear Right drive motor
RR_STEER = 37   # Rear Right steer motor
# ============================================


def clamp(value, min_val, max_val):
    """Clamp a value between min and max."""
    return max(min(value, max_val), min_val)


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        # CAN bus setup
        self.bus = None
        self.can_available = False
        self._connect_can()

        # Current ERPM for ramping
        self.current_left_rpm  = 0.0
        self.current_right_rpm = 0.0

        # Watchdog — stops motors if no cmd_vel received within 0.5s
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self.watchdog_callback)

        # Subscribe to velocity commands from teleop node
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Motor Control Node Started")
        if not self.can_available:
            self.get_logger().warn(
                "CAN bus not available — running in simulation mode"
            )

        # On startup hold steering straight
        self._center_steering()

    def _connect_can(self):
        """Connect to the CAN bus interface."""
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
        """Send SET_RPM command to a VESC over CAN.
        rpm range: -MAX_RPM to MAX_RPM (in ERPM)
        Value is a signed 32-bit integer
        """
        rpm = clamp(rpm, -MAX_RPM, MAX_RPM)
        rpm_int = int(rpm)

        if self.can_available:
            try:
                arb_id = (VESC_SET_RPM << 8) | (vesc_id & 0xFF)
                data = struct.pack('>i', rpm_int)
                msg = can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=True
                )
                self.bus.send(msg)
            except Exception as e:
                self.get_logger().error(f'CAN RPM send error: {e}')
        else:
            self.get_logger().info(f'[SIM] VESC {vesc_id} ERPM: {rpm_int}')

    def vesc_set_position(self, vesc_id, degrees):
        """Send SET_POS command to hold steering at an angle."""
        pos_int = int(degrees * 1000000)

        if self.can_available:
            try:
                arb_id = (VESC_SET_POS << 8) | (vesc_id & 0xFF)
                data = struct.pack('>i', pos_int)
                msg = can.Message(
                    arbitration_id=arb_id,
                    data=data,
                    is_extended_id=True
                )
                self.bus.send(msg)
            except Exception as e:
                self.get_logger().error(f'CAN steer error: {e}')

    def _center_steering(self):
        """Hold all steering motors straight on startup."""
        for steer_id in [FL_STEER, FR_STEER, RL_STEER, RR_STEER]:
            self.vesc_set_position(steer_id, 0.0)

    # ─── DIFFERENTIAL DRIVE ─────────────────────────────────────────

    def cmd_vel_callback(self, msg):
        """Convert Twist message to differential drive ERPM commands.
        linear.x = D-pad up/down (forward/back)
        linear.y = D-pad left/right (arc turn while moving, zero point when stopped)
        """
        self.last_cmd_time = self.get_clock().now()

        linear  = clamp(msg.linear.x, -1.0, 1.0)   # D-pad up/down
        angular = clamp(msg.linear.y, -1.0, 1.0)    # D-pad left/right

        if linear != 0.0 and angular != 0.0:
            # Moving + turning — arc turn
            # Outer wheel = MAX_RPM, inner wheel = MIN_RPM
            sign = 1 if linear > 0 else -1
            if angular < 0:
                # Turning left — left side slower
                target_left  = sign * MIN_RPM
                target_right = sign * MAX_RPM
            else:
                # Turning right — right side slower
                target_left  = sign * MAX_RPM
                target_right = sign * MIN_RPM

        elif linear > 0:
            # Forward only
            target_left  = MAX_RPM
            target_right = MAX_RPM
        elif linear < 0:
            # Backward only
            target_left  = -MAX_RPM
            target_right = -MAX_RPM
        elif angular < 0:
            # Zero point turn left (stopped)
            target_left  = -MAX_RPM
            target_right = MAX_RPM
        elif angular > 0:
            # Zero point turn right (stopped)
            target_left  = MAX_RPM
            target_right = -MAX_RPM
        else:
            # No input — stop
            target_left  = 0.0
            target_right = 0.0

        # Ramp left side toward target smoothly
        if self.current_left_rpm < target_left:
            if self.current_left_rpm == 0 and target_left > 0:
                self.current_left_rpm = target_left  # jump immediately
            else:
                self.current_left_rpm = min(self.current_left_rpm + RAMP_RATE, target_left)
        elif self.current_left_rpm > target_left:
            if self.current_left_rpm == 0 and target_left < 0:
                self.current_left_rpm = target_left  # jump immediately
            else:
                self.current_left_rpm = max(self.current_left_rpm - RAMP_RATE, target_left)

        # Ramp right side toward target smoothly
        if self.current_right_rpm < target_right:
            if self.current_right_rpm == 0 and target_right > 0:
                self.current_right_rpm = target_right  # jump immediately
            else:
                self.current_right_rpm = min(self.current_right_rpm + RAMP_RATE, target_right)
        elif self.current_right_rpm > target_right:
            if self.current_right_rpm == 0 and target_right < 0:
                self.current_right_rpm = target_right  # jump immediately
            else:
                self.current_right_rpm = max(self.current_right_rpm - RAMP_RATE, target_right)

        # Interleave left/right sends to minimize timing gap
        self.vesc_set_rpm(FL_DRIVE, self.current_left_rpm)
        self.vesc_set_rpm(FR_DRIVE, self.current_right_rpm)
        self.vesc_set_rpm(RL_DRIVE, self.current_left_rpm)
        self.vesc_set_rpm(RR_DRIVE, self.current_right_rpm)

        self.get_logger().info(
            f'Left ERPM: {self.current_left_rpm:.0f} | Right ERPM: {self.current_right_rpm:.0f}'
        )

    def watchdog_callback(self):
        """Stop all motors if no cmd_vel received within 0.5 seconds."""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5:
            for drive_id in [FL_DRIVE, FR_DRIVE, RL_DRIVE, RR_DRIVE]:
                self.vesc_set_rpm(drive_id, 0)
            self.current_left_rpm  = 0.0
            self.current_right_rpm = 0.0

    def destroy_node(self):
        """Clean shutdown — stop all motors and center steering."""
        self.get_logger().info('Shutting down — stopping all motors.')
        for drive_id in [FL_DRIVE, FR_DRIVE, RL_DRIVE, RR_DRIVE]:
            self.vesc_set_rpm(drive_id, 0)
        self._center_steering()
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
