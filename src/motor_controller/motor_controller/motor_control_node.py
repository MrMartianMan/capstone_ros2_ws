#!/usr/bin/env python3
"""
motor_control_node.py

D-pad up alone     = forward ramp 900 to 4500 ERPM over 2 seconds
D-pad down alone   = backward ramp 900 to 4500 ERPM over 2 seconds
D-pad left alone   = zero point spin left ramp 900 to 4500 ERPM over 2 seconds
D-pad right alone  = zero point spin right ramp 900 to 4500 ERPM over 2 seconds
ANY combination of 2+ D-pad inputs = ramp down to stop over 3 seconds
Release = ramp down to stop over 3 seconds

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
import math
import time

# ================= CONSTANTS =================
START_RPM          = 900
MAX_RPM            = 4500
MAX_STEER_RPM      = 900
RAMP_UP_DURATION   = 2.0
RAMP_DOWN_DURATION = 1.0
RAMP_RATE          = 100
CAN_INTERFACE      = 'can0'
VESC_SET_RPM       = 3

FL_DRIVE = 76
FR_DRIVE = 7
RL_DRIVE = 53
RR_DRIVE = 61

FL_STEER = 97
FR_STEER = 21
RL_STEER = 59
RR_STEER = 37
# ============================================


def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def s_curve(t):
    t = clamp(t, 0.0, 1.0)
    return (1 - math.cos(math.pi * t)) / 2


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        self.bus = None
        self.can_available = False
        self._connect_can()

        # Drive state
        self.current_left_rpm  = 0.0
        self.current_right_rpm = 0.0
        self.prev_drive_mode   = 'stop'
        self.drive_mode        = 'stop'
        self.ramp_start_time   = None
        self.ramp_start_rpm    = 0.0
        self.ramp_target_rpm   = 0.0
        self.ramping_down      = False

        # Steer state
        self.current_steer_rpm = {
            FL_STEER: 0.0,
            FR_STEER: 0.0,
            RL_STEER: 0.0,
            RR_STEER: 0.0
        }

        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.02, self.control_loop)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Motor Control Node Started")
        if not self.can_available:
            self.get_logger().warn("CAN bus not available — running in simulation mode")

    def _connect_can(self):
        try:
            self.bus = can.interface.Bus(channel='/dev/ttyACM0', interface='slcan', bitrate=500000)
            self.can_available = True
            self.get_logger().info(f'CAN bus connected: {CAN_INTERFACE}')
        except Exception as e:
            self.get_logger().warn(f'CAN not connected: {e}')
            self.can_available = False

    def vesc_set_rpm(self, vesc_id, rpm, max_rpm=None):
        if max_rpm is None:
            max_rpm = MAX_RPM
        rpm = clamp(rpm, -max_rpm, max_rpm)
        rpm_int = int(rpm)
        if self.can_available:
            try:
                arb_id = (VESC_SET_RPM << 8) | (vesc_id & 0xFF)
                data = struct.pack('>i', rpm_int)
                msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
                self.bus.send(msg)
            except Exception as e:
                self.get_logger().error(f'CAN RPM send error: {e}')

    def get_current_rpm(self):
        if self.ramp_start_time is None:
            return 0.0
        elapsed  = time.time() - self.ramp_start_time
        duration = RAMP_DOWN_DURATION if self.ramping_down else RAMP_UP_DURATION
        t        = clamp(elapsed / duration, 0.0, 1.0)
        factor   = s_curve(t)
        return self.ramp_start_rpm + (self.ramp_target_rpm - self.ramp_start_rpm) * factor

    def start_ramp_up(self):
        current = self.get_current_rpm()
        self.ramp_start_time = time.time()
        self.ramp_start_rpm  = max(current, START_RPM)
        self.ramp_target_rpm = MAX_RPM
        self.ramping_down    = False

    def start_ramp_down(self):
        current = self.get_current_rpm()
        if current < START_RPM:
            current = max(abs(self.current_left_rpm), abs(self.current_right_rpm))
        self.ramp_start_time = time.time()
        self.ramp_start_rpm  = max(current, 0.0)
        self.ramp_target_rpm = 0.0
        self.ramping_down    = True

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear  = msg.linear.x
        angular = msg.linear.y

        # Check each direction
        forward  = linear  >  0.5
        backward = linear  < -0.5
        left     = angular >  0.5
        right    = angular < -0.5

        # Count active inputs — includes diagonal combos
        active = sum([forward, backward, left, right])

        # Also check for any non-zero linear.x AND non-zero linear.y simultaneously
        both_axes = (abs(linear) > 0.5) and (abs(angular) > 0.5)

        if active > 1 or both_axes:
            new_mode = 'stop'
        elif forward:
            new_mode = 'forward'
        elif backward:
            new_mode = 'backward'
        elif left:
            new_mode = 'spin_left'
        elif right:
            new_mode = 'spin_right'
        else:
            new_mode = 'stop'

        if new_mode != self.drive_mode:
            self.prev_drive_mode = self.drive_mode
            if new_mode in ('forward', 'backward', 'spin_left', 'spin_right'):
                self.start_ramp_up()
            else:
                self.start_ramp_down()
            self.drive_mode = new_mode

        # ── STEER MOTORS ──────────────────────────────────────────
        steer_cmds = {
            FL_STEER: msg.angular.x,
            FR_STEER: msg.angular.y,
            RL_STEER: msg.angular.z,
            RR_STEER: msg.linear.z
        }

        for steer_id, val in steer_cmds.items():
            target = MAX_STEER_RPM if val > 0 else (-MAX_STEER_RPM if val < 0 else 0.0)
            current = self.current_steer_rpm[steer_id]
            if current < target:
                self.current_steer_rpm[steer_id] = target if current == 0 and target > 0 else min(current + RAMP_RATE, target)
            elif current > target:
                self.current_steer_rpm[steer_id] = target if current == 0 and target < 0 else max(current - RAMP_RATE, target)
            self.vesc_set_rpm(steer_id, self.current_steer_rpm[steer_id], MAX_STEER_RPM)

    def control_loop(self):
        # Watchdog
        elapsed_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed_cmd > 1.0 and self.drive_mode != 'stop':
            self.prev_drive_mode = self.drive_mode
            self.drive_mode = 'stop'
            self.start_ramp_down()

        base_rpm = self.get_current_rpm()

        # Check if ramp down fully complete
        if self.ramping_down and self.ramp_start_time is not None:
            elapsed = time.time() - self.ramp_start_time
            if elapsed >= RAMP_DOWN_DURATION:
                base_rpm = 0.0
                self.ramp_start_time = None

        # Determine left/right RPM based on mode
        if self.drive_mode == 'forward':
            left_rpm  =  base_rpm
            right_rpm =  base_rpm
        elif self.drive_mode == 'backward':
            left_rpm  = -base_rpm
            right_rpm = -base_rpm
        elif self.drive_mode == 'spin_left':
            left_rpm  = -base_rpm
            right_rpm =  base_rpm
        elif self.drive_mode == 'spin_right':
            left_rpm  =  base_rpm
            right_rpm = -base_rpm
        else:
            # Stop mode — ramp down in previous direction
            if self.prev_drive_mode == 'forward':
                left_rpm  =  base_rpm
                right_rpm =  base_rpm
            elif self.prev_drive_mode == 'backward':
                left_rpm  = -base_rpm
                right_rpm = -base_rpm
            elif self.prev_drive_mode == 'spin_left':
                left_rpm  = -base_rpm
                right_rpm =  base_rpm
            elif self.prev_drive_mode == 'spin_right':
                left_rpm  =  base_rpm
                right_rpm = -base_rpm
            else:
                left_rpm  = 0.0
                right_rpm = 0.0

        self.current_left_rpm  = left_rpm
        self.current_right_rpm = right_rpm

        self.vesc_set_rpm(FL_DRIVE, left_rpm)
        self.vesc_set_rpm(FR_DRIVE, right_rpm)
        self.vesc_set_rpm(RL_DRIVE, left_rpm)
        self.vesc_set_rpm(RR_DRIVE, right_rpm)

        self.get_logger().info(
            f'Mode: {self.drive_mode} | ERPM: {base_rpm:.0f} | '
            f'Drive L: {left_rpm:.0f} R: {right_rpm:.0f} | '
            f'Steer FL: {self.current_steer_rpm[FL_STEER]:.0f} '
            f'FR: {self.current_steer_rpm[FR_STEER]:.0f} '
            f'RL: {self.current_steer_rpm[RL_STEER]:.0f} '
            f'RR: {self.current_steer_rpm[RR_STEER]:.0f}'
        )

    def destroy_node(self):
        self.get_logger().info('Shutting down — stopping all motors.')
        for drive_id in [FL_DRIVE, FR_DRIVE, RL_DRIVE, RR_DRIVE]:
            self.vesc_set_rpm(drive_id, 0)
        for steer_id in [FL_STEER, FR_STEER, RL_STEER, RR_STEER]:
            self.vesc_set_rpm(steer_id, 0, MAX_STEER_RPM)
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
