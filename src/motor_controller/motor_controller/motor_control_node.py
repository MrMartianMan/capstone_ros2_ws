#!/usr/bin/env python3
"""
motor_control_node.py

D-pad up alone     = forward ramp 900 to 4500 ERPM over 1.5 seconds
D-pad down alone   = backward ramp 900 to 4500 ERPM over 1.5 seconds
D-pad left alone   = zero point spin left ramp 900 to 4500 ERPM over 1.5 seconds
D-pad right alone  = zero point spin right ramp 900 to 4500 ERPM over 1.5 seconds
ANY combination of 2+ D-pad inputs = ramp down to stop
Release = ramp down to stop

Drive direction changes are protected:
- If a reverse direction is requested while moving, the node first ramps the
  current direction fully down to 0 ERPM
- Only after reaching 0 does it ramp up in the new direction

Drive ramp-down dead-zone behavior:
- Since the motors do not spin below 900 ERPM, ramp-down snaps directly to 0
  once the interpolated RPM reaches 900 or lower

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
RAMP_UP_DURATION   = 1.5
RAMP_DOWN_DURATION = 1.5
RAMP_RATE          = 100
VESC_SET_RPM       = 3

FL_DRIVE = 76
FR_DRIVE = 7
RL_DRIVE = 53
RR_DRIVE = 61

FL_STEER = 97
FR_STEER = 21
RL_STEER = 59
RR_STEER = 37

DRIVE_MOTORS = [FL_DRIVE, FR_DRIVE, RL_DRIVE, RR_DRIVE]
STEER_MOTORS = [FL_STEER, FR_STEER, RL_STEER, RR_STEER]
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
        self.current_left_rpm = 0.0
        self.current_right_rpm = 0.0

        # What the motors are currently doing
        self.active_drive_mode = 'stop'

        # What the latest command is asking for
        self.requested_drive_mode = 'stop'

        # What to switch to after ramping down to zero
        self.pending_drive_mode = None

        self.ramp_start_time = None
        self.ramp_start_rpm = 0.0
        self.ramp_target_rpm = 0.0
        self.ramping_down = False

        # Steer state
        self.current_steer_rpm = {
            FL_STEER: 0.0,
            FR_STEER: 0.0,
            RL_STEER: 0.0,
            RR_STEER: 0.0
        }
        self.steer_targets = {
            FL_STEER: 0.0,
            FR_STEER: 0.0,
            RL_STEER: 0.0,
            RR_STEER: 0.0
        }

        # Round robin index — cycles through all 8 motors one per loop
        self.motor_index = 0
        self.all_motors = DRIVE_MOTORS + STEER_MOTORS

        self.last_cmd_time = self.get_clock().now()

        # 0.0025 sec = 400 Hz loop
        # 8 motors in round-robin => each motor gets updated at ~50 Hz
        self.create_timer(0.0025, self.control_loop)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Motor Control Node Started")
        if not self.can_available:
            self.get_logger().warn("CAN bus not available — running in simulation mode")

    def _connect_can(self):
        try:
            self.bus = can.interface.Bus(
                channel='can2',
                interface='socketcan',
            )
            self.can_available = True
            self.get_logger().info('CAN bus connected: can2')
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

        elapsed = time.time() - self.ramp_start_time
        duration = RAMP_DOWN_DURATION if self.ramping_down else RAMP_UP_DURATION
        t = clamp(elapsed / duration, 0.0, 1.0)
        factor = s_curve(t)

        rpm = self.ramp_start_rpm + (self.ramp_target_rpm - self.ramp_start_rpm) * factor

        # Drive ramp-down: once we hit the dead-zone threshold, snap to zero
        if self.ramping_down and abs(rpm) <= START_RPM:
            return 0.0

        return rpm

    def start_ramp_up(self):
        current = self.get_current_rpm()
        self.ramp_start_time = time.time()
        self.ramp_start_rpm = max(current, START_RPM)
        self.ramp_target_rpm = MAX_RPM
        self.ramping_down = False

    def start_ramp_down(self):
        current = self.get_current_rpm()
        if current < START_RPM:
            current = max(abs(self.current_left_rpm), abs(self.current_right_rpm))
        self.ramp_start_time = time.time()
        self.ramp_start_rpm = max(current, 0.0)
        self.ramp_target_rpm = 0.0
        self.ramping_down = True

    def mode_signs(self, mode):
        if mode == 'forward':
            return (1, 1)
        elif mode == 'backward':
            return (-1, -1)
        elif mode == 'spin_left':
            return (-1, 1)
        elif mode == 'spin_right':
            return (1, -1)
        return (0, 0)

    def is_direction_change(self, from_mode, to_mode):
        if from_mode == 'stop' or to_mode == 'stop':
            return False

        old_left, old_right = self.mode_signs(from_mode)
        new_left, new_right = self.mode_signs(to_mode)

        return (old_left != new_left) or (old_right != new_right)

    def decode_requested_mode(self, msg):
        linear = msg.linear.x
        angular = msg.linear.y

        forward = linear > 0.5
        backward = linear < -0.5
        left = angular > 0.5
        right = angular < -0.5

        active = sum([forward, backward, left, right])
        both_axes = (abs(linear) > 0.5) and (abs(angular) > 0.5)

        if active > 1 or both_axes:
            return 'stop'
        elif forward:
            return 'forward'
        elif backward:
            return 'backward'
        elif left:
            return 'spin_left'
        elif right:
            return 'spin_right'
        else:
            return 'stop'

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        requested_mode = self.decode_requested_mode(msg)
        self.requested_drive_mode = requested_mode

        # Store steer targets
        self.steer_targets[FL_STEER] = msg.angular.x
        self.steer_targets[FR_STEER] = msg.angular.y
        self.steer_targets[RL_STEER] = msg.angular.z
        self.steer_targets[RR_STEER] = msg.linear.z

        # Stop requested
        if requested_mode == 'stop':
            self.pending_drive_mode = None
            if self.active_drive_mode != 'stop' and not self.ramping_down:
                self.start_ramp_down()
            return

        # Fully stopped: can begin new motion immediately
        if self.active_drive_mode == 'stop' and self.ramp_start_time is None:
            self.active_drive_mode = requested_mode
            self.pending_drive_mode = None
            self.start_ramp_up()
            return

        # Same direction: keep going
        if requested_mode == self.active_drive_mode and self.pending_drive_mode is None:
            return

        # Direction reversal: force ramp to zero first
        if self.is_direction_change(self.active_drive_mode, requested_mode):
            self.pending_drive_mode = requested_mode
            if not self.ramping_down:
                self.start_ramp_down()
            return

        # Any other mode change: also force zero first for clean behavior
        self.pending_drive_mode = requested_mode
        if not self.ramping_down:
            self.start_ramp_down()

    def control_loop(self):
        # Watchdog
        elapsed_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed_cmd > 1.0:
            self.requested_drive_mode = 'stop'
            self.pending_drive_mode = None
            if self.active_drive_mode != 'stop' and not self.ramping_down:
                self.start_ramp_down()

        base_rpm = self.get_current_rpm()

        # End ramp-down either when duration expires OR when dead-zone snap hits zero
        if self.ramping_down and self.ramp_start_time is not None:
            elapsed = time.time() - self.ramp_start_time
            if elapsed >= RAMP_DOWN_DURATION or base_rpm == 0.0:
                base_rpm = 0.0
                self.ramp_start_time = None
                self.ramp_start_rpm = 0.0
                self.ramp_target_rpm = 0.0
                self.ramping_down = False

                self.active_drive_mode = 'stop'

                if self.pending_drive_mode is not None:
                    self.active_drive_mode = self.pending_drive_mode
                    self.pending_drive_mode = None
                    self.start_ramp_up()
                    base_rpm = self.get_current_rpm()

        # Calculate drive RPMs strictly from active_drive_mode
        if self.active_drive_mode == 'forward':
            left_rpm = base_rpm
            right_rpm = base_rpm
        elif self.active_drive_mode == 'backward':
            left_rpm = -base_rpm
            right_rpm = -base_rpm
        elif self.active_drive_mode == 'spin_left':
            left_rpm = -base_rpm
            right_rpm = base_rpm
        elif self.active_drive_mode == 'spin_right':
            left_rpm = base_rpm
            right_rpm = -base_rpm
        else:
            left_rpm = 0.0
            right_rpm = 0.0

        self.current_left_rpm = left_rpm
        self.current_right_rpm = right_rpm

        # Update steer current RPMs
        for steer_id, val in self.steer_targets.items():
            target = MAX_STEER_RPM if val > 0 else (-MAX_STEER_RPM if val < 0 else 0.0)
            current = self.current_steer_rpm[steer_id]

            if current < target:
                self.current_steer_rpm[steer_id] = (
                    target if current == 0 and target > 0 else min(current + RAMP_RATE, target)
                )
            elif current > target:
                self.current_steer_rpm[steer_id] = (
                    target if current == 0 and target < 0 else max(current - RAMP_RATE, target)
                )

        # Round robin — send ONE motor per loop
        motor_id = self.all_motors[self.motor_index]
        self.motor_index = (self.motor_index + 1) % len(self.all_motors)

        if motor_id in DRIVE_MOTORS:
            if motor_id in [FL_DRIVE, RL_DRIVE]:
                self.vesc_set_rpm(motor_id, left_rpm)
            else:
                self.vesc_set_rpm(motor_id, right_rpm)
        else:
            self.vesc_set_rpm(motor_id, self.current_steer_rpm[motor_id], MAX_STEER_RPM)

        self.get_logger().info(
            f'Active: {self.active_drive_mode} | Requested: {self.requested_drive_mode} | '
            f'Pending: {self.pending_drive_mode} | RampDown: {self.ramping_down} | '
            f'ERPM: {base_rpm:.0f} | L: {left_rpm:.0f} R: {right_rpm:.0f}'
        )

    def destroy_node(self):
        self.get_logger().info('Shutting down — stopping all motors.')
        for drive_id in DRIVE_MOTORS:
            self.vesc_set_rpm(drive_id, 0)
        for steer_id in STEER_MOTORS:
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
