#!/usr/bin/env python3
import struct
import time
import math

import can
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelToCanNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_can_node')

        # ============================================================
        # EASY-EDIT SECTION
        # ============================================================

        # -------- SocketCAN --------
        self.can_interface = 'can2'

        # -------- Drive CAN IDs --------
        self.fl_drive_id = 76
        self.fr_drive_id = 7
        self.rl_drive_id = 53
        self.rr_drive_id = 61

        # -------- Constant forward-speed row-following --------
        self.base_erpm = 4500
        self.min_erpm = 900
        self.turn_delta_limit = 1800

        # -------- Turn smoothing (rate limiter) --------
        # Max change in turn ERPM difference per second
        self.turn_rate_limit_per_sec = 2500.0

        # -------- Restart ramp-up settings --------
        self.start_erpm = 900
        self.startup_ramp_duration = 1.0

        # -------- Safety ramp-down settings --------
        self.stop_ramp_duration = 1.0

        # angular.z magnitude that maps to full steering effect
        self.angular_full_scale = 0.05

        # -------- Motor sign correction --------
        # Change to -1 if one side spins backward.
        self.left_sign = 1
        self.right_sign = 1

        # -------- Safety --------
        self.cmd_timeout = 0.5
        self.centerline_timeout = 0.4

        # -------- Loop period --------
        self.control_period = 0.02  # 50 Hz

        # ============================================================
        # END EASY-EDIT SECTION
        # ============================================================

        self.CAN_PACKET_SET_RPM = 3

        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.centerline_valid_subscription = self.create_subscription(
            Bool,
            '/centerline_valid',
            self.centerline_valid_callback,
            10
        )

        self.last_cmd_time = self.get_clock().now()
        self.last_centerline_time = self.get_clock().now()
        self.centerline_valid = False
        self.latest_angular_z = 0.0

        self.last_left_erpm = 0
        self.last_right_erpm = 0

        # Track whether robot is fully stopped
        self.was_stopped = True

        # Startup ramp state
        self.startup_ramp_active = False
        self.startup_ramp_start_time = None
        self.startup_ramp_start_left = 0
        self.startup_ramp_start_right = 0

        # Stop ramp state
        self.stop_ramp_active = False
        self.stop_ramp_start_time = None
        self.stop_ramp_start_left = 0
        self.stop_ramp_start_right = 0

        # Smoothed turn state
        self.current_turn_delta = 0.0

        self.bus = None
        self.can_available = False
        self._connect_can()

        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.send_all_stop()

        self.get_logger().info(
            f'cmd_vel_to_can_node started on {self.can_interface}'
        )
        self.get_logger().info(
            f'Constant row-follow mode: base_erpm={self.base_erpm}, '
            f'min_erpm={self.min_erpm}, turn_delta_limit={self.turn_delta_limit}'
        )

        if not self.can_available:
            self.get_logger().warn('CAN bus not available — running without CAN output')

    def _connect_can(self):
        try:
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                interface='socketcan'
            )
            self.can_available = True
            self.get_logger().info(
                f'CAN bus connected via SocketCAN on {self.can_interface}'
            )
        except Exception as e:
            self.bus = None
            self.can_available = False
            self.get_logger().warn(f'CAN not connected: {e}')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.latest_angular_z = float(msg.angular.z)

    def centerline_valid_callback(self, msg: Bool):
        self.centerline_valid = bool(msg.data)
        if self.centerline_valid:
            self.last_centerline_time = self.get_clock().now()

    @staticmethod
    def clamp_float(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def clamp_int(value: int, low: int, high: int) -> int:
        return max(low, min(high, value))

    @staticmethod
    def s_curve(t: float) -> float:
        t = max(0.0, min(1.0, t))
        return (1.0 - math.cos(math.pi * t)) / 2.0

    def angular_to_target_delta_erpm(self, angular_z: float) -> float:
        if self.angular_full_scale <= 1e-6:
            return 0.0

        normalized = angular_z / self.angular_full_scale
        normalized = self.clamp_float(normalized, -1.0, 1.0)

        delta = normalized * self.turn_delta_limit
        return float(delta)

    def update_smoothed_turn_delta(self) -> int:
        target_turn = self.angular_to_target_delta_erpm(self.latest_angular_z)

        max_step = self.turn_rate_limit_per_sec * self.control_period
        delta = target_turn - self.current_turn_delta
        delta = self.clamp_float(delta, -max_step, max_step)

        self.current_turn_delta += delta

        # snap tiny values to zero
        if abs(self.current_turn_delta) < 0.5:
            self.current_turn_delta = 0.0

        return int(round(self.current_turn_delta))

    def compute_target_erpm(self, current_base_erpm: int):
        current_min_erpm = min(self.min_erpm, current_base_erpm)
        turn = self.update_smoothed_turn_delta()

        # Positive angular.z = left turn
        # Negative angular.z = right turn
        # Turn by slowing one side down only
        if turn >= 0:
            left_erpm = current_base_erpm - turn
            right_erpm = current_base_erpm
        else:
            left_erpm = current_base_erpm
            right_erpm = current_base_erpm - abs(turn)

        left_erpm = self.clamp_int(left_erpm, current_min_erpm, current_base_erpm)
        right_erpm = self.clamp_int(right_erpm, current_min_erpm, current_base_erpm)

        left_erpm *= self.left_sign
        right_erpm *= self.right_sign

        return left_erpm, right_erpm

    def start_startup_ramp(self, start_left: int, start_right: int):
        self.startup_ramp_active = True
        self.startup_ramp_start_time = time.time()
        self.startup_ramp_start_left = int(start_left)
        self.startup_ramp_start_right = int(start_right)

    def get_startup_ramp_erpm(self):
        if not self.startup_ramp_active or self.startup_ramp_start_time is None:
            target_left, target_right = self.compute_target_erpm(self.base_erpm)
            return target_left, target_right

        elapsed = time.time() - self.startup_ramp_start_time
        if elapsed >= self.startup_ramp_duration:
            self.startup_ramp_active = False
            self.startup_ramp_start_time = None
            target_left, target_right = self.compute_target_erpm(self.base_erpm)
            return target_left, target_right

        start_left_mag = abs(self.startup_ramp_start_left)
        start_right_mag = abs(self.startup_ramp_start_right)

        if start_left_mag == 0 and start_right_mag == 0:
            start_base = self.start_erpm
        else:
            start_base = max(start_left_mag, start_right_mag)

        t = elapsed / self.startup_ramp_duration
        factor = self.s_curve(t)

        current_base = int(round(start_base + (self.base_erpm - start_base) * factor))
        target_left, target_right = self.compute_target_erpm(current_base)
        return target_left, target_right

    def start_stop_ramp(self, start_left: int, start_right: int):
        self.stop_ramp_active = True
        self.stop_ramp_start_time = time.time()
        self.stop_ramp_start_left = int(start_left)
        self.stop_ramp_start_right = int(start_right)

    def get_stop_ramp_erpm(self):
        if not self.stop_ramp_active or self.stop_ramp_start_time is None:
            return 0, 0

        elapsed = time.time() - self.stop_ramp_start_time
        if elapsed >= self.stop_ramp_duration:
            self.stop_ramp_active = False
            self.stop_ramp_start_time = None
            return 0, 0

        t = elapsed / self.stop_ramp_duration
        factor = self.s_curve(t)

        left = int(round(self.stop_ramp_start_left * (1.0 - factor)))
        right = int(round(self.stop_ramp_start_right * (1.0 - factor)))

        if abs(left) < 20:
            left = 0
        if abs(right) < 20:
            right = 0

        return left, right

    def vesc_set_rpm_eid(self, controller_id: int) -> int:
        return (self.CAN_PACKET_SET_RPM << 8) | (controller_id & 0xFF)

    def send_vesc_rpm(self, controller_id: int, erpm: int):
        if not self.can_available or self.bus is None:
            return

        eid = self.vesc_set_rpm_eid(controller_id)
        data = struct.pack('>i', int(erpm))

        msg = can.Message(
            arbitration_id=eid,
            is_extended_id=True,
            data=data
        )

        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(
                f'CAN send failed: controller_id={controller_id}, '
                f'eid=0x{eid:X}, error={e}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Unexpected CAN error: controller_id={controller_id}, '
                f'eid=0x{eid:X}, error={e}'
            )

    def send_all_stop(self):
        self.send_vesc_rpm(self.fl_drive_id, 0)
        self.send_vesc_rpm(self.fr_drive_id, 0)
        self.send_vesc_rpm(self.rl_drive_id, 0)
        self.send_vesc_rpm(self.rr_drive_id, 0)

    def control_loop(self):
        now = self.get_clock().now()

        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        centerline_age = (now - self.last_centerline_time).nanoseconds / 1e9

        should_stop = (
            cmd_age > self.cmd_timeout or
            (not self.centerline_valid) or
            centerline_age > self.centerline_timeout
        )

        if should_stop:
            if not self.was_stopped and not self.stop_ramp_active:
                self.start_stop_ramp(self.last_left_erpm, self.last_right_erpm)

            # Cancel startup ramp while stopping
            self.startup_ramp_active = False
            self.startup_ramp_start_time = None

            # Also decay steering memory toward zero while stopped
            max_turn_step = self.turn_rate_limit_per_sec * self.control_period
            if self.current_turn_delta > 0:
                self.current_turn_delta = max(0.0, self.current_turn_delta - max_turn_step)
            elif self.current_turn_delta < 0:
                self.current_turn_delta = min(0.0, self.current_turn_delta + max_turn_step)

            left_erpm, right_erpm = self.get_stop_ramp_erpm()

            if left_erpm == 0 and right_erpm == 0:
                self.was_stopped = True

        else:
            if self.was_stopped:
                self.start_startup_ramp(0, 0)
                self.was_stopped = False
            elif self.stop_ramp_active:
                current_left, current_right = self.get_stop_ramp_erpm()
                self.stop_ramp_active = False
                self.stop_ramp_start_time = None
                self.start_startup_ramp(current_left, current_right)

            left_erpm, right_erpm = self.get_startup_ramp_erpm()

        self.send_vesc_rpm(self.fl_drive_id, left_erpm)
        self.send_vesc_rpm(self.rl_drive_id, left_erpm)
        self.send_vesc_rpm(self.fr_drive_id, right_erpm)
        self.send_vesc_rpm(self.rr_drive_id, right_erpm)

        if left_erpm != self.last_left_erpm or right_erpm != self.last_right_erpm:
            self.get_logger().info(
                f'centerline_valid={self.centerline_valid}, '
                f'angular_z={self.latest_angular_z:.3f} | '
                f'turn_delta={self.current_turn_delta:.1f} | '
                f'left_erpm={left_erpm}, right_erpm={right_erpm}'
            )
            self.last_left_erpm = left_erpm
            self.last_right_erpm = right_erpm

    def destroy_node(self):
        try:
            self.get_logger().info('Shutting down — stopping all drive motors.')
            self.send_all_stop()
        except Exception:
            pass

        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
