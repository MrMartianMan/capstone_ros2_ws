#!/usr/bin/env python3
import struct

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

        # -------- SLCAN --------
        self.slcan_port = '/dev/ttyACM0'
        self.slcan_tty_baud = 115200
        self.can_bitrate = 500000

        # -------- Drive CAN IDs --------
        self.fl_drive_id = 76
        self.fr_drive_id = 7
        self.rl_drive_id = 53
        self.rr_drive_id = 61

        # -------- Constant forward-speed row-following --------
        self.base_erpm = 1300
        self.min_erpm = 900
        self.turn_delta_limit = 200

        # angular.z magnitude that maps to full steering effect
        self.angular_full_scale = 0.1

        # -------- Motor sign correction --------
        # Change to -1 if one side spins backward.
        self.left_sign = 1
        self.right_sign = 1

        # -------- Safety --------
        self.cmd_timeout = 0.5
        self.centerline_timeout = 0.4

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

        self.last_left_erpm = None
        self.last_right_erpm = None

        self.bus = None
        self.can_available = False
        self._connect_can()

        self.timer = self.create_timer(0.05, self.control_loop)

        self.send_all_stop()

        self.get_logger().info(
            f'cmd_vel_to_can_node started on {self.slcan_port} '
            f'(tty_baud={self.slcan_tty_baud}, can_bitrate={self.can_bitrate})'
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
                channel=self.slcan_port,
                interface='slcan',
                tty_baudrate=self.slcan_tty_baud,
                bitrate=self.can_bitrate
            )
            self.can_available = True
            self.get_logger().info(
                f'CAN bus connected via python-can SLCAN on {self.slcan_port}'
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
            left_erpm = 0
            right_erpm = 0
        else:
            turn = self.angular_to_delta_erpm(self.latest_angular_z)

            # Positive angular.z = left turn
            # Negative angular.z = right turn
            # Turn by slowing one side down only
            if turn >= 0:
                left_erpm = self.base_erpm - turn
                right_erpm = self.base_erpm
            else:
                left_erpm = self.base_erpm
                right_erpm = self.base_erpm - abs(turn)

            left_erpm = self.clamp_int(left_erpm, self.min_erpm, self.base_erpm)
            right_erpm = self.clamp_int(right_erpm, self.min_erpm, self.base_erpm)

            left_erpm *= self.left_sign
            right_erpm *= self.right_sign

        self.send_vesc_rpm(self.fl_drive_id, left_erpm)
        self.send_vesc_rpm(self.rl_drive_id, left_erpm)
        self.send_vesc_rpm(self.fr_drive_id, right_erpm)
        self.send_vesc_rpm(self.rr_drive_id, right_erpm)

        if left_erpm != self.last_left_erpm or right_erpm != self.last_right_erpm:
            self.get_logger().info(
                f'centerline_valid={self.centerline_valid}, '
                f'angular_z={self.latest_angular_z:.3f} | '
                f'left_erpm={left_erpm}, right_erpm={right_erpm}'
            )
            self.last_left_erpm = left_erpm
            self.last_right_erpm = right_erpm

    def angular_to_delta_erpm(self, angular_z: float) -> int:
        if self.angular_full_scale <= 1e-6:
            return 0

        normalized = angular_z / self.angular_full_scale
        normalized = self.clamp_float(normalized, -1.0, 1.0)

        delta = int(round(normalized * self.turn_delta_limit))
        return delta

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

    @staticmethod
    def clamp_float(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def clamp_int(value: int, low: int, high: int) -> int:
        return max(low, min(high, value))

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
