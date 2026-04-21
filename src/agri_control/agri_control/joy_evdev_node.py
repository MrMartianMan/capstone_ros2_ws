#!/usr/bin/env python3
"""
joy_evdev_node.py

Reads an evdev controller device directly and publishes sensor_msgs/Joy.
Designed to recover if the controller disconnects and later reconnects.

Features:
- Uses /dev/input/by-id/... path by default
- Publishes /joy at 50 Hz
- Zeros all outputs if controller disconnects
- Automatically retries opening the device
- Safely handles reconnects without restarting the node
"""

import threading
import time

import evdev
from evdev import InputDevice, ecodes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyEvdevNode(Node):
    def __init__(self):
        super().__init__('joy_evdev_node')

        self.declare_parameter(
            'device',
            '/dev/input/by-id/usb-Guangzhou_Chicken_Run_Network_Technology_Co.__Ltd._GameSir-G7_Pro-event-joystick'
        )
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('reconnect_interval_sec', 1.0)

        self.device_path = self.get_parameter('device').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.reconnect_interval_sec = float(self.get_parameter('reconnect_interval_sec').value)

        self.publisher = self.create_publisher(Joy, '/joy', 10)

        self.lock = threading.Lock()
        self.device = None
        self.device_connected = False
        self.stop_event = threading.Event()

        self._init_default_states()

        self.get_logger().info(f'Configured evdev device: {self.device_path}')

        self.reader_thread = threading.Thread(target=self.device_manager_loop, daemon=True)
        self.reader_thread.start()

        publish_period = 1.0 / self.publish_rate_hz
        self.create_timer(publish_period, self.publish_joy)

    def _init_default_states(self):
        """Set controller state to neutral/safe defaults."""
        self.axes = {
            ecodes.ABS_X: 127.0,      # Left stick X
            ecodes.ABS_Y: 127.0,      # Left stick Y
            ecodes.ABS_Z: 127.0,      # Right stick X
            ecodes.ABS_RZ: 127.0,     # Right stick Y
            ecodes.ABS_GAS: 0.0,      # Right trigger analog
            ecodes.ABS_BRAKE: 0.0,    # Left trigger analog
            ecodes.ABS_HAT0X: 0.0,    # D-pad X
            ecodes.ABS_HAT0Y: 0.0,    # D-pad Y
        }

        self.buttons = {
            ecodes.BTN_SOUTH: 0,      # A
            ecodes.BTN_EAST: 0,       # B
            ecodes.BTN_NORTH: 0,      # Y
            ecodes.BTN_WEST: 0,       # X
            ecodes.BTN_TL: 0,         # LB
            ecodes.BTN_TR: 0,         # RB
            ecodes.BTN_TL2: 0,        # LT digital
            ecodes.BTN_TR2: 0,        # RT digital
            ecodes.BTN_SELECT: 0,     # Select
            ecodes.BTN_START: 0,      # Start
            ecodes.BTN_MODE: 0,       # Home
            ecodes.BTN_THUMBL: 0,     # L3
            ecodes.BTN_THUMBR: 0,     # R3
        }

    def reset_state(self):
        """Force all published controller values to safe neutral."""
        with self.lock:
            self._init_default_states()

    def normalize_axis(self, value, min_val=0.0, max_val=255.0, center=127.0):
        """Normalize 0-255 style axis to about -1.0 to 1.0."""
        if value <= center:
            return -(center - value) / center
        return (value - center) / (max_val - center)

    def normalize_trigger(self, value, max_val=255.0):
        """Normalize trigger 0-255 to 0.0 to 1.0."""
        return value / max_val

    def try_open_device(self):
        """Attempt to open the evdev device."""
        try:
            dev = InputDevice(self.device_path)
            self.device = dev
            self.device_connected = True
            self.get_logger().info(f'Opened controller: {dev.name} at {self.device_path}')
            return True
        except Exception as e:
            self.device = None
            self.device_connected = False
            self.get_logger().warn(f'Could not open {self.device_path}: {e}')
            return False

    def close_device(self):
        """Close current device handle if present."""
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
        self.device = None
        self.device_connected = False

    def handle_disconnect(self, reason='unknown'):
        """Safely handle controller disconnect."""
        self.get_logger().warn(f'Controller disconnected: {reason}. Zeroing outputs and waiting to reconnect.')
        self.close_device()
        self.reset_state()

    def read_events_until_disconnect(self):
        """Read events until device disconnects or an error occurs."""
        if self.device is None:
            return

        try:
            for event in self.device.read_loop():
                if self.stop_event.is_set():
                    return

                with self.lock:
                    if event.type == ecodes.EV_ABS:
                        if event.code in self.axes:
                            self.axes[event.code] = float(event.value)
                    elif event.type == ecodes.EV_KEY:
                        if event.code in self.buttons:
                            self.buttons[event.code] = int(event.value)

        except OSError as e:
            self.handle_disconnect(reason=f'OSError: {e}')
        except Exception as e:
            self.handle_disconnect(reason=f'Exception: {e}')
        else:
            self.handle_disconnect(reason='read_loop exited')

    def device_manager_loop(self):
        """
        Keep trying to connect to the controller.
        Once connected, read until disconnect.
        Then retry.
        """
        while not self.stop_event.is_set():
            if not self.device_connected:
                opened = self.try_open_device()
                if not opened:
                    time.sleep(self.reconnect_interval_sec)
                    continue

            self.read_events_until_disconnect()

            if not self.stop_event.is_set():
                time.sleep(self.reconnect_interval_sec)

    def publish_joy(self):
        with self.lock:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'joy'

            # Axes order:
            # 0: LX, 1: LY, 2: RX, 3: RY, 4: RT, 5: LT, 6: DpadX, 7: DpadY
            msg.axes = [
                self.normalize_axis(self.axes[ecodes.ABS_X]),
                -self.normalize_axis(self.axes[ecodes.ABS_Y]),
                self.normalize_axis(self.axes[ecodes.ABS_Z]),
                -self.normalize_axis(self.axes[ecodes.ABS_RZ]),
                self.normalize_trigger(self.axes[ecodes.ABS_GAS]),
                self.normalize_trigger(self.axes[ecodes.ABS_BRAKE]),
                float(self.axes[ecodes.ABS_HAT0X]),
                -float(self.axes[ecodes.ABS_HAT0Y]),
            ]

            # Buttons order:
            # A, B, Y, X, LB, RB, LT, RT, Select, Start, Home, L3, R3
            msg.buttons = [
                self.buttons[ecodes.BTN_SOUTH],
                self.buttons[ecodes.BTN_EAST],
                self.buttons[ecodes.BTN_NORTH],
                self.buttons[ecodes.BTN_WEST],
                self.buttons[ecodes.BTN_TL],
                self.buttons[ecodes.BTN_TR],
                self.buttons[ecodes.BTN_TL2],
                self.buttons[ecodes.BTN_TR2],
                self.buttons[ecodes.BTN_SELECT],
                self.buttons[ecodes.BTN_START],
                self.buttons[ecodes.BTN_MODE],
                self.buttons[ecodes.BTN_THUMBL],
                self.buttons[ecodes.BTN_THUMBR],
            ]

        self.publisher.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        self.close_device()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyEvdevNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
