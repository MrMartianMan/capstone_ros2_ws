#!/usr/bin/env python3
"""
joy_evdev_node.py
Reads 8BitDo Ultimate 2C via evdev (event0) and publishes sensor_msgs/Joy.
Bypasses the need for joydev/js0 kernel module.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import evdev
from evdev import InputDevice, ecodes
import threading


class JoyEvdevNode(Node):
    def __init__(self):
        super().__init__('joy_evdev_node')

        self.declare_parameter('device', '/dev/input/by-id/usb-8BitDo_8BitDo_Ultimate_2C_Wireless_Controller_562252A6BD-event-joystick')
        device_path = self.get_parameter('device').value

        self.publisher = self.create_publisher(Joy, '/joy', 10)

        # Axis state — center is 127 for 0–255 range
        self.axes = {
            ecodes.ABS_X:     127.0,  # Left stick X
            ecodes.ABS_Y:     127.0,  # Left stick Y
            ecodes.ABS_Z:     127.0,  # Right stick X
            ecodes.ABS_RZ:    127.0,  # Right stick Y
            ecodes.ABS_GAS:     0.0,  # Right trigger
            ecodes.ABS_BRAKE:   0.0,  # Left trigger
            ecodes.ABS_HAT0X:   0.0,  # D-pad X
            ecodes.ABS_HAT0Y:   0.0,  # D-pad Y
        }

        # Button state
        self.buttons = {
            ecodes.BTN_SOUTH:   0,  # A
            ecodes.BTN_EAST:    0,  # B
            ecodes.BTN_NORTH:   0,  # Y
            ecodes.BTN_WEST:    0,  # X
            ecodes.BTN_TL:      0,  # LB
            ecodes.BTN_TR:      0,  # RB
            ecodes.BTN_TL2:     0,  # LT digital
            ecodes.BTN_TR2:     0,  # RT digital
            ecodes.BTN_SELECT:  0,  # Select
            ecodes.BTN_START:   0,  # Start
            ecodes.BTN_MODE:    0,  # Home
            ecodes.BTN_THUMBL:  0,  # L3
            ecodes.BTN_THUMBR:  0,  # R3
        }

        # Open device
        try:
            self.device = InputDevice(device_path)
            self.get_logger().info(f'Opened: {self.device.name} at {device_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {device_path}: {e}')
            return

        # Read events in background thread
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        # Publish at 50hz
        self.create_timer(0.02, self.publish_joy)

    def normalize_axis(self, value, min_val=0, max_val=255, center=127):
        """Normalize 0-255 → -1.0 to 1.0"""
        if value <= center:
            return -(center - value) / center
        else:
            return (value - center) / (max_val - center)

    def normalize_trigger(self, value, max_val=255):
        """Normalize trigger 0-255 → 0.0 to 1.0"""
        return value / max_val

    def read_loop(self):
        for event in self.device.read_loop():
            if event.type == ecodes.EV_ABS:
                if event.code in self.axes:
                    self.axes[event.code] = float(event.value)
            elif event.type == ecodes.EV_KEY:
                if event.code in self.buttons:
                    self.buttons[event.code] = event.value

    def publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joy'

        # Axes order: LX, LY, RX, RY, RT, LT, DpadX, DpadY
        msg.axes = [
            self.normalize_axis(self.axes[ecodes.ABS_X]),
            -self.normalize_axis(self.axes[ecodes.ABS_Y]),   # invert Y
            self.normalize_axis(self.axes[ecodes.ABS_Z]),
            -self.normalize_axis(self.axes[ecodes.ABS_RZ]),  # invert Y
            self.normalize_trigger(self.axes[ecodes.ABS_GAS]),
            self.normalize_trigger(self.axes[ecodes.ABS_BRAKE]),
            self.axes[ecodes.ABS_HAT0X],
            -self.axes[ecodes.ABS_HAT0Y],
        ]

        # Buttons order: A, B, Y, X, LB, RB, LT, RT, Select, Start, Home, L3, R3
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
