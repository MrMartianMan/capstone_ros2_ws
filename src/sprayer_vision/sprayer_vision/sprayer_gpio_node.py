#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO


class MultiClassSprayerGpioNode(Node):
    def __init__(self):
        super().__init__('sprayer_gpio_node')

        self.declare_parameter('cell_phone_gpio_pin', 13)   # BOARD numbering
        self.declare_parameter('remote_gpio_pin', 32)        # BOARD numbering
        self.declare_parameter('hold_time_sec', 3.0)
        self.declare_parameter('active_high', True)         # <-- CHANGED BACK

        self.cell_phone_gpio_pin = int(self.get_parameter('cell_phone_gpio_pin').value)
        self.remote_gpio_pin = int(self.get_parameter('remote_gpio_pin').value)
        self.hold_time_sec = float(self.get_parameter('hold_time_sec').value)
        self.active_high = bool(self.get_parameter('active_high').value)

        self.cell_phone_active_until = 0.0
        self.remote_active_until = 0.0

        self.cell_phone_output_state = False
        self.remote_output_state = False

        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.cell_phone_gpio_pin, GPIO.OUT)
        GPIO.setup(self.remote_gpio_pin, GPIO.OUT)

        # Start both outputs OFF
        self.output_off(self.cell_phone_gpio_pin)
        self.output_off(self.remote_gpio_pin)

        self.create_subscription(
            Bool,
            '/sprayer/cell_phone_detected',
            self.cell_phone_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/sprayer/remote_detected',
            self.remote_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.update_outputs)

        self.get_logger().info(
            f'Multi GPIO node started: '
            f'cell_phone pin={self.cell_phone_gpio_pin}, '
            f'remote pin={self.remote_gpio_pin}, '
            f'hold_time_sec={self.hold_time_sec}, '
            f'active_high={self.active_high}'
        )

    def output_on(self, pin):
        # Active-high relay logic:
        # HIGH = relay ON
        GPIO.output(pin, GPIO.HIGH if self.active_high else GPIO.LOW)

    def output_off(self, pin):
        # Active-high relay logic:
        # LOW = relay OFF
        GPIO.output(pin, GPIO.LOW if self.active_high else GPIO.HIGH)

    def cell_phone_callback(self, msg: Bool):
        now = time.time()
        if msg.data:
            self.cell_phone_active_until = now + self.hold_time_sec

    def remote_callback(self, msg: Bool):
        now = time.time()
        if msg.data:
            self.remote_active_until = now + self.hold_time_sec

    def update_outputs(self):
        now = time.time()

        cell_phone_should_be_on = now < self.cell_phone_active_until
        remote_should_be_on = now < self.remote_active_until

        if cell_phone_should_be_on and not self.cell_phone_output_state:
            self.output_on(self.cell_phone_gpio_pin)
            self.cell_phone_output_state = True
            self.get_logger().info('CELL PHONE GPIO ON')

        elif not cell_phone_should_be_on and self.cell_phone_output_state:
            self.output_off(self.cell_phone_gpio_pin)
            self.cell_phone_output_state = False
            self.get_logger().info('CELL PHONE GPIO OFF')

        if remote_should_be_on and not self.remote_output_state:
            self.output_on(self.remote_gpio_pin)
            self.remote_output_state = True
            self.get_logger().info('remote GPIO ON')

        elif not remote_should_be_on and self.remote_output_state:
            self.output_off(self.remote_gpio_pin)
            self.remote_output_state = False
            self.get_logger().info('remote GPIO OFF')

    def destroy_node(self):
        self.output_off(self.cell_phone_gpio_pin)
        self.output_off(self.remote_gpio_pin)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiClassSprayerGpioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
