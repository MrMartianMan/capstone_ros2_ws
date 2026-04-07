#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO


class SprayerGpioNode(Node):
    """
    Shared GPIO sprayer node for all vision modes.

    Subscribes to:
    - /sprayer/disease_detected
    - /sprayer/weed_detected

    Row layouts:
    - single_row:
        disease -> disease_gpio_pin
        weed    -> weed_gpio_pin
    - double_row:
        disease -> weed_gpio_pin
        weed    -> disease_gpio_pin
    """

    def __init__(self):
        super().__init__('sprayer_gpio_node')

        self.declare_parameter('disease_gpio_pin', 13)
        self.declare_parameter('weed_gpio_pin', 32)
        self.declare_parameter('hold_time_sec', 3.0)
        self.declare_parameter('signal_delay_sec', 4.0)
        self.declare_parameter('active_high', True)
        self.declare_parameter('row_layout', 'single_row')

        self.disease_gpio_pin = int(self.get_parameter('disease_gpio_pin').value)
        self.weed_gpio_pin = int(self.get_parameter('weed_gpio_pin').value)
        self.hold_time_sec = float(self.get_parameter('hold_time_sec').value)
        self.signal_delay_sec = float(self.get_parameter('signal_delay_sec').value)
        self.active_high = bool(self.get_parameter('active_high').value)
        self.row_layout = str(self.get_parameter('row_layout').value).strip().lower()

        if self.row_layout == 'single_row':
            self.disease_output_pin = self.disease_gpio_pin
            self.weed_output_pin = self.weed_gpio_pin
        elif self.row_layout == 'double_row':
            self.disease_output_pin = self.weed_gpio_pin
            self.weed_output_pin = self.disease_gpio_pin
        else:
            self.get_logger().warn(
                f"Unknown row_layout='{self.row_layout}', defaulting to single_row"
            )
            self.row_layout = 'single_row'
            self.disease_output_pin = self.disease_gpio_pin
            self.weed_output_pin = self.weed_gpio_pin

        self.disease_active_until = 0.0
        self.weed_active_until = 0.0

        self.disease_output_state = False
        self.weed_output_state = False

        self.disease_events = []
        self.weed_events = []

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.disease_gpio_pin, GPIO.OUT)
        GPIO.setup(self.weed_gpio_pin, GPIO.OUT)

        self.output_off(self.disease_gpio_pin)
        self.output_off(self.weed_gpio_pin)

        self.create_subscription(
            Bool,
            '/sprayer/disease_detected',
            self.disease_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/sprayer/weed_detected',
            self.weed_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.update_outputs)

        self.get_logger().info(
            f'Sprayer GPIO node started | '
            f'row_layout={self.row_layout} | '
            f'configured disease pin={self.disease_gpio_pin} | '
            f'configured weed pin={self.weed_gpio_pin} | '
            f'actual disease output pin={self.disease_output_pin} | '
            f'actual weed output pin={self.weed_output_pin} | '
            f'hold_time_sec={self.hold_time_sec} | '
            f'signal_delay_sec={self.signal_delay_sec} | '
            f'active_high={self.active_high}'
        )

    def output_on(self, pin):
        GPIO.output(pin, GPIO.HIGH if self.active_high else GPIO.LOW)

    def output_off(self, pin):
        GPIO.output(pin, GPIO.LOW if self.active_high else GPIO.HIGH)

    def disease_callback(self, msg: Bool):
        if msg.data:
            activation_time = time.time() + self.signal_delay_sec
            self.disease_events.append(activation_time)
            self.get_logger().info(
                f'DISEASE detection received -> scheduled GPIO in {self.signal_delay_sec:.1f}s'
            )

    def weed_callback(self, msg: Bool):
        if msg.data:
            activation_time = time.time() + self.signal_delay_sec
            self.weed_events.append(activation_time)
            self.get_logger().info(
                f'WEED detection received -> scheduled GPIO in {self.signal_delay_sec:.1f}s'
            )

    def update_outputs(self):
        now = time.time()

        remaining_disease_events = []
        for activation_time in self.disease_events:
            if now >= activation_time:
                self.disease_active_until = now + self.hold_time_sec
                self.get_logger().info(
                    f'DISEASE delayed event matured -> hold ON for {self.hold_time_sec:.1f}s'
                )
            else:
                remaining_disease_events.append(activation_time)
        self.disease_events = remaining_disease_events

        remaining_weed_events = []
        for activation_time in self.weed_events:
            if now >= activation_time:
                self.weed_active_until = now + self.hold_time_sec
                self.get_logger().info(
                    f'WEED delayed event matured -> hold ON for {self.hold_time_sec:.1f}s'
                )
            else:
                remaining_weed_events.append(activation_time)
        self.weed_events = remaining_weed_events

        disease_should_be_on = now < self.disease_active_until
        weed_should_be_on = now < self.weed_active_until

        if disease_should_be_on and not self.disease_output_state:
            self.output_on(self.disease_output_pin)
            self.disease_output_state = True
            self.get_logger().info(
                f'DISEASE GPIO ON -> physical pin {self.disease_output_pin}'
            )
        elif not disease_should_be_on and self.disease_output_state:
            self.output_off(self.disease_output_pin)
            self.disease_output_state = False
            self.get_logger().info(
                f'DISEASE GPIO OFF -> physical pin {self.disease_output_pin}'
            )

        if weed_should_be_on and not self.weed_output_state:
            self.output_on(self.weed_output_pin)
            self.weed_output_state = True
            self.get_logger().info(
                f'WEED GPIO ON -> physical pin {self.weed_output_pin}'
            )
        elif not weed_should_be_on and self.weed_output_state:
            self.output_off(self.weed_output_pin)
            self.weed_output_state = False
            self.get_logger().info(
                f'WEED GPIO OFF -> physical pin {self.weed_output_pin}'
            )

    def destroy_node(self):
        self.output_off(self.disease_gpio_pin)
        self.output_off(self.weed_gpio_pin)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SprayerGpioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
