#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO


class MultiClassSprayerGpioNode(Node):
    """
    This node receives detection topics and drives two GPIO outputs.

    Behavior:
    - A detection does not activate the relay immediately.
    - Each detection is delayed by signal_delay_sec before it affects the output.
    - Once the delayed event becomes active, the corresponding relay stays ON
      for hold_time_sec.
    - If another detection arrives, another delayed event is queued.
      When that later event matures, it extends the ON time again.
    """

    def __init__(self):
        super().__init__('sprayer_gpio_node')

        # GPIO pins use BOARD numbering
        self.declare_parameter('cell_phone_gpio_pin', 13)
        self.declare_parameter('remote_gpio_pin', 32)

        # Time the relay stays on after an activation
        self.declare_parameter('hold_time_sec', 3.0)

        # Delay between detection and sprayer actuation
        self.declare_parameter('signal_delay_sec', 4.0)

        # Relay polarity
        self.declare_parameter('active_high', True)

        self.cell_phone_gpio_pin = int(self.get_parameter('cell_phone_gpio_pin').value)
        self.remote_gpio_pin = int(self.get_parameter('remote_gpio_pin').value)
        self.hold_time_sec = float(self.get_parameter('hold_time_sec').value)
        self.signal_delay_sec = float(self.get_parameter('signal_delay_sec').value)
        self.active_high = bool(self.get_parameter('active_high').value)

        # Tracks how long each output should remain active
        self.cell_phone_active_until = 0.0
        self.remote_active_until = 0.0

        # Tracks current output state to avoid spamming GPIO writes/logs
        self.cell_phone_output_state = False
        self.remote_output_state = False

        # Queues of delayed activation times
        self.cell_phone_events = []
        self.remote_events = []

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

        # Main loop checks for matured delayed events and updates outputs
        self.timer = self.create_timer(0.02, self.update_outputs)

        self.get_logger().info(
            f'Multi GPIO node started: '
            f'cell_phone pin={self.cell_phone_gpio_pin}, '
            f'remote pin={self.remote_gpio_pin}, '
            f'hold_time_sec={self.hold_time_sec}, '
            f'signal_delay_sec={self.signal_delay_sec}, '
            f'active_high={self.active_high}'
        )

    def output_on(self, pin):
        """Drive a relay output ON using the configured polarity."""
        GPIO.output(pin, GPIO.HIGH if self.active_high else GPIO.LOW)

    def output_off(self, pin):
        """Drive a relay output OFF using the configured polarity."""
        GPIO.output(pin, GPIO.LOW if self.active_high else GPIO.HIGH)

    def cell_phone_callback(self, msg: Bool):
        """
        When a cell phone detection arrives, schedule a future activation
        instead of turning the output on immediately.
        """
        if msg.data:
            activation_time = time.time() + self.signal_delay_sec
            self.cell_phone_events.append(activation_time)
            self.get_logger().info(
                f'CELL PHONE detection received -> scheduled GPIO in {self.signal_delay_sec:.1f}s'
            )

    def remote_callback(self, msg: Bool):
        """
        When a remote detection arrives, schedule a future activation
        instead of turning the output on immediately.
        """
        if msg.data:
            activation_time = time.time() + self.signal_delay_sec
            self.remote_events.append(activation_time)
            self.get_logger().info(
                f'REMOTE detection received -> scheduled GPIO in {self.signal_delay_sec:.1f}s'
            )

    def update_outputs(self):
        """
        Main output update loop:
        1. Convert delayed events into output hold windows when their delay expires
        2. Turn outputs ON/OFF depending on whether they are still inside the hold window
        """
        now = time.time()

        # Process delayed cell phone events
        remaining_cell_phone_events = []
        for activation_time in self.cell_phone_events:
            if now >= activation_time:
                self.cell_phone_active_until = now + self.hold_time_sec
                self.get_logger().info(
                    f'CELL PHONE delayed event matured -> hold ON for {self.hold_time_sec:.1f}s'
                )
            else:
                remaining_cell_phone_events.append(activation_time)
        self.cell_phone_events = remaining_cell_phone_events

        # Process delayed remote events
        remaining_remote_events = []
        for activation_time in self.remote_events:
            if now >= activation_time:
                self.remote_active_until = now + self.hold_time_sec
                self.get_logger().info(
                    f'REMOTE delayed event matured -> hold ON for {self.hold_time_sec:.1f}s'
                )
            else:
                remaining_remote_events.append(activation_time)
        self.remote_events = remaining_remote_events

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
            self.get_logger().info('REMOTE GPIO ON')

        elif not remote_should_be_on and self.remote_output_state:
            self.output_off(self.remote_gpio_pin)
            self.remote_output_state = False
            self.get_logger().info('REMOTE GPIO OFF')

    def destroy_node(self):
        """Safely turn outputs off when shutting down."""
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
