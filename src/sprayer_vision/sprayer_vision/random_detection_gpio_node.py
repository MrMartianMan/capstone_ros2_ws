#!/usr/bin/env python3

import time
import random
import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO


class RandomDetectionGpioNode(Node):
    """
    This node simulates detection events (Disease and Weed) and drives GPIO outputs
    as if they were coming from a real YOLO detection pipeline.

    Key behavior:
    - Generates random detection events for Disease and Weed independently
    - Each detection is delayed (to simulate camera → sprayer offset)
    - Each output stays ON for a fixed hold time (like spray duration)
    - If multiple detections overlap, the output stays ON continuously
    """

    def __init__(self):
        super().__init__('random_detection_gpio_node')

        # GPIO pin assignments (BOARD numbering)
        self.declare_parameter('disease_gpio_pin', 12)
        self.declare_parameter('weed_gpio_pin', 16)

        # Relay behavior
        self.declare_parameter('active_high', True)

        # How long the sprayer stays ON after a detection
        self.declare_parameter('hold_time_sec', 3.0)

        # Delay between detection and actuation (camera → nozzle offset)
        self.declare_parameter('signal_delay_sec', 4.0)

        # Random timing bounds for detection generation
        self.declare_parameter('disease_min_interval_sec', 0.5)
        self.declare_parameter('disease_max_interval_sec', 10.0)
        self.declare_parameter('weed_min_interval_sec', 0.5)
        self.declare_parameter('weed_max_interval_sec', 10.0)

        # Load parameters
        self.disease_gpio_pin = int(self.get_parameter('disease_gpio_pin').value)
        self.weed_gpio_pin = int(self.get_parameter('weed_gpio_pin').value)
        self.active_high = bool(self.get_parameter('active_high').value)
        self.hold_time_sec = float(self.get_parameter('hold_time_sec').value)
        self.signal_delay_sec = float(self.get_parameter('signal_delay_sec').value)

        self.disease_min_interval_sec = float(self.get_parameter('disease_min_interval_sec').value)
        self.disease_max_interval_sec = float(self.get_parameter('disease_max_interval_sec').value)
        self.weed_min_interval_sec = float(self.get_parameter('weed_min_interval_sec').value)
        self.weed_max_interval_sec = float(self.get_parameter('weed_max_interval_sec').value)

        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.disease_gpio_pin, GPIO.OUT)
        GPIO.setup(self.weed_gpio_pin, GPIO.OUT)

        # Ensure outputs start OFF
        self.output_off(self.disease_gpio_pin)
        self.output_off(self.weed_gpio_pin)

        now = time.time()

        # Tracks how long each output should remain active
        self.disease_active_until = 0.0
        self.weed_active_until = 0.0

        # Schedule first random detections
        self.next_disease_trigger_time = now + self.random_disease_interval()
        self.next_weed_trigger_time = now + self.random_weed_interval()

        # Track current GPIO states
        self.disease_output_state = False
        self.weed_output_state = False

        # Queue of delayed detection events (time, type)
        self.scheduled_events = []

        # Main update loop (runs at ~50 Hz)
        self.timer = self.create_timer(0.02, self.update_loop)

        self.get_logger().info("Random Detection GPIO Node started")

    def random_disease_interval(self):
        """Generate a random time until the next disease detection."""
        return random.uniform(self.disease_min_interval_sec, self.disease_max_interval_sec)

    def random_weed_interval(self):
        """Generate a random time until the next weed detection."""
        return random.uniform(self.weed_min_interval_sec, self.weed_max_interval_sec)

    def output_on(self, pin):
        """Turn a GPIO output ON based on relay polarity."""
        GPIO.output(pin, GPIO.HIGH if self.active_high else GPIO.LOW)

    def output_off(self, pin):
        """Turn a GPIO output OFF based on relay polarity."""
        GPIO.output(pin, GPIO.LOW if self.active_high else GPIO.HIGH)

    def schedule_event(self, event_type, now):
        """
        Instead of triggering immediately, detections are scheduled in the future.
        This simulates the delay between camera detection and sprayer position.
        """
        activation_time = now + self.signal_delay_sec
        self.scheduled_events.append((activation_time, event_type))

        self.get_logger().info(
            f"{event_type.upper()} detection scheduled in {self.signal_delay_sec:.1f}s"
        )

    def activate_event(self, event_type, now):
        """
        When the delay expires, activate the corresponding output.
        This starts the hold timer for the sprayer.
        """
        if event_type == "disease":
            self.disease_active_until = now + self.hold_time_sec
            self.get_logger().info("DISEASE SPRAY ON")

        elif event_type == "weed":
            self.weed_active_until = now + self.hold_time_sec
            self.get_logger().info("WEED SPRAY ON")

    def update_loop(self):
        """
        Main loop:
        1. Generate new random detection events
        2. Activate delayed events when their time arrives
        3. Update GPIO outputs based on active timers
        """
        now = time.time()

        # Step 1: Generate new detection events (scheduled for future activation)
        if now >= self.next_disease_trigger_time:
            self.schedule_event("disease", now)
            self.next_disease_trigger_time = now + self.random_disease_interval()

        if now >= self.next_weed_trigger_time:
            self.schedule_event("weed", now)
            self.next_weed_trigger_time = now + self.random_weed_interval()

        # Step 2: Process scheduled events whose delay has expired
        remaining_events = []
        for activation_time, event_type in self.scheduled_events:
            if now >= activation_time:
                self.activate_event(event_type, now)
            else:
                remaining_events.append((activation_time, event_type))

        self.scheduled_events = remaining_events

        # Step 3: Determine whether outputs should be ON
        disease_should_be_on = now < self.disease_active_until
        weed_should_be_on = now < self.weed_active_until

        # Update disease GPIO
        if disease_should_be_on and not self.disease_output_state:
            self.output_on(self.disease_gpio_pin)
            self.disease_output_state = True

        elif not disease_should_be_on and self.disease_output_state:
            self.output_off(self.disease_gpio_pin)
            self.disease_output_state = False
            self.get_logger().info("DISEASE SPRAY OFF")

        # Update weed GPIO
        if weed_should_be_on and not self.weed_output_state:
            self.output_on(self.weed_gpio_pin)
            self.weed_output_state = True

        elif not weed_should_be_on and self.weed_output_state:
            self.output_off(self.weed_gpio_pin)
            self.weed_output_state = False
            self.get_logger().info("WEED SPRAY OFF")

    def destroy_node(self):
        """Ensure GPIO is safely turned off when shutting down."""
        self.output_off(self.disease_gpio_pin)
        self.output_off(self.weed_gpio_pin)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomDetectionGpioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
