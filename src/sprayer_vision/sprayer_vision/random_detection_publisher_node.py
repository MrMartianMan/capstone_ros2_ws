#!/usr/bin/env python3

import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class RandomDetectionPublisherNode(Node):
    """
    Random standardized detection publisher.

    Publishes short True pulses on:
    - /sprayer/disease_detected
    - /sprayer/disease_confidence
    - /sprayer/weed_detected
    - /sprayer/weed_confidence

    The shared GPIO node handles delay/hold timing.
    """

    def __init__(self):
        super().__init__('random_detection_publisher_node')

        self.declare_parameter('disease_min_interval_sec', 0.5)
        self.declare_parameter('disease_max_interval_sec', 10.0)
        self.declare_parameter('weed_min_interval_sec', 0.5)
        self.declare_parameter('weed_max_interval_sec', 10.0)
        self.declare_parameter('detection_pulse_sec', 0.10)

        self.disease_min_interval_sec = float(self.get_parameter('disease_min_interval_sec').value)
        self.disease_max_interval_sec = float(self.get_parameter('disease_max_interval_sec').value)
        self.weed_min_interval_sec = float(self.get_parameter('weed_min_interval_sec').value)
        self.weed_max_interval_sec = float(self.get_parameter('weed_max_interval_sec').value)
        self.detection_pulse_sec = float(self.get_parameter('detection_pulse_sec').value)

        self.disease_pub = self.create_publisher(Bool, '/sprayer/disease_detected', 10)
        self.disease_conf_pub = self.create_publisher(Float32, '/sprayer/disease_confidence', 10)

        self.weed_pub = self.create_publisher(Bool, '/sprayer/weed_detected', 10)
        self.weed_conf_pub = self.create_publisher(Float32, '/sprayer/weed_confidence', 10)

        now = time.time()

        self.next_disease_trigger_time = now + self.random_disease_interval()
        self.next_weed_trigger_time = now + self.random_weed_interval()

        self.disease_pulse_end_time = 0.0
        self.weed_pulse_end_time = 0.0

        self.disease_pulse_active = False
        self.weed_pulse_active = False

        self.timer = self.create_timer(0.02, self.update_loop)

        self.get_logger().info('Random Detection Publisher Node started')

    def random_disease_interval(self):
        return random.uniform(self.disease_min_interval_sec, self.disease_max_interval_sec)

    def random_weed_interval(self):
        return random.uniform(self.weed_min_interval_sec, self.weed_max_interval_sec)

    def update_loop(self):
        now = time.time()

        if now >= self.next_disease_trigger_time:
            self.disease_pub.publish(Bool(data=True))
            self.disease_conf_pub.publish(Float32(data=1.0))
            self.disease_pulse_active = True
            self.disease_pulse_end_time = now + self.detection_pulse_sec
            self.next_disease_trigger_time = now + self.random_disease_interval()
            self.get_logger().info('Random DISEASE detection published')

        if now >= self.next_weed_trigger_time:
            self.weed_pub.publish(Bool(data=True))
            self.weed_conf_pub.publish(Float32(data=1.0))
            self.weed_pulse_active = True
            self.weed_pulse_end_time = now + self.detection_pulse_sec
            self.next_weed_trigger_time = now + self.random_weed_interval()
            self.get_logger().info('Random WEED detection published')

        if self.disease_pulse_active and now >= self.disease_pulse_end_time:
            self.disease_pub.publish(Bool(data=False))
            self.disease_conf_pub.publish(Float32(data=0.0))
            self.disease_pulse_active = False

        if self.weed_pulse_active and now >= self.weed_pulse_end_time:
            self.weed_pub.publish(Bool(data=False))
            self.weed_conf_pub.publish(Float32(data=0.0))
            self.weed_pulse_active = False


def main(args=None):
    rclpy.init(args=args)
    node = RandomDetectionPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
