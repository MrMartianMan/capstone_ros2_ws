#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class GroundCropNode(Node):
    """
    Crops the camera image to the center ground region between the robot legs.

    Intended use:
      /image_raw -> ground_crop_node -> /image_crop_between_legs -> vision node

    Default behavior:
    - Computes a centered horizontal crop using:
        mount_height_in
        desired_ground_width_in
        hfov_deg
    - Keeps the full image height by default.
    - Supports optional manual x offset and extra crop margin.
    - Can optionally resize the cropped image before publishing.

    For a fixed robot/camera setup, this is usually the simplest and most robust
    way to keep the models focused only on the useful ground region.
    """

    def __init__(self):
        super().__init__('ground_crop_node')

        # Topics
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('cropped_image_topic', '/image_crop_between_legs')

        # Geometry-based crop parameters
        self.declare_parameter('mount_height_in', 85.25)
        self.declare_parameter('desired_ground_width_in', 73.0)
        self.declare_parameter('hfov_deg', 88.0)

        # Manual tuning
        self.declare_parameter('crop_center_x_offset_px', 0)
        self.declare_parameter('crop_margin_px', 0)

        # Vertical crop controls
        self.declare_parameter('full_height', True)
        self.declare_parameter('y_min_px', 0)
        self.declare_parameter('y_max_px', -1)

        # Optional resize after cropping
        self.declare_parameter('resize_width_px', 0)
        self.declare_parameter('resize_height_px', 0)

        # Logging
        self.declare_parameter('log_crop_once', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cropped_image_topic = str(self.get_parameter('cropped_image_topic').value)

        self.mount_height_in = float(self.get_parameter('mount_height_in').value)
        self.desired_ground_width_in = float(
            self.get_parameter('desired_ground_width_in').value
        )
        self.hfov_deg = float(self.get_parameter('hfov_deg').value)

        self.crop_center_x_offset_px = int(
            self.get_parameter('crop_center_x_offset_px').value
        )
        self.crop_margin_px = int(self.get_parameter('crop_margin_px').value)

        self.full_height = bool(self.get_parameter('full_height').value)
        self.y_min_px = int(self.get_parameter('y_min_px').value)
        self.y_max_px = int(self.get_parameter('y_max_px').value)

        self.resize_width_px = int(self.get_parameter('resize_width_px').value)
        self.resize_height_px = int(self.get_parameter('resize_height_px').value)

        self.log_crop_once = bool(self.get_parameter('log_crop_once').value)
        self.has_logged_crop = False

        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        self.cropped_pub = self.create_publisher(
            Image,
            self.cropped_image_topic,
            sensor_qos
        )

        self.get_logger().info(f'Ground crop node subscribed to {self.image_topic}')
        self.get_logger().info(
            f'Publishing cropped images to {self.cropped_image_topic}'
        )

    def compute_crop_bounds(self, image_width_px: int, image_height_px: int):
        """
        Compute horizontal crop from physical geometry, then apply manual tuning.
        """

        hfov_rad = math.radians(self.hfov_deg)
        ground_width_seen_in = 2.0 * self.mount_height_in * math.tan(hfov_rad / 2.0)

        if ground_width_seen_in <= 0.0:
            raise ValueError('Computed ground width is non-positive.')

        crop_fraction = self.desired_ground_width_in / ground_width_seen_in

        # Clamp to valid range
        crop_fraction = max(0.0, min(1.0, crop_fraction))

        crop_width_px = int(round(image_width_px * crop_fraction))

        # Keep at least 1 pixel and no more than full width
        crop_width_px = max(1, min(image_width_px, crop_width_px))

        center_x = (image_width_px // 2) + self.crop_center_x_offset_px

        x_min = center_x - (crop_width_px // 2)
        x_max = x_min + crop_width_px

        # Expand or shrink with manual margin
        x_min -= self.crop_margin_px
        x_max += self.crop_margin_px

        # Clamp to image bounds
        x_min = max(0, x_min)
        x_max = min(image_width_px, x_max)

        # If clamping distorted width too much, fix it safely
        if x_max <= x_min:
            x_min = 0
            x_max = image_width_px

        if self.full_height:
            y_min = 0
            y_max = image_height_px
        else:
            y_min = max(0, self.y_min_px)
            y_max = image_height_px if self.y_max_px < 0 else min(image_height_px, self.y_max_px)

            if y_max <= y_min:
                y_min = 0
                y_max = image_height_px

        return x_min, x_max, y_min, y_max, ground_width_seen_in, crop_fraction

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        image_height_px, image_width_px = frame.shape[:2]

        try:
            x_min, x_max, y_min, y_max, ground_width_seen_in, crop_fraction = (
                self.compute_crop_bounds(image_width_px, image_height_px)
            )
        except Exception as e:
            self.get_logger().error(f'Failed to compute crop bounds: {e}')
            return

        cropped = frame[y_min:y_max, x_min:x_max]

        if cropped.size == 0:
            self.get_logger().error('Computed crop is empty.')
            return

        if self.resize_width_px > 0 and self.resize_height_px > 0:
            try:
                cropped = cv2.resize(
                    cropped,
                    (self.resize_width_px, self.resize_height_px),
                    interpolation=cv2.INTER_LINEAR
                )
            except Exception as e:
                self.get_logger().error(f'Failed to resize cropped image: {e}')
                return

        if self.log_crop_once and not self.has_logged_crop:
            self.get_logger().info(
                f'Input image: {image_width_px}x{image_height_px} | '
                f'Computed ground width seen: {ground_width_seen_in:.2f} in | '
                f'Crop fraction: {crop_fraction:.3f} | '
                f'Crop bounds: x=[{x_min}, {x_max}), y=[{y_min}, {y_max}) | '
                f'Output image: {cropped.shape[1]}x{cropped.shape[0]}'
            )
            self.has_logged_crop = True

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cropped, encoding='bgr8')
            out_msg.header = msg.header
            self.cropped_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish cropped image: {e}')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundCropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
