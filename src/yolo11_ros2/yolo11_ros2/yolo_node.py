#!/usr/bin/env python3
import time
import math
from typing import Any

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge

import cv2
import torch
from ultralytics import YOLO


class Yolo11Node(Node):
    """
    ROS2 Humble node:
      - Subscribes: sensor_msgs/Image (e.g. /image_raw)
      - Crops image to match a desired ground width at a known mount height (assumes camera points straight down)
      - Runs: Ultralytics YOLO (custom .pt or .engine, etc.)
      - Publishes: vision_msgs/Detection2DArray
      - Optional: publishes annotated image (cropped) for debugging
    """

    def __init__(self):
        super().__init__('yolo11_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('annotated_topic', '/detections/image')

        self.declare_parameter('model', 'yolo11n.pt')   # path to .pt/.engine
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)

        # device:
        #   'auto' -> GPU 0 if available else CPU
        #   'cpu'  -> force CPU
        #   0/1/.. -> GPU index (only if CUDA available)
        # NOTE: Ultralytics expects device='cpu' or device=<int>, not 'cuda:0'
        self.declare_parameter('device', 'auto')

        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('max_fps', 0.0)          # 0 = unlimited
        self.declare_parameter('resize_width', 0)       # 0 = no resize (after crop), else resize to this width

        # Crop-by-geometry (assumes camera points straight down)
        self.declare_parameter('enable_ground_crop', True)
        self.declare_parameter('mount_height_in', 85.25)              # camera height above ground (inches)
        self.declare_parameter('desired_ground_width_in', 73.0)       # desired ground width to keep (inches)
        self.declare_parameter('hfov_deg', 88.0)                      # lens horizontal FOV degrees (Arducam IMX678 ~88)
        self.declare_parameter('crop_margin', 0.0)                    # extra fraction margin to keep (e.g. 0.05 = +5%)

        # -----------------------------
        # Read params
        # -----------------------------
        self.image_topic = self.get_parameter('image_topic').value
        self.det_topic = self.get_parameter('detections_topic').value
        self.ann_topic = self.get_parameter('annotated_topic').value

        self.model_path = self.get_parameter('model').value
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)

        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.resize_width = int(self.get_parameter('resize_width').value)

        self.enable_ground_crop = bool(self.get_parameter('enable_ground_crop').value)
        self.mount_height_in = float(self.get_parameter('mount_height_in').value)
        self.desired_ground_width_in = float(self.get_parameter('desired_ground_width_in').value)
        self.hfov_deg = float(self.get_parameter('hfov_deg').value)
        self.crop_margin = float(self.get_parameter('crop_margin').value)

        # -----------------------------
        # Device selection (robust)
        # -----------------------------
        dev_param = self.get_parameter('device').value
        self.device = self._resolve_ultralytics_device(dev_param)

        # -----------------------------
        # Model load
        # -----------------------------
        self.get_logger().info(f'Loading YOLO model: {self.model_path} on device={self.device}')
        self.model = YOLO(self.model_path)

        # -----------------------------
        # ROS I/O
        # -----------------------------
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.det_pub = self.create_publisher(Detection2DArray, self.det_topic, 10)
        self.ann_pub = self.create_publisher(Image, self.ann_topic, 10) if self.publish_annotated else None

        self._last_infer_t = 0.0

        # Log crop math once so you can sanity-check
        if self.enable_ground_crop:
            full_w_in = self._full_ground_width_in(self.mount_height_in, self.hfov_deg)
            frac = self._crop_fraction(self.mount_height_in, self.desired_ground_width_in, self.hfov_deg, self.crop_margin)
            self.get_logger().info(
                f'Ground-crop enabled: height={self.mount_height_in:.2f}in, desired_width={self.desired_ground_width_in:.2f}in, '
                f'hfov={self.hfov_deg:.2f}deg -> full_ground_width≈{full_w_in:.2f}in, keep_frac≈{frac:.3f}'
            )

        self.get_logger().info(f'Subscribed to: {self.image_topic}')
        self.get_logger().info(f'Publishing detections: {self.det_topic}')
        if self.publish_annotated:
            self.get_logger().info(f'Publishing annotated image: {self.ann_topic}')

    # -----------------------------
    # Helpers
    # -----------------------------
    def _resolve_ultralytics_device(self, dev_param: Any) -> object:
        """
        Ultralytics expects:
          - 'cpu' (string) for CPU
          - 0, 1, ... (int) for CUDA GPU indices
        We'll accept:
          - 'auto'
          - 'cpu'
          - 0 / '0'
          - 'cuda:0' (normalize)
        """
        try:
            if isinstance(dev_param, str):
                s = dev_param.strip().lower()
                if s == 'auto':
                    return 0 if (torch.cuda.is_available() and torch.cuda.device_count() > 0) else 'cpu'
                if s == 'cpu':
                    return 'cpu'
                if s.startswith('cuda:'):
                    s = s.replace('cuda:', '')
                return int(s)

            if isinstance(dev_param, int):
                return 'cpu' if dev_param < 0 else dev_param

        except Exception as e:
            self.get_logger().warn(f"Invalid 'device' param ({dev_param}); falling back to CPU. Reason: {e}")
            return 'cpu'

        return 'cpu'

    def _full_ground_width_in(self, height_in: float, hfov_deg: float) -> float:
        hfov = math.radians(hfov_deg)
        return 2.0 * height_in * math.tan(hfov / 2.0)

    def _crop_fraction(self, height_in: float, desired_width_in: float, hfov_deg: float, margin_frac: float) -> float:
        full_w = self._full_ground_width_in(height_in, hfov_deg)
        if full_w <= 1e-6:
            return 1.0
        frac = desired_width_in / full_w
        frac = frac * (1.0 + max(0.0, margin_frac))
        return max(0.05, min(frac, 1.0))

    def _apply_ground_crop(self, frame):
        """
        Center-crop horizontally to match the desired ground width.
        Assumes camera optical axis points straight down onto a flat ground plane.
        Keeps full height; crops only width (horizontal row width).
        """
        h_px, w_px = frame.shape[:2]
        frac = self._crop_fraction(self.mount_height_in, self.desired_ground_width_in, self.hfov_deg, self.crop_margin)

        crop_w = int(w_px * frac)
        crop_w = max(2, min(crop_w, w_px))

        x1 = (w_px - crop_w) // 2
        x2 = x1 + crop_w

        return frame[:, x1:x2]

    def _maybe_resize(self, frame):
        if self.resize_width and self.resize_width > 0:
            h, w = frame.shape[:2]
            if w > 0 and w != self.resize_width:
                scale = self.resize_width / float(w)
                new_h = max(2, int(h * scale))
                frame = cv2.resize(frame, (self.resize_width, new_h), interpolation=cv2.INTER_LINEAR)
        return frame

    # -----------------------------
    # Main callback
    # -----------------------------
    def image_cb(self, msg: Image):
        # Optional FPS limiting
        if self.max_fps > 0.0:
            now = time.time()
            min_dt = 1.0 / self.max_fps
            if now - self._last_infer_t < min_dt:
                return
            self._last_infer_t = now

        # ROS Image -> OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Apply ground crop first (so YOLO only sees the zoomed region)
        if self.enable_ground_crop:
            frame = self._apply_ground_crop(frame)

        # Optional resize (after crop) to manage performance
        frame = self._maybe_resize(frame)

        # Inference
        try:
            results = self.model.predict(
                source=frame,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                verbose=False
            )[0]
        except Exception as e:
            self.get_logger().error(f'YOLO predict failed on device={self.device}: {e}')
            return

        # Publish detections in crop-image pixel coordinates
        det_msg = Detection2DArray()
        det_msg.header = msg.header

        if results.boxes is not None and len(results.boxes) > 0:
            boxes_xyxy = results.boxes.xyxy.detach().cpu().numpy()
            confs = results.boxes.conf.detach().cpu().numpy()
            clss = results.boxes.cls.detach().cpu().numpy().astype(int)

            for (x1, y1, x2, y2), score, cls_id in zip(boxes_xyxy, confs, clss):
                det = Detection2D()
                det.header = msg.header

                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                w = float(x2 - x1)
                h = float(y2 - y1)

                bbox = BoundingBox2D()
                bbox.center.position.x = cx
                bbox.center.position.y = cy
                bbox.size_x = w
                bbox.size_y = h
                det.bbox = bbox

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls_id)   # numeric id; map to names downstream if desired
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)

                det_msg.detections.append(det)

        self.det_pub.publish(det_msg)

        # Optional annotated cropped image
        if self.publish_annotated and self.ann_pub is not None:
            annotated = frame.copy()
            names = getattr(self.model, "names", None) or {}

            if results.boxes is not None and len(results.boxes) > 0:
                for b in results.boxes:
                    x1, y1, x2, y2 = b.xyxy[0].detach().cpu().tolist()
                    cls_id = int(b.cls[0].item())
                    score = float(b.conf[0].item())
                    label = f'{names.get(cls_id, cls_id)} {score:.2f}'

                    cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(
                        annotated,
                        label,
                        (int(x1), max(0, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self.ann_pub.publish(ann_msg)


def main():
    rclpy.init()
    node = Yolo11Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
