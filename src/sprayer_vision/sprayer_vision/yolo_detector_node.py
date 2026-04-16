#!/usr/bin/env python3

import os
from collections import deque

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy
)

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2


class YoloDetectorNode(Node):
    """
    Standardized object-detection node.

    Publishes:
    - /sprayer/disease_detected
    - /sprayer/disease_confidence
    - /sprayer/weed_detected
    - /sprayer/weed_confidence

    For pretrained mode:
    - disease_class_name = scissors
    - weed_class_name = remote
    """

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', '/home/project-48/models/yolo/yolo11s_640.engine')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('annotated_image_topic', '/detections/image')
        self.declare_parameter('conf_threshold', 0.50)
        self.declare_parameter('required_consecutive_frames', 3)
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '0')

        self.declare_parameter('disease_class_name', 'scissors')
        self.declare_parameter('weed_class_name', 'remote')

        model_path = self.get_parameter('model_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.annotated_image_topic = self.get_parameter('annotated_image_topic').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.required_consecutive_frames = int(
            self.get_parameter('required_consecutive_frames').value
        )
        self.process_every_n_frames = int(
            self.get_parameter('process_every_n_frames').value
        )
        self.publish_annotated_image = bool(
            self.get_parameter('publish_annotated_image').value
        )
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = str(self.get_parameter('device').value)

        self.disease_class_name = str(self.get_parameter('disease_class_name').value)
        self.weed_class_name = str(self.get_parameter('weed_class_name').value)

        self.model_path = model_path
        self.model_name = os.path.basename(model_path)

        self.get_logger().info(f'Loading model: {model_path}')
        self.get_logger().info(
            f'Inference settings -> imgsz={self.imgsz}, device={self.device}'
        )
        self.get_logger().info(
            f'Mapping classes -> disease={self.disease_class_name}, weed={self.weed_class_name}'
        )

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.disease_pub = self.create_publisher(
            Bool, '/sprayer/disease_detected', 10
        )
        self.weed_pub = self.create_publisher(
            Bool, '/sprayer/weed_detected', 10
        )

        self.disease_conf_pub = self.create_publisher(
            Float32, '/sprayer/disease_confidence', 10
        )
        self.weed_conf_pub = self.create_publisher(
            Float32, '/sprayer/weed_confidence', 10
        )

        self.annotated_image_pub = self.create_publisher(
            Image, self.annotated_image_topic, sensor_qos
        )

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        self.frame_count = 0
        self.disease_hit_count = 0
        self.weed_hit_count = 0
        self.latency_history_ms = deque(maxlen=30)

        self.get_logger().info(f'YOLO detector subscribed to {self.image_topic}')
        self.get_logger().info(f'Annotated image topic: {self.annotated_image_topic}')
        self.get_logger().info(f'Overlay model label: {self.model_name}')

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if self.process_every_n_frames > 1:
            if (self.frame_count % self.process_every_n_frames) != 0:
                return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        try:
            results = self.model(
                frame,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        preprocess_ms = 0.0
        inference_ms = 0.0
        postprocess_ms = 0.0

        if len(results) > 0 and hasattr(results[0], 'speed') and results[0].speed is not None:
            preprocess_ms = float(results[0].speed.get('preprocess', 0.0))
            inference_ms = float(results[0].speed.get('inference', 0.0))
            postprocess_ms = float(results[0].speed.get('postprocess', 0.0))

        total_latency_ms = preprocess_ms + inference_ms + postprocess_ms
        self.latency_history_ms.append(total_latency_ms)
        avg_latency_ms = sum(self.latency_history_ms) / len(self.latency_history_ms)

        disease_found = False
        weed_found = False
        best_disease_conf = 0.0
        best_weed_conf = 0.0

        annotated_frame = frame.copy()
        if len(results) > 0:
            annotated_frame = results[0].plot()

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                class_name = self.model.names[cls_id]

                if conf < self.conf_threshold:
                    continue

                if class_name == self.disease_class_name:
                    disease_found = True
                    if conf > best_disease_conf:
                        best_disease_conf = conf

                elif class_name == self.weed_class_name:
                    weed_found = True
                    if conf > best_weed_conf:
                        best_weed_conf = conf

        if disease_found:
            self.disease_hit_count += 1
        else:
            self.disease_hit_count = 0

        if weed_found:
            self.weed_hit_count += 1
        else:
            self.weed_hit_count = 0

        disease_stable = self.disease_hit_count >= self.required_consecutive_frames
        weed_stable = self.weed_hit_count >= self.required_consecutive_frames

        self.disease_pub.publish(Bool(data=disease_stable))
        self.weed_pub.publish(Bool(data=weed_stable))

        self.disease_conf_pub.publish(Float32(data=best_disease_conf))
        self.weed_conf_pub.publish(Float32(data=best_weed_conf))

        if disease_stable:
            self.get_logger().info(
                f'DISEASE DETECTED: conf={best_disease_conf:.2f} frames={self.disease_hit_count}'
            )

        if weed_stable:
            self.get_logger().info(
                f'WEED DETECTED: conf={best_weed_conf:.2f} frames={self.weed_hit_count}'
            )

        if self.publish_annotated_image:
            status_text = (
                f"disease={disease_stable}({self.disease_hit_count}) "
                f"weed={weed_stable}({self.weed_hit_count})"
            )
            latency_text = (
                f"avg_latency={avg_latency_ms:.1f} ms "
                f"(pre={preprocess_ms:.1f} inf={inference_ms:.1f} post={postprocess_ms:.1f})"
            )
            model_text = f"model={self.model_name}"

            cv2.putText(
                annotated_frame, status_text, (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )
            cv2.putText(
                annotated_frame, latency_text, (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )
            cv2.putText(
                annotated_frame, model_text, (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 255), 2
            )

            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(
                    annotated_frame, encoding='bgr8'
                )
                annotated_msg.header = msg.header
                self.annotated_image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish annotated image: {e}')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
