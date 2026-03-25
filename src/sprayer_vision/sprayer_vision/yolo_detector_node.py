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
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Parameters
        self.declare_parameter('model_path', '/home/project-48/ros2_ws/yolo11s.engine')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('annotated_image_topic', '/detections/image')
        self.declare_parameter('conf_threshold', 0.50)
        self.declare_parameter('required_consecutive_frames', 3)
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '0')

        # Read parameters
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

        # Store model info for overlay text
        self.model_path = model_path
        self.model_name = os.path.basename(model_path)

        # Load model
        self.get_logger().info(f'Loading model: {model_path}')
        self.get_logger().info(
            f'Inference settings -> imgsz={self.imgsz}, device={self.device}'
        )
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # QoS for low-latency camera-style streaming
        # depth=1 helps prevent backlog of old frames
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Detection publishers
        self.cell_phone_pub = self.create_publisher(
            Bool, '/sprayer/cell_phone_detected', 10
        )
        self.remote_pub = self.create_publisher(
            Bool, '/sprayer/remote_detected', 10
        )

        self.cell_phone_conf_pub = self.create_publisher(
            Float32, '/sprayer/cell_phone_confidence', 10
        )
        self.remote_conf_pub = self.create_publisher(
            Float32, '/sprayer/remote_confidence', 10
        )

        # Annotated image publisher
        self.annotated_image_pub = self.create_publisher(
            Image, self.annotated_image_topic, sensor_qos
        )

        # Latest-frame style subscription
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        self.frame_count = 0

        # Separate consecutive-frame counters
        self.cell_phone_hit_count = 0
        self.remote_hit_count = 0

        # Rolling average latency tracking
        self.latency_history_ms = deque(maxlen=30)

        self.get_logger().info(f'YOLO detector subscribed to {self.image_topic}')
        self.get_logger().info(
            f'Annotated image topic: {self.annotated_image_topic}'
        )
        self.get_logger().info(f'Overlay model label: {self.model_name}')
        self.get_logger().info(
            'Using low-latency QoS: KEEP_LAST depth=1, BEST_EFFORT'
        )

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

        cell_phone_found = False
        remote_found = False

        best_cell_phone_conf = 0.0
        best_remote_conf = 0.0

        # Use plotted YOLO image if available
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

                if class_name == 'cell phone':
                    cell_phone_found = True
                    if conf > best_cell_phone_conf:
                        best_cell_phone_conf = conf

                elif class_name == 'remote':
                    remote_found = True
                    if conf > best_remote_conf:
                        best_remote_conf = conf

        # Update consecutive-frame counters
        if cell_phone_found:
            self.cell_phone_hit_count += 1
        else:
            self.cell_phone_hit_count = 0

        if remote_found:
            self.remote_hit_count += 1
        else:
            self.remote_hit_count = 0

        cell_phone_stable = (
            self.cell_phone_hit_count >= self.required_consecutive_frames
        )
        remote_stable = (
            self.remote_hit_count >= self.required_consecutive_frames
        )

        # Publish detection topics
        self.cell_phone_pub.publish(Bool(data=cell_phone_stable))
        self.remote_pub.publish(Bool(data=remote_stable))

        self.cell_phone_conf_pub.publish(Float32(data=best_cell_phone_conf))
        self.remote_conf_pub.publish(Float32(data=best_remote_conf))

        if cell_phone_stable:
            self.get_logger().info(
                f'DETECTED: cell phone conf={best_cell_phone_conf:.2f} '
                f'frames={self.cell_phone_hit_count}'
            )

        if remote_stable:
            self.get_logger().info(
                f'DETECTED: remote conf={best_remote_conf:.2f} '
                f'frames={self.remote_hit_count}'
            )

        # Overlay text onto annotated image and publish to ROS
        if self.publish_annotated_image:
            status_text = (
                f"phone={cell_phone_stable}({self.cell_phone_hit_count}) "
                f"remote={remote_stable}({self.remote_hit_count})"
            )
            latency_text = (
                f"avg_latency={avg_latency_ms:.1f} ms "
                f"(pre={preprocess_ms:.1f} inf={inference_ms:.1f} post={postprocess_ms:.1f})"
            )
            model_text = f"model={self.model_name}"

            cv2.putText(
                annotated_frame,
                status_text,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )
            cv2.putText(
                annotated_frame,
                latency_text,
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )
            cv2.putText(
                annotated_frame,
                model_text,
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 200, 255),
                2
            )

            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(
                    annotated_frame, encoding='bgr8'
                )
                annotated_msg.header = msg.header
                self.annotated_image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(
                    f'Failed to publish annotated image: {e}'
                )

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
