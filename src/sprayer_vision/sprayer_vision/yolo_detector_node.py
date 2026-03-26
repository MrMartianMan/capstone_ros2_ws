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
    ROS 2 node that:
    1. Subscribes to a camera image topic
    2. Runs YOLO inference on incoming frames
    3. Detects cell phones and remotes
    4. Requires several consecutive frames before declaring a detection "stable"
    5. Publishes detection booleans and confidences
    6. Publishes an annotated image for debugging/visualization
    """

    def __init__(self):
        super().__init__('yolo_detector_node')

        # ----------------------------
        # User-configurable parameters
        # ----------------------------
        # model_path:
        #   Path to the YOLO model or TensorRT engine file.
        # image_topic:
        #   ROS topic providing raw camera images.
        # annotated_image_topic:
        #   ROS topic where the node publishes the annotated/debug image.
        # conf_threshold:
        #   Minimum confidence required before a detection is considered valid.
        # required_consecutive_frames:
        #   Number of consecutive frames required before a target is treated as
        #   a stable detection.
        # process_every_n_frames:
        #   Lets you skip frames to reduce compute load.
        # publish_annotated_image:
        #   Whether to publish the plotted debug image.
        # imgsz:
        #   Inference image size passed to YOLO.
        # device:
        #   Inference device string (for example "0" for GPU 0).
        self.declare_parameter('model_path', '/home/project-48/ros2_ws/yolo11s.engine')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('annotated_image_topic', '/detections/image')
        self.declare_parameter('conf_threshold', 0.50)
        self.declare_parameter('required_consecutive_frames', 3)
        self.declare_parameter('process_every_n_frames', 1)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '0')

        # ----------------------------
        # Read parameter values
        # ----------------------------
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

        # Save the model name separately so it can be shown on the annotated image.
        self.model_path = model_path
        self.model_name = os.path.basename(model_path)

        # ----------------------------
        # Load YOLO model/engine
        # ----------------------------
        self.get_logger().info(f'Loading model: {model_path}')
        self.get_logger().info(
            f'Inference settings -> imgsz={self.imgsz}, device={self.device}'
        )
        self.model = YOLO(model_path)

        # CvBridge converts ROS Image messages to OpenCV images and back.
        self.bridge = CvBridge()

        # ----------------------------
        # QoS configuration
        # ----------------------------
        # This QoS is intended for camera-style streaming, where low latency matters
        # more than keeping every old frame. A small queue depth helps prevent buildup
        # of stale frames if inference is slower than the camera frame rate.
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # ----------------------------
        # Publishers
        # ----------------------------
        # These boolean topics represent whether each class is currently considered
        # "stable" based on consecutive-frame logic.
        self.cell_phone_pub = self.create_publisher(
            Bool, '/sprayer/cell_phone_detected', 10
        )
        self.remote_pub = self.create_publisher(
            Bool, '/sprayer/remote_detected', 10
        )

        # These confidence topics publish the best confidence seen for the class
        # in the current processed frame.
        self.cell_phone_conf_pub = self.create_publisher(
            Float32, '/sprayer/cell_phone_confidence', 10
        )
        self.remote_conf_pub = self.create_publisher(
            Float32, '/sprayer/remote_confidence', 10
        )

        # Optional debug image showing YOLO boxes plus overlay text.
        self.annotated_image_pub = self.create_publisher(
            Image, self.annotated_image_topic, sensor_qos
        )

        # ----------------------------
        # Subscriber
        # ----------------------------
        # Subscribe to the raw camera image topic using the same low-latency QoS.
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        # Counts how many frames have been received total.
        self.frame_count = 0

        # Separate consecutive detection counters for each target class.
        # A class must remain detected for N consecutive processed frames before
        # it is treated as a stable detection.
        self.cell_phone_hit_count = 0
        self.remote_hit_count = 0

        # Keep a short rolling history of latency so the overlay can show a smoother
        # average instead of only the most recent frame timing.
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
        """
        Called every time a new image arrives.

        Main flow:
        1. Optionally skip frames
        2. Convert ROS image -> OpenCV image
        3. Run YOLO inference
        4. Parse detections for cell phone / remote
        5. Update consecutive-frame counters
        6. Publish detection states and confidences
        7. Publish an annotated image with debugging overlays
        """
        self.frame_count += 1

        # Optionally process only every Nth frame to reduce load.
        if self.process_every_n_frames > 1:
            if (self.frame_count % self.process_every_n_frames) != 0:
                return

        # Convert incoming ROS image message into an OpenCV BGR image.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # Run YOLO inference on the current frame.
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

        # ----------------------------
        # Timing / latency bookkeeping
        # ----------------------------
        # Ultralytics provides preprocess, inference, and postprocess timing
        # for each result when available. These are useful for debugging latency.
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

        # Track whether each class was found in this frame, and what the best
        # confidence was for that class.
        cell_phone_found = False
        remote_found = False

        best_cell_phone_conf = 0.0
        best_remote_conf = 0.0

        # Use the plotted YOLO result as the annotated debug image if available.
        annotated_frame = frame.copy()
        if len(results) > 0:
            annotated_frame = results[0].plot()

        # ----------------------------
        # Parse YOLO detections
        # ----------------------------
        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                class_name = self.model.names[cls_id]

                # Ignore detections that do not meet the configured confidence threshold.
                if conf < self.conf_threshold:
                    continue

                # Track only the two classes this node cares about.
                if class_name == 'cell phone':
                    cell_phone_found = True
                    if conf > best_cell_phone_conf:
                        best_cell_phone_conf = conf

                elif class_name == 'remote':
                    remote_found = True
                    if conf > best_remote_conf:
                        best_remote_conf = conf

        # ----------------------------
        # Consecutive-frame filtering
        # ----------------------------
        # This prevents one noisy frame from immediately triggering an output.
        # A target must persist across several processed frames in a row.
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

        # ----------------------------
        # Publish detection outputs
        # ----------------------------
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

        # ----------------------------
        # Publish annotated/debug image
        # ----------------------------
        # The debug image includes:
        # - the YOLO bounding boxes
        # - stable detection state/counters
        # - rolling latency statistics
        # - model file name
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
        """ROS node shutdown cleanup hook."""
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
