#!/usr/bin/env python3

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from sensor_msgs_py import point_cloud2


class CenterlineNode(Node):
    def __init__(self) -> None:
        super().__init__('centerline_node')

        self.cloud_topic = '/zed/zed_node/point_cloud/cloud_registered'
        self.target_frame = 'zed_left_camera_frame'

        # Tighter forward-looking ROI to reduce jitter
        self.x_min_roi = 0.25
        self.x_max_roi = 1.60
        self.y_min_roi = -0.60
        self.y_max_roi = 0.60
        self.z_min_roi = -3.0
        self.z_max_roi = 3.0

        self.slice_count = 12
        self.min_points_per_slice = 30
        self.min_points_per_side = 8
        self.min_row_gap = 0.12
        self.max_row_gap = 1.20

        self.row_marker_size = 0.04
        self.centerline_width = 0.05
        self.path_z_offset = 0.02

        self.marker_z = 0.0

        # Temporal smoothing
        self.prev_center_pts: List[Tuple[float, float]] = []
        self.temporal_alpha = 0.35  # lower = smoother

        self.sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cb,
            10
        )

        self.path_pub = self.create_publisher(Path, '/centerline_path', 10)
        self.center_pub = self.create_publisher(Marker, '/centerline_marker', 10)
        self.rows_pub = self.create_publisher(Marker, '/row_points_marker', 10)

        self.get_logger().info('### STABLE FORWARD CENTERLINE NODE ###')
        self.get_logger().info(f'cloud_topic={self.cloud_topic}')
        self.get_logger().info(f'target_frame={self.target_frame}')
        self.get_logger().info(
            f'ROI x:[{self.x_min_roi:.2f}, {self.x_max_roi:.2f}] '
            f'y:[{self.y_min_roi:.2f}, {self.y_max_roi:.2f}]'
        )

    def smooth_points(self, pts: List[Tuple[float, float]], window: int = 5) -> List[Tuple[float, float]]:
        if len(pts) < 3:
            return pts

        out: List[Tuple[float, float]] = []
        half = window // 2

        for i in range(len(pts)):
            i0 = max(0, i - half)
            i1 = min(len(pts), i + half + 1)
            xs = [p[0] for p in pts[i0:i1]]
            ys = [p[1] for p in pts[i0:i1]]
            out.append((float(np.mean(xs)), float(np.mean(ys))))

        return out

    def temporal_smooth(self, pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not self.prev_center_pts or len(self.prev_center_pts) != len(pts):
            self.prev_center_pts = pts.copy()
            return pts

        smoothed: List[Tuple[float, float]] = []
        for (x_now, y_now), (x_prev, y_prev) in zip(pts, self.prev_center_pts):
            x = self.temporal_alpha * x_now + (1.0 - self.temporal_alpha) * x_prev
            y = self.temporal_alpha * y_now + (1.0 - self.temporal_alpha) * y_prev
            smoothed.append((x, y))

        self.prev_center_pts = smoothed.copy()
        return smoothed

    def cb(self, msg: PointCloud2) -> None:
        if msg.header.frame_id != self.target_frame:
            self.get_logger().warn(
                f'Expected frame {self.target_frame}, got {msg.header.frame_id}'
            )
            return

        pts = []

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if not (self.x_min_roi < x < self.x_max_roi):
                continue
            if not (self.y_min_roi < y < self.y_max_roi):
                continue
            if not (self.z_min_roi < z < self.z_max_roi):
                continue

            pts.append((x, y, z))

        self.get_logger().info(f'roi_pts={len(pts)}')

        if len(pts) < 50:
            self.get_logger().warn('Not enough ROI points')
            return

        pts_np = np.array(pts, dtype=np.float64)
        self.marker_z = float(np.median(pts_np[:, 2])) + self.path_z_offset

        xs = pts_np[:, 0]
        ys = pts_np[:, 1]

        x_min = float(np.min(xs))
        x_max = float(np.max(xs))

        if abs(x_max - x_min) < 1e-6:
            self.get_logger().warn('Forward span too small')
            return

        x_step = (x_max - x_min) / self.slice_count

        left_row_pts = []
        right_row_pts = []
        center_pts = []

        for i in range(self.slice_count):
            x0 = x_min + i * x_step
            x1 = x0 + x_step

            mask = (xs > x0) & (xs < x1)
            slice_pts = pts_np[mask]

            if len(slice_pts) < self.min_points_per_slice:
                continue

            y_vals = np.sort(slice_pts[:, 1])

            mid_y = float(np.mean(y_vals))
            left = y_vals[y_vals < mid_y]
            right = y_vals[y_vals > mid_y]

            if len(left) < self.min_points_per_side or len(right) < self.min_points_per_side:
                continue

            left_center = float(np.median(left))
            right_center = float(np.median(right))

            gap = abs(right_center - left_center)
            if not (self.min_row_gap < gap < self.max_row_gap):
                continue

            xc = 0.5 * (x0 + x1)
            yc = 0.5 * (left_center + right_center)

            left_row_pts.append((xc, left_center))
            right_row_pts.append((xc, right_center))
            center_pts.append((xc, yc))

        self.get_logger().info(
            f'left_pts={len(left_row_pts)}, right_pts={len(right_row_pts)}, center_pts={len(center_pts)}'
        )

        if len(center_pts) < 3:
            self.get_logger().warn('Not enough centerline points')
            return

        center_pts.sort(key=lambda p: p[0])
        left_row_pts.sort(key=lambda p: p[0])
        right_row_pts.sort(key=lambda p: p[0])

        center_pts = self.smooth_points(center_pts, window=5)
        left_row_pts = self.smooth_points(left_row_pts, window=5)
        right_row_pts = self.smooth_points(right_row_pts, window=5)

        center_pts = self.temporal_smooth(center_pts)

        self.publish_rows(left_row_pts, right_row_pts)
        self.publish_centerline(center_pts)
        self.publish_path(center_pts)

    def publish_rows(self, left_row_pts, right_row_pts) -> None:
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'row_points'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self.row_marker_size
        marker.scale.y = self.row_marker_size
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

        for x, y in left_row_pts + right_row_pts:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(self.marker_z)
            marker.points.append(p)

        self.rows_pub.publish(marker)

    def publish_centerline(self, center_pts) -> None:
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'centerline'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.centerline_width
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        for x, y in center_pts:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(self.marker_z)
            marker.points.append(p)

        self.center_pub.publish(marker)

    def publish_path(self, center_pts) -> None:
        path = Path()
        path.header.frame_id = self.target_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(center_pts):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(self.marker_z)

            if i < len(center_pts) - 1:
                dx = center_pts[i + 1][0] - x
                dy = center_pts[i + 1][1] - y
                yaw = math.atan2(dy, dx)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CenterlineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
