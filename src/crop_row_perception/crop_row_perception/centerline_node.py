#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from sensor_msgs_py import point_cloud2


class CenterlineNode(Node):
    def __init__(self):
        super().__init__('centerline_node')

        self.cloud_topic = '/zed/zed_node/point_cloud/cloud_registered'
        self.frame_id = 'zed_camera_center'

        self.sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cb,
            10
        )

        self.path_pub = self.create_publisher(Path, '/centerline_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/centerline_marker', 10)

        self.get_logger().info(f'centerline_node started, listening on {self.cloud_topic}')

    def cb(self, msg):
        raw_pts = []

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p

            # Tight tabletop ROI
            if -0.35 < x < 0.35 and -0.35 < y < 0.35 and -0.50 < z < 0.20:
                raw_pts.append((x, y, z))

        if len(raw_pts) < 20:
            self.get_logger().warn(f'Not enough raw points: {len(raw_pts)}')
            return

        # Keep top points only (simple table rejection without numpy)
        z_vals = [p[2] for p in raw_pts]
        z_vals.sort()

        cut_index = int(0.85 * len(z_vals))
        if cut_index >= len(z_vals):
            cut_index = len(z_vals) - 1

        z_cut = z_vals[cut_index]

        top_pts = [p for p in raw_pts if p[2] >= z_cut]

        # Split rows with a forced center gap
        gap = 0.10
        left_pts = [(x, y) for x, y, z in top_pts if y > gap]
        right_pts = [(x, y) for x, y, z in top_pts if y < -gap]

        self.get_logger().info(
            f'raw={len(raw_pts)}, top={len(top_pts)}, left={len(left_pts)}, right={len(right_pts)}, z_cut={z_cut:.3f}'
        )

        if len(left_pts) < 5 or len(right_pts) < 5:
            self.get_logger().warn('Not enough left/right points')
            return

        # Sort both rows by x so points pair consistently
        left_pts.sort(key=lambda p: p[0])
        right_pts.sort(key=lambda p: p[0])

        count = min(len(left_pts), len(right_pts))
        if count < 5:
            self.get_logger().warn('Not enough paired row points')
            return

        center = []
        for i in range(count):
            lx, ly = left_pts[i]
            rx, ry = right_pts[i]

            cx = 0.5 * (lx + rx)
            cy = 0.5 * (ly + ry)
            center.append((cx, cy))

        # Sort centerline so marker/path draw cleanly
        center.sort(key=lambda p: p[0])

        self.publish_path(center)
        self.publish_marker(center)

    def publish_path(self, center):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in center:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.03
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)

    def publish_marker(self, center):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'centerline'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for x, y in center:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.03
            marker.points.append(p)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CenterlineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
