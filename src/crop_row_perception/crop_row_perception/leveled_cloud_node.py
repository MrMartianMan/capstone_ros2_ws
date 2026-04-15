#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


class LeveledCloudNode(Node):
    def __init__(self):
        super().__init__('leveled_cloud_node')

        self.input_topic = '/zed/zed_node/point_cloud/cloud_registered'
        self.output_topic = '/rotated_cloud'
        self.output_frame = 'rotated_cloud_frame'

        # ---------------------------------
        # ROTATION SETTINGS
        # ---------------------------------
        # Based on your camera being almost straight down:
        # 1) pitch flattens the cloud
        # 2) yaw turns it so the rows line up like your ideal screenshot
        #
        # If it rotates the wrong direction:
        #   change yaw_deg from -90 to +90
        self.roll_deg = 0.0
        self.pitch_deg = 90.0
        self.yaw_deg = -90.0

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cb,
            10
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info('### ROTATED CLOUD NODE IS RUNNING ###')
        self.get_logger().info(f'Input topic:  {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Output frame: {self.output_frame}')
        self.get_logger().info(
            f'Rotation deg (roll, pitch, yaw): {self.roll_deg}, {self.pitch_deg}, {self.yaw_deg}'
        )

    def rotate_point(self, x, y, z):
        roll = math.radians(self.roll_deg)
        pitch = math.radians(self.pitch_deg)
        yaw = math.radians(self.yaw_deg)

        # Roll around X
        x1 = x
        y1 = y * math.cos(roll) - z * math.sin(roll)
        z1 = y * math.sin(roll) + z * math.cos(roll)

        # Pitch around Y
        x2 = x1 * math.cos(pitch) + z1 * math.sin(pitch)
        y2 = y1
        z2 = -x1 * math.sin(pitch) + z1 * math.cos(pitch)

        # Yaw around Z
        x3 = x2 * math.cos(yaw) - y2 * math.sin(yaw)
        y3 = x2 * math.sin(yaw) + y2 * math.cos(yaw)
        z3 = z2

        return x3, y3, z3

    def cb(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]

        has_rgb = 'rgb' in field_names
        has_rgba = 'rgba' in field_names

        if has_rgb:
            read_fields = ('x', 'y', 'z', 'rgb')
        elif has_rgba:
            read_fields = ('x', 'y', 'z', 'rgba')
        else:
            read_fields = ('x', 'y', 'z')

        rotated_points = []
        count_in = 0

        for p in point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True):
            count_in += 1

            if has_rgb or has_rgba:
                x, y, z, color = p
            else:
                x, y, z = p
                color = None

            xr, yr, zr = self.rotate_point(float(x), float(y), float(z))

            if has_rgb or has_rgba:
                rotated_points.append((xr, yr, zr, color))
            else:
                rotated_points.append((xr, yr, zr))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame

        if has_rgb:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            cloud_out = point_cloud2.create_cloud(header, fields, rotated_points)
        elif has_rgba:
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
            ]
            cloud_out = point_cloud2.create_cloud(header, fields, rotated_points)
        else:
            cloud_out = point_cloud2.create_cloud_xyz32(header, rotated_points)

        self.pub.publish(cloud_out)
        self.get_logger().info(f'Published rotated cloud with {count_in} input points')


def main(args=None):
    rclpy.init(args=args)
    node = LeveledCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
