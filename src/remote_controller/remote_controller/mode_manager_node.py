#!/usr/bin/env python3

import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager_node')

        self.declare_parameter('start_button_index', 9)
        self.declare_parameter('hold_seconds', 3.0)
        self.declare_parameter('workspace_path', '/home/project-48/capstone_ros2_ws')

        self.start_button_index = int(self.get_parameter('start_button_index').value)
        self.hold_seconds = float(self.get_parameter('hold_seconds').value)
        self.workspace_path = str(self.get_parameter('workspace_path').value)

        self.current_mode = 'manual'
        self.button_press_time = None
        self.toggle_latched = False

        self.manual_proc = None
        self.autonomy_proc = None

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('Mode manager started')
        self.get_logger().info('Default mode: MANUAL')
        self.start_manual_mode()

    def joy_callback(self, msg: Joy):
        if self.start_button_index >= len(msg.buttons):
            return

        pressed = (msg.buttons[self.start_button_index] == 1)

        if pressed:
            if self.button_press_time is None:
                self.button_press_time = time.time()

            held_time = time.time() - self.button_press_time

            if held_time >= self.hold_seconds and not self.toggle_latched:
                self.toggle_latched = True
                self.toggle_mode()
        else:
            self.button_press_time = None
            self.toggle_latched = False

    def make_ros_command(self, inner_cmd: str) -> str:
        return (
            f"source /opt/ros/humble/setup.bash && "
            f"source {self.workspace_path}/install/setup.bash && "
            f"{inner_cmd}"
        )

    def start_process(self, cmd: str):
        return subprocess.Popen(
            ['bash', '-lc', cmd],
            preexec_fn=os.setsid
        )

    def stop_process_group(self, proc):
        if proc is not None and proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        return None

    def start_manual_mode(self):
        self.get_logger().info('Switching to MANUAL mode')
        self.autonomy_proc = self.stop_process_group(self.autonomy_proc)

        manual_cmd = self.make_ros_command(
            "ros2 launch remote_controller manual_mode.launch.py"
        )
        self.manual_proc = self.start_process(manual_cmd)
        self.current_mode = 'manual'

    def start_autonomy_mode(self):
        self.get_logger().info('Switching to AUTONOMY mode')
        self.manual_proc = self.stop_process_group(self.manual_proc)

        auto_cmd = self.make_ros_command(
            "ros2 launch remote_controller autonomy_mode.launch.py"
        )
        self.autonomy_proc = self.start_process(auto_cmd)
        self.current_mode = 'autonomy'

    def toggle_mode(self):
        if self.current_mode == 'manual':
            self.start_autonomy_mode()
        else:
            self.start_manual_mode()

    def destroy_node(self):
        self.get_logger().info('Stopping all managed processes...')
        self.manual_proc = self.stop_process_group(self.manual_proc)
        self.autonomy_proc = self.stop_process_group(self.autonomy_proc)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
