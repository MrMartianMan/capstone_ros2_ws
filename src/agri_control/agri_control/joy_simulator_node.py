import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import termios
import tty
import threading
import time

class JoySimulator(Node):

    def __init__(self):
        super().__init__('joy_simulator')

        self.publisher = self.create_publisher(Joy, '/joy', 10)

        self.axes = [0.0]*8
        self.buttons = [0]*11

        self.timer = self.create_timer(0.05, self.publish_joy)

        self.get_logger().info("Joy Simulator Ready")
        self.get_logger().info("Controls: W/S A/D Q/E SPACE")

        thread = threading.Thread(target=self.keyboard_loop)
        thread.daemon = True
        thread.start()

    def publish_joy(self):
        msg = Joy()
        msg.axes = self.axes
        msg.buttons = self.buttons
        self.publisher.publish(msg)

    def keyboard_loop(self):

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            while True:

                key = sys.stdin.read(1)

                if key == 'w':
                    self.axes[1] = 1.0
                elif key == 's':
                    self.axes[1] = -1.0
                elif key == 'a':
                    self.axes[0] = -1.0
                elif key == 'd':
                    self.axes[0] = 1.0
                elif key == 'q':
                    self.axes[3] = -1.0
                elif key == 'e':
                    self.axes[3] = 1.0
                elif key == ' ':
                    self.buttons[0] = 1
                else:
                    self.axes = [0.0]*8
                    self.buttons = [0]*11

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = JoySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
