import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# ================= CONSTANTS =================
LINEAR_AXIS  = 1    # Left stick up/down
ANGULAR_AXIS = 0    # Left stick left/right
DEADZONE = 0.1

MAX_LINEAR  = 0.6
MAX_ANGULAR = 1.0
# ============================================


def apply_deadzone(value):
    return 0.0 if abs(value) < DEADZONE else value


class RemoteControllerNode(Node):

    def __init__(self):
        super().__init__('remote_controller_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info("Remote Controller Node Started")


    def joy_callback(self, msg):
        linear_input  = apply_deadzone(msg.axes[LINEAR_AXIS])
        angular_input = apply_deadzone(msg.axes[ANGULAR_AXIS])

        cmd = Twist()
        cmd.linear.x  = linear_input  * MAX_LINEAR
        cmd.angular.z = angular_input * MAX_ANGULAR

        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = RemoteControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()