from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='remote_controller',
            executable='teleop_node',
            name='teleop_node',
            output='screen'
        ),
        Node(
            package='motor_controller',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen'
        ),
    ])
