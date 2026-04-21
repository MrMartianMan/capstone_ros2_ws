from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agri_control',
            executable='joy_evdev_node',
            name='joy_evdev_node',
            output='screen',
            parameters=[{
                'device': '/dev/input/by-id/usb-Guangzhou_Chicken_Run_Network_Technology_Co.__Ltd._GameSir-G7_Pro-event-joystick'
            }]
        ),

        Node(
            package='remote_controller',
            executable='mode_manager_node',
            name='mode_manager_node',
            output='screen',
            parameters=[{
                'start_button_index': 9,
                'hold_seconds': 1.0,
                'workspace_path': '/home/project-48/capstone_ros2_ws'
            }]
        ),
    ])
