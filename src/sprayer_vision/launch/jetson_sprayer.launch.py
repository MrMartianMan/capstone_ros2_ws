from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
		'image_size': [3840, 2160],
		'time_per_frame': [1, 15]
            }]
        ),
        Node(
            package='sprayer_vision',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen',
            parameters=[{
                'model_path': '/home/project-48/ros2_ws/yolo11s_640.engine',
                'image_topic': '/image_raw',
		'annotated_image_topic': '/detections/image',
                'conf_threshold': 0.50,
                'required_consecutive_frames': 3,
                'process_every_n_frames': 1,
		'device': "0",
		'imgsz': 640,
		'publish_annotated_image': True
            }]
        ),
        Node(
            package='sprayer_vision',
            executable='sprayer_gpio_node',
            name='sprayer_gpio_node',
            output='screen',
            parameters=[{
                'cell_phone_gpio_pin': 13,
                'remote_gpio_pin': 32,
                'hold_time_sec': 3.0,
                'active_high': True,
            }]
        ),
    ])
