from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    VISION_MODE = 'pretrained'   # 'random', 'pretrained', or 'cercospora'
    USE_CROP = True              # True or False
    ROW_LAYOUT = 'single_row'    # 'single_row' or 'double_row'

    nodes = []

    if VISION_MODE in ['pretrained', 'cercospora']:
        nodes.append(
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='v4l2_camera',
                output='screen',
                parameters=[{
                    'video_device': '/dev/video0',
                    'pixel_format': 'YUYV',
                    'output_encoding': 'bgr8',
                    'image_size': [3840, 2160],
                }]
            )
        )

        if USE_CROP:
            nodes.append(
                Node(
                    package='sprayer_vision',
                    executable='ground_crop_node',
                    name='ground_crop_node',
                    output='screen',
                    parameters=[{
                        'image_topic': '/image_raw',
                        'cropped_image_topic': '/image_crop_between_legs',
                        'mount_height_in': 72.0,
                        'desired_ground_width_in': 80.0,
                        'hfov_deg': 88.0,
                        'crop_center_x_offset_px': 0,
                        'crop_margin_px': 0,
                        'full_height': True,
                        'resize_width_px': 0,
                        'resize_height_px': 0,
                        'log_crop_once': True,
                    }]
                )
            )

    vision_image_topic = '/image_crop_between_legs' if USE_CROP else '/image_raw'

    nodes.append(
        Node(
            package='sprayer_vision',
            executable='sprayer_gpio_node',
            name='sprayer_gpio_node',
            output='screen',
            parameters=[{
                'disease_gpio_pin': 13,
                'weed_gpio_pin': 32,
                'hold_time_sec': 2.0,
                'active_high': True,
                'signal_delay_sec': 0.0,
                'row_layout': ROW_LAYOUT,
            }]
        )
    )

    if VISION_MODE == 'random':
        nodes.append(
            Node(
                package='sprayer_vision',
                executable='random_detection_publisher_node',
                name='random_detection_publisher_node',
                output='screen',
                parameters=[{
                    'disease_min_interval_sec': 0.1,
                    'disease_max_interval_sec': 10.0,
                    'weed_min_interval_sec': 0.1,
                    'weed_max_interval_sec': 10.0,
                    'detection_pulse_sec': 0.10,
                }]
            )
        )

    elif VISION_MODE == 'pretrained':
        nodes.append(
            Node(
                package='sprayer_vision',
                executable='yolo_detector_node',
                name='yolo_detector_node',
                output='screen',
                parameters=[{
                    'model_path': '/home/project-48/models/yolo/yolo11s_640.engine',
                    'image_topic': vision_image_topic,
                    'annotated_image_topic': '/detections/image',
                    'conf_threshold': 0.50,
                    'required_consecutive_frames': 3,
                    'process_every_n_frames': 1,
                    'device': '0',
                    'imgsz': 640,
                    'publish_annotated_image': True,
                    'disease_class_name': 'scissors',
                    'weed_class_name': 'remote',
                }]
            )
        )

    elif VISION_MODE == 'cercospora':
        nodes.append(
            Node(
                package='sprayer_vision',
                executable='cercospora_classifier_node',
                name='cercospora_classifier_node',
                output='screen',
                parameters=[{
                    'model_path': '/home/project-48/models/yolo/cercospora_best.engine',
                    'image_topic': vision_image_topic,
                    'annotated_image_topic': '/cercospora/image',
                    'conf_threshold': 0.50,
                    'required_consecutive_frames': 3,
                    'process_every_n_frames': 1,
                    'device': '0',
                    'imgsz': 224,
                    'publish_annotated_image': True,
                    'positive_class_name': 'Cercospora Present'
                }]
            )
        )

    else:
        raise RuntimeError(
            f"Unknown VISION_MODE='{VISION_MODE}'. Use random, pretrained, or cercospora."
        )

    return LaunchDescription(nodes)
