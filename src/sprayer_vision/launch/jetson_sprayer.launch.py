from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Main launch file for the sprayer vision system.

    This launch file supports two operating modes:

    1. USE_YOLO = True
       - Runs the real camera
       - Runs the YOLO detector node
       - Runs the GPIO sprayer control node

    2. USE_YOLO = False
       - Runs the real camera
       - Skips YOLO and GPIO detection control
       - Runs the random detection GPIO simulator instead

    This makes it easy to switch between:
    - real object detection testing
    - fallback demonstration mode when YOLO is unavailable
    """

    # Set this flag to choose between the real detection pipeline and the
    # random detection simulator.
    #
    # True  -> use the real YOLO + GPIO sprayer pipeline
    # False -> use the random detection generator instead
    USE_YOLO = False

    # Start with the shared base node list.
    # The camera node is always launched because both real mode and simulation
    # mode may still benefit from having the camera available.
    nodes = [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[{
                # Linux camera device path
                'video_device': '/dev/video0',

                # Input format requested from the camera
                'pixel_format': 'YUYV',

                # Output format published on the ROS image topic
                'output_encoding': 'bgr8',

                # Camera resolution
                'image_size': [3840, 2160],
            }]
        )
    ]

    if USE_YOLO:
        # ------------------------------------------------------------
        # Real detection mode
        # ------------------------------------------------------------
        # This mode runs the full pipeline:
        #
        # camera -> YOLO detector -> detection topics -> GPIO sprayer node
        #
        # The YOLO node detects targets and publishes stable detections.
        # The GPIO node waits for those detections, applies the actuation delay,
        # and then drives the sprayer relay outputs.
        nodes.append(
            Node(
                package='sprayer_vision',
                executable='yolo_detector_node',
                name='yolo_detector_node',
                output='screen',
                parameters=[{
                    # Path to the TensorRT engine or YOLO model file
                    'model_path': '/home/project-48/models/yolo/yolo11s_640.engine',

                    # Input image topic from the camera node
                    'image_topic': '/image_raw',

                    # Topic where the annotated debug image is published
                    'annotated_image_topic': '/detections/image',

                    # Minimum confidence required for a detection to count
                    'conf_threshold': 0.50,

                    # Number of consecutive frames required before a detection
                    # is treated as stable
                    'required_consecutive_frames': 3,

                    # Process every frame (set higher to skip frames if needed)
                    'process_every_n_frames': 1,

                    # GPU device selection for inference
                    'device': "0",

                    # YOLO inference image size
                    'imgsz': 640,

                    # Whether to publish annotated detection images
                    'publish_annotated_image': True
                }]
            )
        )

        nodes.append(
            Node(
                package='sprayer_vision',
                executable='sprayer_gpio_node',
                name='sprayer_gpio_node',
                output='screen',
                parameters=[{
                    # GPIO pin for cell phone detections
                    'cell_phone_gpio_pin': 13,

                    # GPIO pin for mouse detections
                    'mouse_gpio_pin': 32,

                    # How long the sprayer stays ON after actuation begins
                    'hold_time_sec': 2.0,

                    # Relay polarity:
                    # True  -> HIGH turns relay ON
                    # False -> LOW turns relay ON
                    'active_high': True,

                    # Delay between the detection event and spray actuation.
                    # This compensates for the physical distance between the
                    # camera and the sprayer nozzles.
                    'signal_delay_sec': 4.0,
                }]
            )
        )

    else:
        # ------------------------------------------------------------
        # Simulation mode
        # ------------------------------------------------------------
        # This mode does not use YOLO.
        #
        # Instead, it launches a node that randomly generates simulated
        # disease and weed detections and directly drives the GPIO outputs.
        #
        # This is useful when:
        # - YOLO is not working
        # - the model is unavailable
        # - you want to demonstrate spray logic without real detections
        nodes.append(
            Node(
                package='sprayer_vision',
                executable='random_detection_gpio_node',
                name='random_detection_gpio_node',
                output='screen',
                parameters=[{
                    # GPIO pin used for disease spray simulation
                    'disease_gpio_pin': 13,

                    # GPIO pin used for weed spray simulation
                    'weed_gpio_pin': 32,

                    # Relay polarity
                    'active_high': True,

                    # How long the output remains ON after activation
                    'hold_time_sec': 2.0,

                    # Minimum / maximum random spacing between simulated disease detections
                    'disease_min_interval_sec': 0.1,
                    'disease_max_interval_sec': 10.0,

                    # Minimum / maximum random spacing between simulated weed detections
                    'weed_min_interval_sec': 0.1,
                    'weed_max_interval_sec': 10.0,
                }]
            )
        )

    return LaunchDescription(nodes)
