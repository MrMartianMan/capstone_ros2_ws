from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py',
                'camera_model:=zedm'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'crop_row_perception', 'centerline_node'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_controller', 'controller_server',
                '--ros-args',
                '--params-file',
                '/home/project-48/capstone_ros2_ws/src/crop_row_perception/config/nav2_params.yaml'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager',
                '--ros-args',
                '-p', 'use_sim_time:=false',
                '-p', 'autostart:=true',
                '-p', "node_names:=['controller_server']"
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'crop_row_perception', 'follow_centerline_nav2_node'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'crop_row_perception', 'cmd_vel_to_can_node'
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'sprayer_vision', 'jetson_sprayer.launch.py'
            ],
            output='screen'
        ),
    ])
