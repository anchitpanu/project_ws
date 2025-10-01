import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    camera_driver = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'camera_name': 'usb_cam',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',  
            'output_encoding': 'yuv422_yuy2',
            'time_per_frame': [1, 30]  # 30 FPS
        }],
        remappings=[
            ('/image_raw', '/quin/image_raw'),
            ('/camera_info', '/quin/camera_info'),
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'camera:=error',
            '--log-level', 'v4l2_camera:=error',
            '--log-level', 'camera_calibration_parsers:=error',
            '--log-level', 'camera_calibration_parsers:=fatal',
        ],
        # output='screen'
    )

    # camera_upright = ExecuteProcess(
    # cmd=[
    #     'python3', '/home/quin/project_ws/quin_ws/src/quin_core/scripts/flip_camera.py',
    #     '--ros-args',
    #     '-p', 'in_topic:=/quin/image_raw',
    #     '-p', 'out_topic:=/quin/image_raw/camera_upright'
    # ],
    # output='screen'
    # )

    ld.add_action(camera_driver)
    # ld.add_action(camera_upright)

    return ld

if __name__ == '__main__':
    generate_launch_description()