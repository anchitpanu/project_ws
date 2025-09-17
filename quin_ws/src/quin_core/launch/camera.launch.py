import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    camera_driver = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
        output="screen",
        parameters=[{
            "video_device": "/dev/video2",
            "image_size" : [640, 480],              # [width, height]
            "pixel_format": "yuyv",                # 'raw', 'yuyv', 'mjpeg'
            "camera_frame_id": "camera_link",
            "framerate": 30.0,                      # 30 fps
            "io_method": "userptr",
            # "camera_info_url": "file:///home/quin/quin_ws/src/quin_core/config/camera_info.yaml"
        }],
        remappings=[
            ("/image_raw", "/quin/image_raw"),
            ("/camera_info", "/quin/camera_info")
        ]
    )

    ld.add_action(camera_driver)

    return ld

if __name__ == "__main__":
    generate_launch_description()

