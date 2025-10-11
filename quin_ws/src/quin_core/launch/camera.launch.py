#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Optional: allow parameter file override
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/home/quin/project_ws/quin_ws/src/quin_core/config/camera.yaml",
        description="Camera parameters YAML file",
    )
    params_file = LaunchConfiguration("params_file")

    # ---- Camera parameters (your configuration) ----
    cam_params = {
        "video_device": "/dev/video0",   # change to /dev/cam_main if you created a udev alias
        "camera_name": "usb_cam",
        "image_size": [1280, 720],
        "pixel_format": "YUYV",          # using YUYV to avoid MJPG crashes
        "output_encoding": "bgr8",
        "time_per_frame": [1, 30],       # 30 FPS
    }

    camera_driver = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[cam_params],  # or [params_file] if you prefer loading from YAML
        remappings=[
            ("/image_raw", "/quin/image_raw"),
            ("/camera_info", "/quin/camera_info"),
        ],
        arguments=[
            "--ros-args",
            "--log-level", "camera:=error",
            "--log-level", "v4l2_camera:=error",
            "--log-level", "camera_calibration_parsers:=error",
            "--log-level", "camera_calibration_parsers:=fatal",
        ],
    )

    # ---- Web video server (HTTP MJPEG stream) ----
    wvs_node = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        arguments=[],  # e.g. ["--port", "8080"]
    )

    # Delay startup by 1 s to ensure camera is detected
    delayed = TimerAction(period=1.0, actions=[camera_driver, wvs_node])

    ld.add_action(params_file_arg)
    ld.add_action(delayed)

    return ld


if __name__ == "__main__":
    generate_launch_description()
