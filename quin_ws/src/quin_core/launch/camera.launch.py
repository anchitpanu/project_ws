import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # ---- OPTION A: SD, low-latency raw (good for testing) ----
    # sd_params = {
    #     'video_device': '/dev/video0',
    #     'camera_name': 'usb_cam',
    #     'image_size': [640, 480],        # VGA
    #     'pixel_format': 'YUYV',          # raw, bigger bandwidth
    #     'output_encoding': 'yuv422_yuy2',
    #     'time_per_frame': [1, 30],       # 30 FPS  (you had [1,10] = 10 FPS)
    # }

    # ---- OPTION B: HD, MJPEG from camera (recommended over Wi-Fi) ----
    # Camera: use YUYV (supported), convert to bgr8
    cam_params = {
        'video_device': '/dev/video0',
        'camera_name': 'usb_cam',
        'image_size': [1280, 720],     # try [960, 540] if Wi-Fi is weak
        'pixel_format': 'YUYV',        # <- supported; avoids MJPG crash
        'output_encoding': 'bgr8',
        'time_per_frame': [1, 30],     # 30 FPS
    }

    qos_overrides = {
        '/image_raw': {
            'publisher': {
                'reliability': 'best_effort',
                'history': 'keep_last',
                'depth': 5,
                'durability': 'volatile'
            }
        },
        '/camera_info': {
            'publisher': {
                'reliability': 'best_effort',
                'history': 'keep_last',
                'depth': 5,
                'durability': 'volatile'
            }
        }
    }

    camera_driver = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[cam_params, {'qos_overrides': qos_overrides}],
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
    )

    ld.add_action(camera_driver)
    return ld

if __name__ == '__main__':
    generate_launch_description()


# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
# # from launch.actions import ExecuteProcess


# def generate_launch_description():
#     ld = LaunchDescription()

#     camera_driver = Node(
#         package='v4l2_camera',
#         executable='v4l2_camera_node',
#         name='camera',
#         parameters=[{
#             'video_device': '/dev/video0',
#             'camera_name': 'usb_cam',
#             'image_size': [320, 240],
#             'pixel_format': 'YUYV',  
#             'output_encoding': 'yuv422_yuy2',
#             'time_per_frame': [1, 10]  # 30 FPS
#         }],
#         remappings=[
#             ('/image_raw', '/quin/image_raw'),
#             ('/camera_info', '/quin/camera_info'),
#         ],
#         arguments=[
#             '--ros-args',
#             '--log-level', 'camera:=error',
#             '--log-level', 'v4l2_camera:=error',
#             '--log-level', 'camera_calibration_parsers:=error',
#             '--log-level', 'camera_calibration_parsers:=fatal',
#         ],
#         # output='screen'
#     )

#     # camera_upright = ExecuteProcess(
#     # cmd=[
#     #     'python3', '/home/quin/project_ws/quin_ws/src/quin_core/scripts/flip_camera.py',
#     #     '--ros-args',
#     #     '-p', 'in_topic:=/quin/image_raw',
#     #     '-p', 'out_topic:=/quin/image_raw/camera_upright'
#     # ],
#     # output='screen'
#     # )

#     ld.add_action(camera_driver)
#     # ld.add_action(camera_upright)

#     return ld

# if __name__ == '__main__':
#     generate_launch_description()