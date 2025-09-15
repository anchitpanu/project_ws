import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Define the path to the launch file directory
    launch_file_dir = os.path.join(get_package_share_directory('quin_core'), 'launch')
    
    
    # Include microros.launch.py
    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'microros.launch.py')
        )
    )
    
    
    cmd_move = Node(
        package="quin_core",
        executable="cmd_move.py",
        name="Cmd_Move",
        # output="screen",
        namespace="",
        # parameters=[], #Testing
    )

    # apriltag_auto_aim = Node(
    #     package="quin_core",
    #     executable="apriltag_auto_aim",
    #     name="Apriltag_Distance",
    #     # output="screen",
    #     namespace="",
    #     # parameters=[], #Testing
    # )

    # hoop_detection = Node(
    #     package="quin_core",
    #     executable="hoop_detection.py",
    #     name="Hoop_Detection",
    #     output="screen",
    #     namespace="",
    #     # parameters=[], #Testing
    # )

    # camera_driver = Node(
    #     package='v4l2_camera',
    #     executable='v4l2_camera_node',
    #     name='camera',
    #     parameters=[{
    #         'video_device': '/dev/video0',
    #         'image_size': [640, 480],
    #         'time_per_frame': [1, 20]  # 30 FPS
    #     }],
    #     remappings=[
    #         ('/image_raw', '/shaq/image_raw')
    #     ],
    #     output='screen'
    # )



    # Add actions to the launch description
    ld.add_action(microros_launch)
    ld.add_action(cmd_move)
    # ld.add_action(camera_driver)
    # ld.add_action(apriltag_auto_aim)
    # ld.add_action(hoop_detection)


    return ld

if __name__ == '__main__':
    generate_launch_description()
