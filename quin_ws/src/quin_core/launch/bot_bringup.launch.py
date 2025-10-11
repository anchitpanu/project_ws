import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # # Define the path to the launch file directory
    # launch_file_dir = os.path.join(get_package_share_directory('quin_core'), 'launch')
    
    
    # # Include microros.launch.py
    # microros_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'microros.launch.py')
    #     )
    # )
    
    node_microros_1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )
    
    node_microros_2 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB1"],
    ) 

    # node_microros_3 = Node(
    #     package="micro_ros_agent",
    #     executable="micro_ros_agent",
    #     output="screen",
    #     arguments=["serial", "--dev", "/dev/ttyUSB2"],
    # ) 

    cmd_move = Node(
        package="quin_core",
        executable="cmd_move.py",
        name="Cmd_Move",
        # output="screen",
        namespace="",
        # parameters=[], #Testing
    )



    # Add actions to the launch description
    # ld.add_action(microros_launch)
    # ld.add_action(cmd_move)
    ld.add_action(node_microros_1)
    ld.add_action(node_microros_2) 
    # ld.add_action(node_microros_3)

    return ld
