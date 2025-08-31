#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    motor_config = os.path.join(
        get_package_share_directory('quin_core'),
        'config',
        'motor_config.yaml'
    )

    joy = Node(
        package="joy",
        executable="joy_node",
        name="Joy_Node",
        # output="screen",
        namespace="",
        # parameters=[{"autorepeat_rate": 50.0}],
        # arguments=["--dev", "/dev/input/js0"],  # replace with your joystick device path
        remappings = [
            ('/joy', '/quin/joy')
        ]
    )

    joystick_control = Node(
        package="quin_core",
        executable="joystick_control",
        name="Joystick_Node",
        # output="screen",
        namespace="",
    )
    
    
    cmd_vel_to_motor_speed = Node(
        package="quin_core",
        executable="cmd_move",
        name="Cmd_Vel_To_Rpm",
        # output="screen",
        namespace="",
        # parameters=[motor_config], #Testing
    )
    
    # joy_auto = Node(
    #     package="quin_core",
    #     executable="joystick_control_3sec.py",
    #     name="joy_auto",
    #     # output="screen",
    #     namespace="",
    # )
    
    ld.add_action(joy)
    ld.add_action(joystick_control)
    # ld.add_action(joy_auto)
    # ld.add_action(cmd_vel_to_motor_speed)

    return ld
