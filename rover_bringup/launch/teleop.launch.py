#!/usr/bin/env python3
"""
Teleop launch file for UGV Rover
Launches joystick OR keyboard teleop for manual control
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_bringup')
    config_file = os.path.join(pkg_share, 'config', 'rover_params.yaml')
    joy_config_file = os.path.join(pkg_share, 'config', 'joystick_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_joystick = LaunchConfiguration('use_joystick', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'use_joystick',
            default_value='true',
            description='Use joystick teleop if true, keyboard teleop if false'
        ),

        # Joy Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
                'use_sim_time': use_sim_time
            }],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_joystick)
        ),

        # Joy Teleop Node
        Node(
            package='rover_bringup',
            executable='joy_teleop.py',
            name='joy_teleop',
            parameters=[
                config_file,        # rover_params.yaml (speed limits etc.)
                joy_config_file,    # joystick_params.yaml (buttons, axes, headlights)
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_joystick)
        ),

        # Keyboard Teleop Node
        Node(
            package='rover_bringup',
            executable='keyboard_teleop.py',
            name='keyboard_teleop',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(use_joystick)
        ),
    ])