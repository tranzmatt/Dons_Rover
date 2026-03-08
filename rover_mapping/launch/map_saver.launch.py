#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('rover_mapping')
    default_map_dir = os.path.join(pkg_share, 'maps')
    os.makedirs(default_map_dir, exist_ok=True)

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='rover_map',
        description='Base filename for the saved map (no extension)'
    )
    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value=default_map_dir,
        description='Directory to save map files into'
    )

    map_name = LaunchConfiguration('map_name')
    map_dir  = LaunchConfiguration('map_dir')

    save_map = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', PathJoinSubstitution([map_dir, map_name]),
            '--ros-args',
            '-p', 'map_subscribe_transient_local:=true',
        ],
        output='screen',
    )

    return LaunchDescription([
        map_name_arg,
        map_dir_arg,
        save_map,
    ])