#!/usr/bin/env python3
"""
cartographer.launch.py — Cartographer 2D SLAM for UGV Rover

Assumes rover_bringup robot.launch.py is already running, which provides:
  - LD19 LIDAR              -> /scan
  - odom_publisher          -> /odom + odom->base_footprint TF
  - imu_filter_madgwick     -> /imu/data
  - robot_state_publisher   -> base_footprint->base_link TF (from URDF)

This launch file adds:
  - USB camera driver (usb_cam)     -> /image_raw
  - Camera viewer (rover_vision)
  - Cartographer SLAM               -> /map

Full mapping session workflow:
  # Terminal 1 - robot hardware
  ros2 launch rover_bringup robot.launch.py

  # Terminal 2 - Cartographer SLAM + camera
  ros2 launch rover_mapping cartographer.launch.py

  # Terminal 3 - drive to build the map
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # Terminal 4 - save map when done
  ros2 launch rover_mapping map_saver.launch.py map_name:=my_room

Optional args:
  ros2 launch rover_mapping cartographer.launch.py camera_device:=/dev/video2

Prerequisites:
  sudo apt install ros-jazzy-cartographer ros-jazzy-cartographer-ros
  sudo apt install ros-jazzy-usb-cam
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -- Arguments -------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='USB camera device path'
    )

    use_sim_time  = LaunchConfiguration('use_sim_time')
    camera_device = LaunchConfiguration('camera_device')

    # -- Package paths ---------------------------------------------------------
    pkg_rover_mapping = get_package_share_directory('rover_mapping')
    pkg_rover_vision  = get_package_share_directory('rover_vision')

    cartographer_config_dir = os.path.join(pkg_rover_mapping, 'config')
    cartographer_config_basename = 'rover_ld19.lua'

    rover_vision_config = os.path.join(pkg_rover_vision, 'config', 'camera_params.yaml')

    # -- USB Camera Driver -----------------------------------------------------
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': camera_device,
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_link',
            'io_method': 'mmap',
        }],
        output='screen',
    )

    # -- Camera Viewer ---------------------------------------------------------
    camera_viewer = Node(
        package='rover_vision',
        executable='camera_viewer',
        name='camera_viewer',
        parameters=[rover_vision_config],
        output='screen',
    )

    # -- Cartographer ----------------------------------------------------------
    # Reads rover_ld19.lua from rover_mapping/config/
    # Subscribes: /scan, /odom
    # Publishes:  /map, /submap_list, pose TF (map->odom)
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
        output='screen',
    )

    # -- Cartographer Occupancy Grid -------------------------------------------
    # Converts Cartographer's submaps into a standard /map topic
    # that RViz2 and nav2_map_server can read
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},
            {'publish_period_sec': 1.0},
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_device_arg,
        usb_cam_node,
        camera_viewer,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])