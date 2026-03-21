#!/usr/bin/env python3
"""
slam.launch.py — SLAM mapping launch for UGV Rover

Assumes rover_bringup robot.launch.py is already running, which provides:
  - LD19 LIDAR              -> /scan
  - odom_publisher          -> /odom + odom->base_footprint TF
  - robot_state_publisher   -> base_footprint->base_link TF (from URDF)

This launch file adds:
  - USB camera driver (usb_cam)         -> /image_raw
  - Camera viewer (rover_vision)
  - SLAM Toolbox (async, lifecycle-managed)

Full mapping session workflow:
  # Terminal 1 - robot hardware
  ros2 launch rover_bringup robot.launch.py

  # Terminal 2 - SLAM + camera (this file)
  ros2 launch rover_mapping slam.launch.py

  # Terminal 3 - drive to build the map
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # Terminal 4 - save map when done
  ros2 launch rover_mapping map_saver.launch.py map_name:=my_room

Optional args:
  ros2 launch rover_mapping slam.launch.py camera_device:=/dev/video2

Prerequisites:
  sudo apt install ros-jazzy-slam-toolbox
  sudo apt install ros-jazzy-nav2-map-server
  sudo apt install ros-jazzy-usb-cam
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


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
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='Show OpenCV window (set to true only with X11 forwarding)'
    )

    use_sim_time  = LaunchConfiguration('use_sim_time')
    camera_device = LaunchConfiguration('camera_device')

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
    rover_vision_config = os.path.join(
        get_package_share_directory('rover_vision'),
        'config',
        'camera_params.yaml'
    )

    camera_viewer = Node(
        package='rover_vision',
        executable='camera_viewer',
        name='camera_viewer',
        parameters=[rover_vision_config],
        output='screen',
    )

    # -- SLAM Toolbox ----------------------------------------------------------
    # Delayed 3 seconds to ensure /scan and /odom are publishing first.
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_mapping'),
                'config',
                'slam_toolbox_params.yaml'
            ]),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[configure_event],
        )
    )

    slam_delayed = TimerAction(
        period=3.0,
        actions=[slam_node, configure_on_start, activate_event],
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_device_arg,
        show_window_arg,
        usb_cam_node,
        camera_viewer,
        slam_delayed,
    ])