#!/usr/bin/env python3
"""
slam.launch.py — SLAM mapping launch for UGV Rover

Assumes rover_bringup robot.launch.py is already running, which provides:
  - LD19 LIDAR              -> /scan
  - odom_publisher          -> /odom (wheel encoders, TF disabled)
  - robot_state_publisher   -> base_footprint->base_link TF (from URDF)

This launch file adds:
  - USB camera driver (usb_cam)         -> /image_raw
  - Camera viewer (rover_vision)
  - rf2o laser odometry                 -> /odom_rf2o (TF disabled)
  - Odometry filter                     -> /odom_filtered + odom->base_footprint TF
  - SLAM Toolbox (async, lifecycle-managed)

Important: rover_params.yaml must have pub_odom_tf: false so that
odom_publisher does not conflict with the odom_filter TF broadcast.

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
  ros2 launch rover_mapping slam.launch.py linear_threshold:=0.01
  ros2 launch rover_mapping slam.launch.py angular_threshold:=0.03

Prerequisites:
  sudo apt install ros-jazzy-slam-toolbox
  sudo apt install ros-jazzy-nav2-map-server
  sudo apt install ros-jazzy-usb-cam
  # rf2o built from source in rover_ws
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
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Camera image topic to subscribe to'
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='Show OpenCV window (set to true only with X11 forwarding)'
    )
    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='true',
        description='Use config file or command line parameters'
    )
    linear_threshold_arg = DeclareLaunchArgument(
        'linear_threshold',
        default_value='0.005',
        description='Linear velocity noise threshold in m/s (increase if map twitches)'
    )
    angular_threshold_arg = DeclareLaunchArgument(
        'angular_threshold',
        default_value='0.02',
        description='Angular velocity noise threshold in rad/s (increase if map rotates)'
    )

    use_sim_time       = LaunchConfiguration('use_sim_time')
    camera_device      = LaunchConfiguration('camera_device')
    linear_threshold   = LaunchConfiguration('linear_threshold')
    angular_threshold  = LaunchConfiguration('angular_threshold')

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

    # -- rf2o Laser Odometry ---------------------------------------------------
    # publish_tf is FALSE — odom_filter takes over TF broadcasting.
    # This prevents phantom motion from scan noise reaching slam_toolbox directly.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic':     '/scan',
            'odom_topic':           '/odom_rf2o',
            'publish_tf':           False,
            'base_frame_id':        'base_footprint',
            'odom_frame_id':        'odom',
            'init_pose_from_topic': '',
            'freq':                 10.0,
        }],
        output='screen',
    )

    # -- Odometry Filter -------------------------------------------------------
    # Subscribes to /odom_rf2o, zeroes velocities below noise threshold,
    # publishes /odom_filtered and broadcasts odom->base_footprint TF.
    # Tune linear_threshold and angular_threshold to stop map twitching.
    odom_filter_node = Node(
        package='rover_mapping',
        executable='odom_filter.py',
        name='odom_filter',
        parameters=[{
            'linear_threshold':  linear_threshold,
            'angular_threshold': angular_threshold,
            'odom_frame':        'odom',
            'base_frame':        'base_footprint',
        }],
        output='screen',
    )

    # -- SLAM Toolbox ----------------------------------------------------------
    # Delayed 3 seconds to ensure /scan and odom_filter are publishing first.
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
        image_topic_arg,
        show_window_arg,
        use_config_arg,
        linear_threshold_arg,
        angular_threshold_arg,
        usb_cam_node,
        camera_viewer,
        rf2o_node,
        odom_filter_node,
        slam_delayed,
    ])