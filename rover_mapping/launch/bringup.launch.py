#!/usr/bin/env python3
"""
Usage:
  ros2 launch rover_mapping bringup.launch.py

Optional args:
  ros2 launch rover_mapping bringup.launch.py lidar_port:=/dev/ttyUSB1
  ros2 launch rover_mapping bringup.launch.py use_imu_heading:=true
"""

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
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg


def generate_launch_description():

    # ── Arguments ───────────────────────────────────────────────────────────
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LD19 LIDAR'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    use_imu_heading_arg = DeclareLaunchArgument(
        'use_imu_heading',
        default_value='false',
        description='Use IMU yaw for heading instead of wheel odometry'
    )

    lidar_port      = LaunchConfiguration('lidar_port')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    use_imu_heading = LaunchConfiguration('use_imu_heading')

    # ── 1. LD19 LIDAR ────────────────────────────────────────────────────────
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_node',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'port_name': lidar_port,
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
        }],
        output='screen',
    )

    # ── 2. Odometry Publisher (from rover_motion) ────────────────────────────
    # Subscribes to: odom/odom_raw (Float32MultiArray from firmware)
    # Publishes:     /odom (nav_msgs/Odometry)
    # Broadcasts TF: odom → base_footprint
    odom_node = Node(
        package='rover_motion',
        executable='odom_publisher',
        name='odom_publisher',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'odom_frame':           'odom',
            'base_footprint_frame': 'base_footprint',
            'base_frame':           'base_link',
            'pub_odom_tf':          True,
            'wheel_separation':     0.284,
            'publish_rate':         10.0,
            'use_imu_heading':      use_imu_heading,
        }],
        output='screen',
    )

    # ── 3. Static TF: base_footprint → base_link ─────────────────────────────
    # The Rover is 2D so this is a zero-offset transform.
    # slam_toolbox needs base_footprint; other Nav2 nodes expect base_link.
    # This static broadcast satisfies both.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0',   # x y z translation
                   '0', '0', '0',   # roll pitch yaw
                   'base_footprint',
                   'base_link'],
        output='screen',
    )

    # ── 4. SLAM Toolbox ──────────────────────────────────────────────────────
    # Delayed 3 seconds so LIDAR and odom are publishing before SLAM starts.
    # If you see "waiting for transform" errors on first run, increase to 5.0.
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
        lidar_port_arg,
        use_sim_time_arg,
        use_imu_heading_arg,
        lidar_node,
        odom_node,
        static_tf_node,
        slam_delayed,
    ])