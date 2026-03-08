#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    slam_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rover_mapping'),
                        'launch',
                        'slam.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        lidar_port_arg,
        use_sim_time_arg,
        use_imu_heading_arg,
        lidar_node,
        odom_node,
        static_tf_node,
        slam_launch,
    ])