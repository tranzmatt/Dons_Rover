#!/usr/bin/env python3
"""
SLAM launch file for UGV Rover
Launches slam_toolbox for online mapping with lifecycle management
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import lifecycle_msgs.msg
import os

def generate_launch_description():
    # Get paths
    pkg_share = get_package_share_directory('rover_mapping')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    
    # Debug: print the path
    print(f"Loading SLAM params from: {slam_params_file}")
    
    # Verify file exists
    if not os.path.exists(slam_params_file):
        print(f"ERROR: Config file not found at {slam_params_file}")
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # SLAM Toolbox Node (as LifecycleNode)
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.2,
                'minimum_travel_heading': 0.2,
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                'link_match_minimum_response_fine': 0.1,
                'link_scan_maximum_distance': 1.5,
                'loop_search_maximum_distance': 3.0,
                'do_loop_closing': True,
                'solver_plugin': 'solver_plugins::CeresSolver',
            }
        ],
        output='screen',
    )
    
    # Configure event - triggered when node starts
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    
    # Activate event - triggered after configure succeeds
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
    
    # Trigger configure when process starts
    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[configure_event],
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        slam_node,
        configure_on_start,
        activate_event,
    ])