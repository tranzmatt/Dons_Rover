from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('rover_vision')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    
    # Declare arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Camera image topic to subscribe to'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Show OpenCV window (set to false when SSH without X11)'
    )
    
    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='true',
        description='Use config file or command line parameters'
    )
    
    # Camera viewer node
    camera_viewer = Node(
        package='rover_vision',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[config_file] if LaunchConfiguration('use_config') else [{
            'image_topic': LaunchConfiguration('image_topic'),
            'show_window': LaunchConfiguration('show_window'),
        }]
    )
    
    return LaunchDescription([
        image_topic_arg,
        show_window_arg,
        use_config_arg,
        camera_viewer
    ])