import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_dir = get_package_share_directory('localization_layer')
    config_dir = os.path.join(localization_dir, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    pbstream_filename = LaunchConfiguration('pbstream_filename')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'pbstream_filename',
            description='Absolute path to .pbstream map file',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d_localization.lua',
                '-load_state_filename', pbstream_filename,
                '-load_frozen_state', 'true',
            ],
            remappings=[
                ('imu', '/ebimu/imu'),
                ('odom', '/odom'),
                ('scan', '/scan'),
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
