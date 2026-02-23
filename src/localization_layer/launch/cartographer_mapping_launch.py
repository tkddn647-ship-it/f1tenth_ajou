import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_dir = get_package_share_directory('localization_layer')
    config_dir = os.path.join(localization_dir, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_auto_save = LaunchConfiguration('enable_auto_save')
    map_save_dir = LaunchConfiguration('map_save_dir')
    map_file_prefix = LaunchConfiguration('map_file_prefix')
    include_unfinished_submaps = LaunchConfiguration('include_unfinished_submaps')
    save_on_shutdown = LaunchConfiguration('save_on_shutdown')
    save_interval_sec = LaunchConfiguration('save_interval_sec')
    export_ros_map = LaunchConfiguration('export_ros_map')
    ros_map_topic = LaunchConfiguration('ros_map_topic')
    ros_map_format = LaunchConfiguration('ros_map_format')
    ros_map_mode = LaunchConfiguration('ros_map_mode')
    ros_map_timeout_sec = LaunchConfiguration('ros_map_timeout_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'enable_auto_save',
            default_value='true',
            description='Enable automatic pbstream export helper node',
        ),
        DeclareLaunchArgument(
            'map_save_dir',
            default_value='/home/tkddn647/test/maps', # 파일 경로는 Jeston Nano의 절대 경로로 지정. 예시 : /home/username/maps
            description='Directory to store exported pbstream files',
        ),
        DeclareLaunchArgument(
            'map_file_prefix',
            default_value='cartographer_map',
            description='Prefix for exported pbstream file names',
        ),
        DeclareLaunchArgument(
            'include_unfinished_submaps',
            default_value='true',
            description='Include unfinished submaps when exporting',
        ),
        DeclareLaunchArgument(
            'save_on_shutdown',
            default_value='true',
            description='Export pbstream automatically when launch is stopped',
        ),
        DeclareLaunchArgument(
            'save_interval_sec',
            default_value='0.0',
            description='Periodic export interval in seconds (0 disables periodic export)',
        ),
        DeclareLaunchArgument(
            'export_ros_map',
            default_value='true',
            description='Also export ROS map image+yaml after pbstream save',
        ),
        DeclareLaunchArgument(
            'ros_map_topic',
            default_value='/map',
            description='Topic used by map_saver_cli',
        ),
        DeclareLaunchArgument(
            'ros_map_format',
            default_value='png',
            description='Output image format for map_saver_cli (png/pgm)',
        ),
        DeclareLaunchArgument(
            'ros_map_mode',
            default_value='trinary',
            description='map_saver_cli mode: trinary/scale/raw',
        ),
        DeclareLaunchArgument(
            'ros_map_timeout_sec',
            default_value='20.0',
            description='Timeout for map_saver_cli command',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d_mapping.lua',
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
        Node(
            package='localization_layer',
            executable='map_auto_saver.py',
            name='map_auto_saver',
            output='screen',
            condition=IfCondition(enable_auto_save),
            parameters=[{
                'map_save_dir': map_save_dir,
                'map_file_prefix': map_file_prefix,
                'include_unfinished_submaps': ParameterValue(
                    include_unfinished_submaps, value_type=bool
                ),
                'save_on_shutdown': ParameterValue(save_on_shutdown, value_type=bool),
                'save_interval_sec': ParameterValue(save_interval_sec, value_type=float),
                'export_ros_map': ParameterValue(export_ros_map, value_type=bool),
                'ros_map_topic': ros_map_topic,
                'ros_map_format': ros_map_format,
                'ros_map_mode': ros_map_mode,
                'ros_map_timeout_sec': ParameterValue(ros_map_timeout_sec, value_type=float),
            }],
        ),
    ])
