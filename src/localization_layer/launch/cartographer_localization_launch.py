import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_dir = get_package_share_directory('localization_layer')
    config_dir = os.path.join(localization_dir, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    pbstream_filename = LaunchConfiguration('pbstream_filename')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    enable_sensor_bringup = LaunchConfiguration('enable_sensor_bringup')
    ebimu_port = LaunchConfiguration('ebimu_port')
    ebimu_baud = LaunchConfiguration('ebimu_baud')
    lidar_channel_type = LaunchConfiguration('lidar_channel_type')
    lidar_udp_ip = LaunchConfiguration('lidar_udp_ip')
    lidar_udp_port = LaunchConfiguration('lidar_udp_port')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    lidar_angle_compensate = LaunchConfiguration('lidar_angle_compensate')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode')

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
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='IMU topic used by Cartographer',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic used by Cartographer',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used by Cartographer',
        ),
        DeclareLaunchArgument(
            'enable_sensor_bringup',
            default_value='false',
            description='Include sensor_layer launch for IMU/LiDAR/TF when true',
        ),
        DeclareLaunchArgument(
            'ebimu_port',
            default_value='/dev/ttyUSB0',
            description='EBIMU serial port (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'ebimu_baud',
            default_value='115200',
            description='EBIMU baud rate (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'lidar_channel_type',
            default_value='udp',
            description='LiDAR channel type (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'lidar_udp_ip',
            default_value='192.168.11.2',
            description='LiDAR UDP IP (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'lidar_udp_port',
            default_value='8089',
            description='LiDAR UDP port (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='LiDAR frame_id (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'lidar_inverted',
            default_value='false',
            description='LiDAR inverted flag (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'lidar_angle_compensate',
            default_value='true',
            description='LiDAR angle compensation flag (used when sensor bringup is enabled)',
        ),
        DeclareLaunchArgument(
            'lidar_scan_mode',
            default_value='Sensitivity',
            description='LiDAR scan mode (used when sensor bringup is enabled)',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sensor_layer'),
                    'launch',
                    'sensor_layer_launch.py',
                )
            ),
            condition=IfCondition(enable_sensor_bringup),
            launch_arguments={
                'channel_type': lidar_channel_type,
                'udp_ip': lidar_udp_ip,
                'udp_port': lidar_udp_port,
                'frame_id': lidar_frame_id,
                'inverted': lidar_inverted,
                'angle_compensate': lidar_angle_compensate,
                'scan_mode': lidar_scan_mode,
                'ebimu_port': ebimu_port,
                'ebimu_baud': ebimu_baud,
            }.items(),
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
                ('imu', imu_topic),
                ('odom', odom_topic),
                ('scan', scan_topic),
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
