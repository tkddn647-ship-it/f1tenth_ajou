import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    export_ros_map_on_shutdown = LaunchConfiguration('export_ros_map_on_shutdown')
    ros_map_topic = LaunchConfiguration('ros_map_topic')
    ros_map_format = LaunchConfiguration('ros_map_format')
    ros_map_mode = LaunchConfiguration('ros_map_mode')
    ros_map_timeout_sec = LaunchConfiguration('ros_map_timeout_sec')
    write_state_timeout_sec = LaunchConfiguration('write_state_timeout_sec')
    shutdown_write_state_timeout_sec = LaunchConfiguration('shutdown_write_state_timeout_sec')
    shutdown_ros_map_timeout_sec = LaunchConfiguration('shutdown_ros_map_timeout_sec')
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
            'enable_auto_save',
            default_value='true',
            description='Enable automatic pbstream export helper node',
        ),
        DeclareLaunchArgument(
            'map_save_dir',
            default_value='/home/tkddn647/test/maps',  # 파일 경로는 Jeston Nano의 절대 경로로 지정. 예시 : /home/username/maps
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
            'export_ros_map_on_shutdown',
            default_value='true',
            description='Also export ROS map image+yaml on shutdown auto-save',
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
        DeclareLaunchArgument(
            'write_state_timeout_sec',
            default_value='60.0',
            description='Timeout for /write_state service response during runtime saves',
        ),
        DeclareLaunchArgument(
            'shutdown_write_state_timeout_sec',
            default_value='60.0',
            description='Timeout for /write_state service response during shutdown save',
        ),
        DeclareLaunchArgument(
            'shutdown_ros_map_timeout_sec',
            default_value='40.0',
            description='Timeout for map_saver_cli during shutdown map export',
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
                '-configuration_basename', 'cartographer_2d_mapping.lua',
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
        Node(
            package='localization_layer',
            executable='map_auto_saver.py',
            name='map_auto_saver',
            output='screen',
            condition=IfCondition(enable_auto_save),
            sigterm_timeout='20',
            sigkill_timeout='20',
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
                'write_state_timeout_sec': ParameterValue(
                    write_state_timeout_sec, value_type=float
                ),
                'export_ros_map_on_shutdown': ParameterValue(
                    export_ros_map_on_shutdown, value_type=bool
                ),
                'shutdown_write_state_timeout_sec': ParameterValue(
                    shutdown_write_state_timeout_sec, value_type=float
                ),
                'shutdown_ros_map_timeout_sec': ParameterValue(
                    shutdown_ros_map_timeout_sec, value_type=float
                ),
            }],
        ),
    ])
