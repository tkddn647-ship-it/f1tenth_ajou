import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("localization_layer")
    localization_launch = os.path.join(pkg_dir, "launch", "cartographer_localization_launch.py")

    pbstream_filename = LaunchConfiguration("pbstream_filename")
    world_type = LaunchConfiguration("world_type")
    map_yaml_path = LaunchConfiguration("map_yaml_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    random_seed = LaunchConfiguration("random_seed")
    odom_velocity_scale_error = LaunchConfiguration("odom_velocity_scale_error")
    odom_yaw_rate_bias_deg = LaunchConfiguration("odom_yaw_rate_bias_deg")
    odom_yaw_rate_noise_std_deg = LaunchConfiguration("odom_yaw_rate_noise_std_deg")
    imu_yaw_bias_deg = LaunchConfiguration("imu_yaw_bias_deg")
    imu_ang_vel_bias_deg = LaunchConfiguration("imu_ang_vel_bias_deg")
    imu_ang_vel_noise_std_deg = LaunchConfiguration("imu_ang_vel_noise_std_deg")
    lidar_rate_hz = LaunchConfiguration("lidar_rate_hz")
    lidar_range_max = LaunchConfiguration("lidar_range_max")
    scan_noise_std_m = LaunchConfiguration("scan_noise_std_m")
    scan_dropout_ratio = LaunchConfiguration("scan_dropout_ratio")
    scan_outlier_ratio = LaunchConfiguration("scan_outlier_ratio")
    scan_outlier_max_m = LaunchConfiguration("scan_outlier_max_m")
    imu_rate_hz = LaunchConfiguration("imu_rate_hz")
    odom_rate_hz = LaunchConfiguration("odom_rate_hz")
    state_rate_hz = LaunchConfiguration("state_rate_hz")
    map_path_speed_mps = LaunchConfiguration("map_path_speed_mps")

    return LaunchDescription([
        DeclareLaunchArgument("pbstream_filename", description="Absolute path to .pbstream map file"),
        DeclareLaunchArgument("world_type", default_value="map"),
        DeclareLaunchArgument(
            "map_yaml_path", default_value="/home/tkddn647/test/maps/Austin_map.yaml"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("random_seed", default_value="42"),
        DeclareLaunchArgument("odom_velocity_scale_error", default_value="0.005"),
        DeclareLaunchArgument("odom_yaw_rate_bias_deg", default_value="0.1"),
        DeclareLaunchArgument("odom_yaw_rate_noise_std_deg", default_value="0.05"),
        DeclareLaunchArgument("imu_yaw_bias_deg", default_value="0.0"),
        DeclareLaunchArgument("imu_ang_vel_bias_deg", default_value="0.05"),
        DeclareLaunchArgument("imu_ang_vel_noise_std_deg", default_value="0.03"),
        DeclareLaunchArgument("lidar_rate_hz", default_value="40.0"),
        DeclareLaunchArgument("lidar_range_max", default_value="40.0"),
        DeclareLaunchArgument("scan_noise_std_m", default_value="0.01"),
        DeclareLaunchArgument("scan_dropout_ratio", default_value="0.005"),
        DeclareLaunchArgument("scan_outlier_ratio", default_value="0.003"),
        DeclareLaunchArgument("scan_outlier_max_m", default_value="0.25"),
        DeclareLaunchArgument("imu_rate_hz", default_value="1000.0"),
        DeclareLaunchArgument("odom_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("state_rate_hz", default_value="1000.0"),
        DeclareLaunchArgument("map_path_speed_mps", default_value="2.2"),
        Node(
            package="tf_manager_cpp",
            executable="sensor_static_tf",
            name="sensor_static_tf_node",
            output="screen",
        ),
        Node(
            package="localization_layer",
            executable="sim_fake_sensor_publisher.py",
            name="sim_fake_sensor_publisher",
            output="screen",
            parameters=[{
                "lidar_rate_hz": lidar_rate_hz,
                "imu_rate_hz": imu_rate_hz,
                "odom_rate_hz": odom_rate_hz,
                "state_rate_hz": state_rate_hz,
                "scan_range_min": 0.05,
                "scan_range_max": lidar_range_max,
                "scan_angle_min_deg": -135.0,
                "scan_angle_max_deg": 135.0,
                "scan_angle_increment_deg": 0.12,
                "scan_noise_std_m": scan_noise_std_m,
                "scan_dropout_ratio": scan_dropout_ratio,
                "scan_outlier_ratio": scan_outlier_ratio,
                "scan_outlier_max_m": scan_outlier_max_m,
                "publish_static_tf": False,
                "world_type": world_type,
                "map_yaml_path": map_yaml_path,
                "map_path_speed_mps": map_path_speed_mps,
                "random_seed": random_seed,
                "odom_velocity_scale_error": odom_velocity_scale_error,
                "odom_yaw_rate_bias_deg": odom_yaw_rate_bias_deg,
                "odom_yaw_rate_noise_std_deg": odom_yaw_rate_noise_std_deg,
                "imu_yaw_bias_deg": imu_yaw_bias_deg,
                "imu_ang_vel_bias_deg": imu_ang_vel_bias_deg,
                "imu_ang_vel_noise_std_deg": imu_ang_vel_noise_std_deg,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "pbstream_filename": pbstream_filename,
                "use_sim_time": use_sim_time,
                "imu_topic": "/ebimu/imu",
                "odom_topic": "/odom",
                "scan_topic": "/scan",
                "enable_sensor_bringup": "false",
            }.items(),
        ),
    ])
