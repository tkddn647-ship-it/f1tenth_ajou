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
    sim_pkg_dir = get_package_share_directory("sim_test")
    localization_launch = os.path.join(pkg_dir, "launch", "cartographer_localization_launch.py")
    default_sensor_params = os.path.join(sim_pkg_dir, "config", "sim_sensor_params_localization.yaml")

    pbstream_filename = LaunchConfiguration("pbstream_filename")
    world_type = LaunchConfiguration("world_type")
    map_yaml_path = LaunchConfiguration("map_yaml_path")
    centerline_csv_path = LaunchConfiguration("centerline_csv_path")
    sensor_params_file = LaunchConfiguration("sensor_params_file")
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
        DeclareLaunchArgument(
            "pbstream_filename",
            description="Absolute path to .pbstream map file used for pure localization",
        ),
        DeclareLaunchArgument(
            "world_type",
            default_value="map",
            description="Fake world type: racing, room, or map",
        ),
        DeclareLaunchArgument(
            "map_yaml_path",
            default_value="/home/tkddn647/test/maps/example_map.yaml",
            description="YAML file path for occupancy map world (used when world_type=map)",
        ),
        DeclareLaunchArgument(
            "centerline_csv_path",
            default_value="",
            description="CSV centerline path (used when available)",
        ),
        DeclareLaunchArgument(
            "sensor_params_file",
            default_value=default_sensor_params,
            description="YAML parameter file for sim fake sensor publisher",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "random_seed",
            default_value="42",
            description="Random seed used by fake sensor noise generator",
        ),
        DeclareLaunchArgument(
            "odom_velocity_scale_error",
            default_value="0.005",
            description="Relative scale error on odom linear velocity (e.g. 0.02 = +2%)",
        ),
        DeclareLaunchArgument(
            "odom_yaw_rate_bias_deg",
            default_value="0.1",
            description="Constant odom yaw-rate bias in deg/s",
        ),
        DeclareLaunchArgument(
            "odom_yaw_rate_noise_std_deg",
            default_value="0.05",
            description="Gaussian noise std of odom yaw-rate in deg/s",
        ),
        DeclareLaunchArgument(
            "imu_yaw_bias_deg",
            default_value="0.0",
            description="Constant IMU orientation yaw bias in deg",
        ),
        DeclareLaunchArgument(
            "imu_ang_vel_bias_deg",
            default_value="0.05",
            description="Constant IMU angular velocity bias in deg/s",
        ),
        DeclareLaunchArgument(
            "imu_ang_vel_noise_std_deg",
            default_value="0.03",
            description="Gaussian noise std of IMU angular velocity in deg/s",
        ),
        DeclareLaunchArgument(
            "lidar_rate_hz",
            default_value="8.0",
            description="Fake LiDAR publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "lidar_range_max",
            default_value="8.0",
            description="Fake LiDAR maximum range in meters",
        ),
        DeclareLaunchArgument(
            "scan_noise_std_m",
            default_value="0.01",
            description="Gaussian stddev of fake LiDAR range noise in meters",
        ),
        DeclareLaunchArgument(
            "scan_dropout_ratio",
            default_value="0.005",
            description="Per-ray dropout ratio (0.0~1.0) for fake LiDAR",
        ),
        DeclareLaunchArgument(
            "scan_outlier_ratio",
            default_value="0.003",
            description="Per-ray outlier ratio (0.0~1.0) for fake LiDAR",
        ),
        DeclareLaunchArgument(
            "scan_outlier_max_m",
            default_value="0.25",
            description="Maximum absolute outlier offset in meters",
        ),
        DeclareLaunchArgument(
            "imu_rate_hz",
            default_value="8.0",
            description="Fake IMU publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "odom_rate_hz",
            default_value="8.0",
            description="Fake wheel odom publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "state_rate_hz",
            default_value="50.0",
            description="Internal ground-truth integration rate in Hz",
        ),
        DeclareLaunchArgument(
            "map_path_speed_mps",
            default_value="0.5",
            description="Target speed along extracted map centerline",
        ),
        Node(
            package="tf_manager_cpp",
            executable="sensor_static_tf",
            name="sensor_static_tf_node",
            output="screen",
        ),
        Node(
            package="sim_test",
            executable="sim_fake_sensor_publisher.py",
            name="sim_fake_sensor_publisher",
            output="screen",
            remappings=[
                ("/scan", "/sim_scan"),
                ("/odom", "/sim_odom"),
                ("/ebimu/imu", "/sim_imu"),
            ],
            parameters=[
                sensor_params_file,
                {
                    "lidar_rate_hz": lidar_rate_hz,
                    "imu_rate_hz": imu_rate_hz,
                    "odom_rate_hz": odom_rate_hz,
                    "state_rate_hz": state_rate_hz,
                    "scan_range_min": 0.05,
                    "scan_range_max": lidar_range_max,
                    "scan_angle_min_deg": -80.0,
                    "scan_angle_max_deg": 80.0,
                    "scan_angle_increment_deg": 2.0,
                    "scan_noise_std_m": scan_noise_std_m,
                    "scan_dropout_ratio": scan_dropout_ratio,
                    "scan_outlier_ratio": scan_outlier_ratio,
                    "scan_outlier_max_m": scan_outlier_max_m,
                    "publish_static_tf": False,
                    "publish_odom_tf": False,
                    "world_type": world_type,
                    "map_yaml_path": map_yaml_path,
                    "centerline_csv_path": centerline_csv_path,
                    "map_path_speed_mps": map_path_speed_mps,
                    "map_min_speed_mps": 0.5,
                    "map_use_pure_pursuit": True,
                    "map_use_waypoint_heading": False,
                    "random_seed": random_seed,
                    "odom_velocity_scale_error": odom_velocity_scale_error,
                    "odom_yaw_rate_bias_deg": odom_yaw_rate_bias_deg,
                    "odom_yaw_rate_noise_std_deg": odom_yaw_rate_noise_std_deg,
                    "imu_yaw_bias_deg": imu_yaw_bias_deg,
                    "imu_ang_vel_bias_deg": imu_ang_vel_bias_deg,
                    "imu_ang_vel_noise_std_deg": imu_ang_vel_noise_std_deg,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "pbstream_filename": pbstream_filename,
                "use_sim_time": use_sim_time,
                "imu_topic": "/sim_imu",
                "odom_topic": "/sim_odom",
                "scan_topic": "/sim_scan",
                "enable_sensor_bringup": "false",
            }.items(),
        ),
    ])
