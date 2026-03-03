import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg_dir = get_package_share_directory("sim_test")
    loc_pkg_dir = get_package_share_directory("localization_layer")
    mapping_launch = os.path.join(loc_pkg_dir, "launch", "cartographer_mapping_launch.py")
    default_params = os.path.join(sim_pkg_dir, "config", "sim_sensor_params_mapping.yaml")

    map_save_dir = LaunchConfiguration("map_save_dir")
    map_file_prefix = LaunchConfiguration("map_file_prefix")
    save_interval_sec = LaunchConfiguration("save_interval_sec")
    world_type = LaunchConfiguration("world_type")
    map_yaml_path = LaunchConfiguration("map_yaml_path")
    centerline_csv_path = LaunchConfiguration("centerline_csv_path")
    centerline_auto_align_to_map = LaunchConfiguration("centerline_auto_align_to_map")
    centerline_offset_x_m = LaunchConfiguration("centerline_offset_x_m")
    centerline_offset_y_m = LaunchConfiguration("centerline_offset_y_m")
    sensor_params_file = LaunchConfiguration("sensor_params_file")
    map_path_speed_mps = LaunchConfiguration("map_path_speed_mps")

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument("map_save_dir", default_value="/home/tkddn647/test/maps"),
        DeclareLaunchArgument("map_file_prefix", default_value="sim_map"),
        DeclareLaunchArgument("save_interval_sec", default_value="0.0"),
        DeclareLaunchArgument("world_type", default_value="map"),
        DeclareLaunchArgument("map_yaml_path", default_value="/home/tkddn647/test/maps/example_map.yaml"),
        DeclareLaunchArgument(
            "centerline_csv_path",
            default_value="",
        ),
        DeclareLaunchArgument("centerline_auto_align_to_map", default_value="true"),
        DeclareLaunchArgument("centerline_offset_x_m", default_value="0.0"),
        DeclareLaunchArgument("centerline_offset_y_m", default_value="0.0"),
        DeclareLaunchArgument("sensor_params_file", default_value=default_params),
        DeclareLaunchArgument("map_path_speed_mps", default_value="0.35"),
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
                    "world_type": world_type,
                    "map_yaml_path": map_yaml_path,
                    "centerline_csv_path": centerline_csv_path,
                    "centerline_auto_align_to_map": centerline_auto_align_to_map,
                    "centerline_offset_x_m": centerline_offset_x_m,
                    "centerline_offset_y_m": centerline_offset_y_m,
                    "lidar_rate_hz": 15.0,
                    "imu_rate_hz": 15.0,
                    "odom_rate_hz": 15.0,
                    "state_rate_hz": 60.0,
                    "scan_range_max": 8.0,
                    "scan_angle_min_deg": -135.0,
                    "scan_angle_max_deg": 135.0,
                    "scan_angle_increment_deg": 2.0,
                    "map_path_speed_mps": map_path_speed_mps,
                    "map_use_pure_pursuit": False,
                    "map_use_waypoint_heading": False,
                    "map_follow_waypoint_yaw": False,
                    "map_min_speed_mps": 0.2,
                    "pp_lookahead_m": 0.9,
                    "pp_lookahead_gain": 0.03,
                    "pp_max_yaw_rate_radps": 2.2,
                    "publish_imu": False,
                    "publish_odom": True,
                    "publish_odom_tf": False,
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                "map_save_dir": map_save_dir,
                "map_file_prefix": map_file_prefix,
                "save_interval_sec": save_interval_sec,
                "save_on_shutdown": "true",
                "export_ros_map": "false",
                "export_ros_map_on_shutdown": "true",
                "use_sim_time": "false",
                "imu_topic": "/unused_imu",
                "odom_topic": "/sim_odom",
                "scan_topic": "/sim_scan",
                "configuration_basename": "cartographer_2d_mapping_lidar_only.lua",
                "enable_sensor_bringup": "false",
            }.items(),
        ),
    ])
