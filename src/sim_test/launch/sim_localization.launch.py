import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg_dir = get_package_share_directory("sim_test")
    loc_pkg_dir = get_package_share_directory("localization_layer")
    localization_launch = os.path.join(loc_pkg_dir, "launch", "cartographer_localization_launch.py")
    default_params = os.path.join(sim_pkg_dir, "config", "sim_sensor_params_localization.yaml")

    pbstream_filename = LaunchConfiguration("pbstream_filename")
    world_type = LaunchConfiguration("world_type")
    map_yaml_path = LaunchConfiguration("map_yaml_path")
    centerline_csv_path = LaunchConfiguration("centerline_csv_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sensor_params_file = LaunchConfiguration("sensor_params_file")
    map_path_speed_mps = LaunchConfiguration("map_path_speed_mps")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pbstream_filename",
            description="Absolute path to .pbstream map file used for pure localization",
        ),
        DeclareLaunchArgument("world_type", default_value="map"),
        DeclareLaunchArgument("map_yaml_path", default_value="/home/tkddn647/test/maps/example_map.yaml"),
        # Keep empty by default so fake sensor path is extracted from the same map.
        # Using an unrelated CSV often causes scan/map mismatch and bad localization.
        DeclareLaunchArgument("centerline_csv_path", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("sensor_params_file", default_value=default_params),
        # 70.0 was interpreted as km/h by sim_fake_sensor_publisher (-> 19.4 m/s), too fast for stable scan matching.
        DeclareLaunchArgument("map_path_speed_mps", default_value="8.0"),
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
            parameters=[
                sensor_params_file,
                {
                    "world_type": world_type,
                    "map_yaml_path": map_yaml_path,
                    "centerline_csv_path": centerline_csv_path,
                    "map_path_speed_mps": map_path_speed_mps,
                },
            ],
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
