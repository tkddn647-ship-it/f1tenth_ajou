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
    mapping_launch = os.path.join(pkg_dir, "launch", "cartographer_mapping_launch.py")

    map_save_dir = LaunchConfiguration("map_save_dir")
    map_file_prefix = LaunchConfiguration("map_file_prefix")
    save_interval_sec = LaunchConfiguration("save_interval_sec")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_save_dir",
            default_value="/home/tkddn647/test/maps",
            description="Directory to store exported map files",
        ),
        DeclareLaunchArgument(
            "map_file_prefix",
            default_value="fake_hwless_map",
            description="Prefix for exported map files",
        ),
        DeclareLaunchArgument(
            "save_interval_sec",
            default_value="10.0",
            description="Periodic export interval in seconds",
        ),
        Node(
            package="tf_manager_cpp",
            executable="sensor_static_tf",
            name="sensor_static_tf_node",
            output="screen",
        ),
        Node(
            package="localization_layer",
            executable="fake_sensor_publisher.py",
            name="fake_sensor_publisher",
            output="screen",
            parameters=[{
                "rate_hz": 10.0,
                "publish_static_tf": False,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                "map_save_dir": map_save_dir,
                "map_file_prefix": map_file_prefix,
                "save_interval_sec": save_interval_sec,
                "save_on_shutdown": "false",
                "use_sim_time": "false",
            }.items(),
        ),
    ])
