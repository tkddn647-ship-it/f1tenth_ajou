import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Single source of truth: use sim_test/launch/sim_mapping.launch.py
    # for fake simulation mapping.
    sim_pkg_dir = get_package_share_directory("sim_test")
    sim_mapping_launch = os.path.join(sim_pkg_dir, "launch", "sim_mapping.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_mapping_launch),
        ),
    ])
