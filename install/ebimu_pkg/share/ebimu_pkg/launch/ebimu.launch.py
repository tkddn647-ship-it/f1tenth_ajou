#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription([

        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="EBIMU 시리얼 포트 (/dev/ttyUSB0 등)",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="EBIMU 시리얼 통신 속도 (115200 등)",
        ),

        Node(
            package="ebimu_pkg",
            executable="ebimu_driver",
            name="ebimu_driver",
            parameters=[{
                "port": port,
                "baud": baud,
            }],
            output="screen",
        ),
    ])

