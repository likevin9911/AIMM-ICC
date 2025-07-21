#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Path to Nav2's built‑in navigation_launch.py
    pkg_nav2nav = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(
        pkg_nav2nav, 'launch', 'navigation_launch.py'
    )

    # 2) Your Nav2 parameter file
    pkg_local = get_package_share_directory('localization_launch')
    nav2_params = os.path.join(pkg_local, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file':                 nav2_params,
                'use_sim_time':                'False',
                'autostart':                   'True',
                'map_subscribe_transient_local':'True',
            }.items()
        )
    ])

