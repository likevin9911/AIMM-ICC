#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('aimm_navigation')

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(pkg, 'maps', 'E.yaml')}]
    )

    amcl = Node(
        package='nav2_amcl', executable='amcl',
        name='amcl', output='screen',
        parameters=[os.path.join(pkg, 'config', 'amcl.yaml')]
    )

    lifecycle_mgr = Node(
  package='nav2_lifecycle_manager',
  executable='lifecycle_manager',
  name='lifecycle_manager',
  output='screen',
  emulate_tty=True,
  parameters=[
    {'use_sim_time': False},
    {'autostart': True},
    {'node_names': ['map_server', 'amcl']},  # ← include amcl here
  ]
)


    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_mgr,

    ])
