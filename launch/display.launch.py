# aimm_navigation/launch/display.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('aimm_navigation')
    urdf_path = os.path.join(pkg_share, 'urdf', 'aimm.urdf')

    # read the URDF once
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # 1) static odom → base_link (identity rotation)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_odom_to_base_link',
            arguments=[
                '0', '0', '0',        # x y z
                '0', '0', '0', '1',   # qx qy qz qw
                'odom', 'base_link'
            ],
        ),

        # Add this before robot_state_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        ),

        # 2) robot_state_publisher – ALSO publish /robot_description!
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'publish_robot_description': True},   # <<======= KEY
            ],
            output='screen',
        ),

        # 3) joint_state_publisher (headless), fed the URDF file path
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf_path],                     # loads its own copy
            parameters=[{'publish_default_positions': True}],
            output='screen',
        ),

        # 4) RViz (optional config – change to your file if you have one)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # uncomment the next line and point at your RViz file if desired
            # arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        ),

        







        #  # 5) map_server (to publish the static map)
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[os.path.join(pkg_share, 'config', 'map_server_params.yaml')],
        # ),

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_map',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'autostart': True,
        #         'node_names': ['map_server']
        #     }],
        # ),

    ])
