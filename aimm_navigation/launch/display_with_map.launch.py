import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('aimm_navigation')
    urdf_path = os.path.join(pkg_share, 'urdf', 'aimm.urdf')
    # rviz_config = os.path.join(pkg_share, 'rviz', 'aimm.rviz')
    map_yaml = os.path.join(pkg_share, 'maps', 'smalltown_world.yaml')

    # Read robot description
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # 1) static map -> odom
        Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
            output='screen',
        ),

        # # 2) static odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_link',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'odom', 'base_link'
            ],
            output='screen',
        ),

        # 3) robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 4) joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf_path],
            parameters=[{'publish_default_positions': True}],
            output='screen',
        ),

    TimerAction(
            period=10.0,  # give RViz 2s to spin up & subscribe
            actions=[
                LifecycleNode(
                    package='nav2_map_server',
                    executable='map_server',
                    namespace='',
                    name='map_server',
                    output='screen',
                    parameters=[
                        {'use_sim_time': False},
                        {'yaml_filename': map_yaml}
                    ],
                ),

                # 6) Lifecycle manager to bring map_server up
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}
                    ],
                ),

                # # 7) Map saver service
                # Node(
                #     package='nav2_map_server',
                #     executable='map_saver_server',
                #     name='map_saver_server',
                #     output='screen',
                #     parameters=[{'use_sim_time': False}],
                # ),

                # 8) RViz2 with custom config
            ]
        ),
  
    ])

