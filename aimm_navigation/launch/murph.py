from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get config directory
    pkg_share = get_package_share_directory('aimm_navigation')

    # Path to EKF config
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Path to NavSat config
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')

    return LaunchDescription([
        # Dummy odometry publisher node (simulated data)
        Node(
            package='dummy_odom',
            executable='dummy_odom_node.py',
            name='dummy_odom_node',
            output='screen',
            remappings=[
                ('odom0', '/odometry/filtered')
            ]
        ),

        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            remappings=[
                ('imu', '/mavros/imu/data'),
                ('gps/fix', '/mavros/global_position/global'),
                ('odometry/filtered', '/odometry/filtered')
            ],
            parameters=[ekf_config]
        ),

        # NavSat Transform node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            remappings=[
                ('imu', '/mavros/imu/data'),
                ('gps/fix', '/mavros/global_position/global'),
                ('odometry/filtered', '/odometry/filtered')
            ],
            parameters=[navsat_config]
        ),
    ])