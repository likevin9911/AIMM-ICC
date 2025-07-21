from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('aimm_navigation')
    ekf_dir = os.path.join(pkg_dir, 'config')
    ekf_config = os.path.join(ekf_dir, 'dual_ekf.yaml')

    os.environ['FILE_PATH'] = str(ekf_dir)

    return LaunchDescription([
        DeclareLaunchArgument('output_final_position', default_value='true'),
        DeclareLaunchArgument('output_location',       default_value='~/dual_ekf_navsat_debug.txt'),

	# 1) IMU-ONLY EKF -> publishes to /odometry/local
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odometry/local')]           
           ),

    # 2) GPS + IMU EKF -> publishes to /odometry/global
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odometry/global')]
           ),           

    # 3) navsat_transform -> subscribes to MAVROS, republish /odometry/gps & /gps/filtered
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform_node',
	        output='screen',
            parameters=[ekf_config],
            remappings=[('imu', '/mavros/imu/data'),
                        ('gps/fix', '/mavros/global_position/raw/fix'), 
                        ('gps/filtered', '/gps/filtered'),
                        ('odometry/gps', '/odometry/gps'),
                        ('odometry/filtered', '/odometry/global')]           

           )           
])

