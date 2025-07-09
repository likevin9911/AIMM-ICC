from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg = FindPackageShare('aimm_navigation').find('aimm_navigation')
    default_model = os.path.join(pkg, 'urdf', 'aimm.urdf')
    default_rviz  = os.path.join(pkg, 'rviz', 'aimm.rviz')

    return LaunchDescription([
      DeclareLaunchArgument('gui',       default_value='true'),
      DeclareLaunchArgument('model',     default_value=default_model),
      DeclareLaunchArgument('rvizconfig',default_value=default_rviz),

      Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
      ),
      Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
      ),
      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(LaunchConfiguration('model').perform({})).read()}]
      ),
      Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
      ),
    ])
