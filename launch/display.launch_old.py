from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # <install prefix>/share/urdf_launch/launch/display.launch.py
                PathJoinSubstitution([
                    FindPackageShare('aimm_navigation'),
                    'launch',
                    'display.launch.py'
                ])
            ),
            launch_arguments={
                # These keys must exactly match the DeclareLaunchArgument names
                'urdf_package':      TextSubstitution(text='aimm_navigation'),
                'urdf_package_path': TextSubstitution(text='urdf/aimm.urdf'),
                'rviz_config':       TextSubstitution(text='rviz/urdf.rviz'),
                'jsp_gui':           TextSubstitution(text='false'),
            }.items()
        )
    ])
