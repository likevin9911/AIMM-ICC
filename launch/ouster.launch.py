#!/usr/bin/env python3
import os
from pathlib import Path
import launch
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1) Paths and URDF load
    # -------------------------------------------------------------------------
    pkg_share = get_package_share_directory('aimm_navigation')
    urdf_path = os.path.join(pkg_share, 'urdf', 'aimm.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # -------------------------------------------------------------------------
    # 2) Ouster driver parameter file and args
    # -------------------------------------------------------------------------
    ouster_pkg = get_package_share_directory('ouster_ros')
    default_params = Path(ouster_pkg) / 'config' / 'driver_params.yaml'
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=str(default_params),
        description='Path to driver_params.yaml'
    )
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns', default_value='ouster',
        description='ROS namespace for Ouster'
    )
    viz_arg = DeclareLaunchArgument(
        'viz', default_value='False',
        description='Whether to launch Ouster RViz'
    )
    os_driver_name_arg = DeclareLaunchArgument(
        'os_driver_name', default_value='os_driver',
        description='Name of the lifecycle driver node'
    )

    # -------------------------------------------------------------------------
    # 3) Static transforms: map->odom and odom->base_link
    # -------------------------------------------------------------------------
    static_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )
    static_odom_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    # -------------------------------------------------------------------------
    # 4) robot_state_publisher & joint_state_publisher
    # -------------------------------------------------------------------------
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'publish_robot_description': True},
        ],
        output='screen'
    )
    jsp = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        parameters=[{'publish_default_positions': True}],
        output='screen'
    )
    static_base_to_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_to_lidar_link',
        arguments=[
            '0', '0', '0.4',    # approximate z-offset from base_link to lidar_link
            '0', '0', '0', '1',  # qx qy qz qw
            'base_link', 'lidar_link'
        ]
    )
    # -------------------------------------------------------------------------
    # 5) RViz2 for full robot+Ouster visualization
    # -------------------------------------------------------------------------
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
    )

    # -------------------------------------------------------------------------
    # 6) Ouster LifecycleNode (driver)
    #    override frames to match URDF's lidar_link
    # -------------------------------------------------------------------------
    os_driver = LifecycleNode(
        package='ouster_ros', executable='os_driver',
        name=LaunchConfiguration('os_driver_name'),
        namespace=LaunchConfiguration('ouster_ns'),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                #'sensor_frame':       'base_link',
                'lidar_frame': 'lidar_link',
                'point_cloud_frame': 'lidar_link',
                'pub_static_tf': False,
                'use_system_default_qos': True, 
            }
        ],
        output='screen'
    )
    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )
    sensor_finalized_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='finalized',
            entities=[
                EmitEvent(event=launch.events.Shutdown(
                    reason="Couldn't communicate with sensor"
                ))
            ],
        )
    )

    # -------------------------------------------------------------------------
    # 7) (Optional) include the package's own rviz.launch.py, if desired
    # -------------------------------------------------------------------------
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [str(Path(ouster_pkg) / 'launch' / 'rviz.launch.py')]
        ),
        condition=IfCondition(LaunchConfiguration('viz'))
    )

    # -------------------------------------------------------------------------
    # 8) Assemble all nodes/handlers
    # -------------------------------------------------------------------------
    return LaunchDescription([
        # Ouster args
        params_file_arg,
        ouster_ns_arg,
        viz_arg,
        os_driver_name_arg,

        # TF
        static_map_to_odom,
        static_odom_to_base,
        static_base_to_lidar,

        # Robot state & joints
        rsp,
        jsp,

        # RViz
        rviz,

        # Ouster driver + lifecycle events
        os_driver,
        sensor_configure_event,
        sensor_activate_event,
        sensor_finalized_event,

        # Optional include of original rviz.launch.py
        rviz_launch,
    ])
