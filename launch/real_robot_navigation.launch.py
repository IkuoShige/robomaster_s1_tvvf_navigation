#!/usr/bin/env python3
"""
RoboMaster S1 Real Robot Navigation Launch File

Launches the complete navigation stack for real robot operation:
- Livox MID-360 LiDAR driver
- Robot state publisher (TF: base_link -> livox_frame)
- S1 driver (TF: odom -> base_link, /odom topic)
- Navigation stack (map_scan_manager, emcl2, obstacle_tracker, tvvf_vo, waypoint_follower)
- Optional: Joystick teleoperation
- Optional: RViz2 visualization

Usage:
  # Basic navigation
  ros2 launch robomaster_s1_tvvf_navigation real_robot_navigation.launch.py

  # With teleoperation enabled
  ros2 launch robomaster_s1_tvvf_navigation real_robot_navigation.launch.py enable_teleop:=true

  # Without RViz (headless)
  ros2 launch robomaster_s1_tvvf_navigation real_robot_navigation.launch.py rviz:=false

Prerequisites:
  1. CAN interface setup: ./src/robomaster_s1_rust/can0_interface_setup.sh
  2. Source workspaces:
     source ~/livox_ws/install/setup.bash
     source ~/robomaster_s1_ws/install/setup.bash
"""

import os
from typing import List

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_robot_description(context: LaunchContext) -> List[Node]:
    """Generate robot_state_publisher node with URDF."""
    # Get URDF file path
    urdf_xacro = os.path.join(
        get_package_share_directory('robomaster_description'),
        'urdf', 'robomaster_s1_real.urdf.xacro'
    )

    # Process xacro to URDF
    doc = xacro.process_file(urdf_xacro)
    robot_description = doc.toxml()

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return [robot_state_publisher]


def generate_launch_description():
    # ========================================
    # Package directories
    # ========================================
    tvvf_nav_pkg = get_package_share_directory('robomaster_s1_tvvf_navigation')
    s1_teleop_pkg = FindPackageShare('s1_teleop')

    # Livox config path (in livox_ws)
    livox_config_path = os.path.expanduser(
        '~/livox_ws/src/livox_ros_driver2/config/MID360_config.json'
    )

    # ========================================
    # Configuration files
    # ========================================
    map_scan_config = os.path.join(tvvf_nav_pkg, 'config', 'map_scan_params.yaml')
    map_regions_config = os.path.join(tvvf_nav_pkg, 'config', 'map_regions.yaml')
    emcl2_config = os.path.join(tvvf_nav_pkg, 'config', 'emcl2_params.yaml')
    obstacle_tracker_config = os.path.join(tvvf_nav_pkg, 'config', 'obstacle_tracker_params.yaml')
    tvvf_vo_config = os.path.join(tvvf_nav_pkg, 'config', 'tvvf_vo_params.yaml')
    waypoint_params = os.path.join(tvvf_nav_pkg, 'config', 'waypoint_follower_params.yaml')
    rviz_config = os.path.join(tvvf_nav_pkg, 'rviz', 'navigation.rviz')
    # emcl2 map file
    default_map_file = os.path.join(tvvf_nav_pkg, 'maps', 'f19.yaml')

    joystick_config = PathJoinSubstitution([
        s1_teleop_pkg, 'config', 'joystick_config.yaml'
    ])

    # ========================================
    # Launch arguments
    # ========================================
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Full path to map yaml file'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    declare_enable_teleop = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Enable joystick teleoperation'
    )

    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can2',
        description='CAN interface for S1 driver'
    )

    declare_joystick_device = DeclareLaunchArgument(
        'joystick_device',
        default_value='0',
        description='Joystick device ID'
    )

    # Get launch configurations
    map_file = LaunchConfiguration('map_file')
    rviz = LaunchConfiguration('rviz')
    enable_teleop = LaunchConfiguration('enable_teleop')
    can_interface = LaunchConfiguration('can_interface')
    joystick_device = LaunchConfiguration('joystick_device')

    # ========================================
    # Core robot nodes
    # ========================================

    # 1. Livox MID-360 driver
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,           # PointCloud2 (PointXYZRTL)
            'multi_topic': 0,           # All LiDARs share same topic
            'data_src': 0,              # LiDAR data source
            'publish_freq': 10.0,       # Publish frequency (Hz)
            'output_data_type': 0,
            'frame_id': 'livox_frame',  # Frame ID (must match URDF)
            'lvx_file_path': '',
            'user_config_path': livox_config_path,
            'cmdline_input_bd_code': ''
        }]
    )

    # 2. Robot state publisher (publishes base_link -> livox_frame TF from URDF)
    robot_state_publisher_action = OpaqueFunction(function=generate_robot_description)

    # 3. S1 driver (publishes odom -> base_link TF and /odom topic)
    s1_driver = Node(
        package='s1_driver',
        executable='s1_driver_node',
        name='s1_driver_node',
        output='screen',
        parameters=[{
            'can_interface': can_interface,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'publish_tf': True,
            'control_frequency': 50.0,
            'cmd_vel_timeout': 1.0
        }]
    )

    # ========================================
    # Optional teleoperation nodes
    # ========================================
    teleop_group = GroupAction(
        condition=IfCondition(enable_teleop),
        actions=[
            # Joy node
            Node(
                package='joy',
                executable='joy_node',
                name='joy',
                parameters=[
                    joystick_config,
                    {'device_id': joystick_device}
                ]
            ),
            # S1 joystick teleop node
            Node(
                package='s1_teleop',
                executable='s1_joystick_teleop',
                name='s1_joystick_teleop',
                parameters=[joystick_config],
                output='screen'
            )
        ]
    )

    # ========================================
    # Navigation stack nodes
    # ========================================
    navigation_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', False),

            # 1. map_scan_manager - 3D LiDAR to 2D LaserScan conversion
            Node(
                package='map_scan_manager',
                executable='map_scan_manager_node',
                name='map_scan_manager',
                output='screen',
                parameters=[
                    map_scan_config,
                    {'map_regions_config_path': map_regions_config}
                ]
            ),

            # 2. simple_map_server - Publish map for localization
            Node(
                package='robomaster_s1_tvvf_navigation',
                executable='simple_map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'map_yaml_path': map_file,
                    'use_sim_time': False
                }]
            ),

            # 3. emcl2 - Monte Carlo Localization
            Node(
                package='emcl2',
                executable='emcl2_node',
                name='emcl2',
                output='screen',
                parameters=[emcl2_config],
                remappings=[
                    ('scan', 'scan/localization')
                ]
            ),

            # 4. obstacle_tracker - Detect and track obstacles
            Node(
                package='obstacle_tracker',
                executable='obstacle_tracker',
                name='obstacle_tracker',
                output='screen',
                parameters=[obstacle_tracker_config]
            ),

            # 5. tvvf_vo_c - Time-Varying Vector Field local planner
            Node(
                package='tvvf_vo_c',
                executable='tvvf_vo_c_node',
                name='tvvf_vo_c_node',
                output='screen',
                parameters=[tvvf_vo_config]
            ),

            # 6. waypoint_follower - Waypoint navigation controller
            Node(
                package='robomaster_s1_tvvf_navigation',
                executable='waypoint_follower_node',
                name='waypoint_follower_node',
                output='screen',
                parameters=[waypoint_params]
            )
        ]
    )

    # ========================================
    # Visualization
    # ========================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz)
    )

    # ========================================
    # Build launch description
    # ========================================
    return LaunchDescription([
        # Declare arguments
        declare_map_file,
        declare_rviz,
        declare_enable_teleop,
        declare_can_interface,
        declare_joystick_device,

        # Core robot nodes
        livox_driver,
        robot_state_publisher_action,
        s1_driver,

        # Optional teleop
        teleop_group,

        # Navigation stack
        navigation_nodes,

        # Visualization
        rviz_node,
    ])
