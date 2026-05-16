#!/usr/bin/python3
"""
Online SLAM Navigation — uses gmapping for real-time localization + Nav2 for path planning.

No saved map needed. No AMCL. Gmapping provides both the /map topic and
the map→odom TF transform. Nav2 planner + DWB controller generate /cmd_vel.

Usage:
  ros2 launch handheld_mapping handheld_nav.launch.py

In RViz:
  1. "2D Pose Estimate" to set initial pose on gmapping (not needed, gmapping starts at origin)
  2. "Nav2 Goal" to set navigation goal
  3. Watch /plan path and /cmd_vel output in terminal
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────
    lidar_model = LaunchConfiguration('lidar_model')
    lidar_port = LaunchConfiguration('lidar_port')
    stm32_port = LaunchConfiguration('stm32_port')
    stm32_baud = LaunchConfiguration('stm32_baud')
    lookahead = LaunchConfiguration('lookahead')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_lidar_model = DeclareLaunchArgument(
        'lidar_model', default_value='TminiPro',
        description='YDLidar model: X4, X2, G1, TminiPro, TG')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='LiDAR serial port')

    declare_stm32_port = DeclareLaunchArgument(
        'stm32_port',
        default_value=Command([
            'python3 ',
            PathJoinSubstitution([
                FindPackageShare('handheld_mapping'), 'scripts', 'find_stm32_port.py'])]),
        description='STM32 virtual COM port (auto-detected)')

    declare_stm32_baud = DeclareLaunchArgument(
        'stm32_baud', default_value='115200',
        description='STM32 serial baudrate')

    declare_lookahead = DeclareLaunchArgument(
        'lookahead', default_value='2.0',
        description='Virtual goal lookahead distance (metres)')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2')

    # ── Paths ─────────────────────────────────────────────────────────
    lidar_params_file = PathJoinSubstitution([
        FindPackageShare('handheld_mapping'), 'params',
        ['ydlidar_', lidar_model, '.yaml']])

    slam_params_file = PathJoinSubstitution([
        FindPackageShare('handheld_mapping'), 'params', 'slam_gmapping.yaml'])

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('handheld_mapping'), 'params', 'nav2_params.yaml'])

    # ── 1. LiDAR driver ──────────────────────────────────────────────
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[lidar_params_file, {'port': lidar_port}],
    )

    # ── 2. TF transforms ─────────────────────────────────────────────
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1',
                   'base_link', 'laser_frame'],
    )

    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'base_footprint', 'base_link'],
    )

    # ── 3. Laser odometry (rf2o) ────────────────────────────────────
    laser_odom_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    # ── 4. SLAM gmapping (map + map→odom TF) ────────────────────────
    slam_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[slam_params_file],
    )

    # ── 5. Nav2 navigation stack ────────────────────────────────────
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
    )

    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
    )

    lifecycle_nav_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ── 6. Virtual goal publisher (continuous carrot) ──────────────────
    virtual_goal_node = Node(
        package='handheld_mapping',
        executable='virtual_goal_publisher',
        name='virtual_goal_publisher',
        output='screen',
        parameters=[{
            'lookahead_distance': lookahead,
        }],
    )

    # ── 7. STM32 bidirectional bridge (cmd_vel + mode → STM32, heading ← STM32) ──
    stm32_bridge_node = Node(
        package='handheld_mapping',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
        parameters=[{
            'port': stm32_port,
            'baudrate': stm32_baud,
        }],
    )

    # ── 8. MQTT cloud bridge ────────────────────────────────────────
    mqtt_bridge_node = Node(
        package='handheld_mapping',
        executable='mqtt_bridge',
        name='mqtt_bridge',
        output='screen',
    )

    # ── 9. Map saver ────────────────────────────────────────────────
    map_saver_node = Node(
        package='handheld_mapping',
        executable='map_saver',
        name='map_saver',
        output='screen',
    )

    # ── 10. cmd_vel monitor (debug) ─────────────────────────────────
    cmd_vel_monitor = Node(
        package='handheld_mapping',
        executable='cmd_vel_monitor',
        name='cmd_vel_monitor',
        output='screen',
    )

    # ── 11. RViz2 ───────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('handheld_mapping'), 'config', 'handheld_nav.rviz'
        ])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_lidar_model,
        declare_lidar_port,
        declare_stm32_port,
        declare_stm32_baud,
        declare_lookahead,
        declare_use_rviz,
        ydlidar_node,
        tf_base_laser,
        tf_footprint_base,
        laser_odom_node,
        slam_node,
        planner_node,
        controller_node,
        behavior_node,
        bt_navigator_node,
        waypoint_node,
        lifecycle_nav_node,
        virtual_goal_node,
        stm32_bridge_node,
        mqtt_bridge_node,
        map_saver_node,
        cmd_vel_monitor,
        rviz_node,
    ])
