#!/usr/bin/python3
"""
Online SLAM Navigation — indoor Nav2 plus direct GPS/LiDAR outdoor control.

No saved map needed. No AMCL. Gmapping provides both the /map topic and
the map→odom TF transform. Nav2 is reserved for indoor mode; GPS+ROS mode
directly combines GPS heading and LaserScan without path planning.

Usage:
  ros2 launch handheld_mapping handheld_nav.launch.py

In RViz:
  1. "2D Pose Estimate" to set initial pose on gmapping (not needed, gmapping starts at origin)
  2. "Nav2 Goal" to set navigation goal
  3. Watch /plan path and /cmd_vel output in terminal
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import fcntl


_INSTANCE_LOCK = None


def _acquire_instance_lock(_context):
    """Prevent two copies of the handheld stack from running concurrently."""
    global _INSTANCE_LOCK
    path = '/tmp/handheld_mapping_nav.lock'
    lock = open(path, 'a+')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock.seek(0)
        owner = lock.read().strip()
        lock.close()
        owner_hint = f' (PID {owner})' if owner else ''
        raise RuntimeError(
            f'handheld_mapping is already running{owner_hint}; stop the '
            'existing launch before starting another copy') from exc
    lock.seek(0)
    lock.truncate()
    lock.write(str(os.getpid()))
    lock.flush()
    _INSTANCE_LOCK = lock
    return []


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────
    lidar_model = LaunchConfiguration('lidar_model')
    lidar_port = LaunchConfiguration('lidar_port')
    stm32_port = LaunchConfiguration('stm32_port')
    stm32_baud = LaunchConfiguration('stm32_baud')
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
    bt_xml_file = PathJoinSubstitution([
        FindPackageShare('handheld_mapping'), 'params',
        'navigate_to_pose_w_replanning_and_recovery.xml'])

    # ── 1. LiDAR driver ──────────────────────────────────────────────
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
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
        # Smooth the raw controller output before it reaches the chassis.
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel'),
        ],
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
        parameters=[nav2_params_file, {
            'default_nav_to_pose_bt_xml': bt_xml_file,
            'default_nav_through_poses_bt_xml': bt_xml_file,
        }],
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

    # ── 6. Direct GPS heading + LiDAR reactive avoidance ──────────────
    virtual_goal_node = Node(
        package='handheld_mapping',
        executable='virtual_goal_publisher',
        name='virtual_goal_publisher',
        output='screen',
        parameters=[{
            'control_hz': 10.0,
            'arrival_radius': 1.0,
            'waypoint_dwell': 5.0,
            'sensor_timeout': 0.75,
            'scan_timeout': 0.60,
            # Fusion uses GPS waypoint/heading sources, but chassis direction
            # and LiDAR avoidance match the verified indoor (LiDAR-front) mode.
            'heading_offset_deg': 180.0,
            'cruise_speed': 0.24,
            'minimum_drive_speed': 0.08,
            'max_angular_speed': 0.78,
            'turn_in_place_angle_deg': 40.0,
            'front_stop_distance': 0.34,
            'front_slow_distance': 0.68,
            'side_stop_distance': 0.30,
            'obstacle_turn_speed': 1.36,
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
            'gps_ros_linear_sign': -1.0,
            'gps_ros_angular_sign': -1.0,
        }],
    )

    # ── 8. MQTT cloud bridge ────────────────────────────────────────
    mqtt_bridge_node = Node(
        package='handheld_mapping',
        executable='mqtt_bridge',
        name='mqtt_bridge',
        output='screen',
    )

    indoor_map_bridge_node = Node(
        package='handheld_mapping',
        executable='indoor_map_bridge',
        name='indoor_map_bridge',
        output='screen',
    )

    indoor_mission_manager_node = Node(
        package='handheld_mapping',
        executable='indoor_mission_manager',
        name='indoor_mission_manager',
        output='screen',
        parameters=[{
            'control_hz': 10.0,
            'goal_radius': 0.30,
            'goal_yaw_tolerance': 0.25,
            'position_kp': 1.10,
            'position_ki': 0.02,
            'position_kd': 0.06,
            'max_linear_speed': 0.24,
            'angle_kp': 2.20,
            'angle_ki': 0.04,
            'angle_kd': 0.28,
            'max_angular_speed': 0.78,
            'final_turn_speed_scale': 1.5,
            'turn_in_place_angle': 0.70,
            'obstacle_turn_speed': 0.68,
            'obstacle_turn_speed_scale': 2.0,
            'front_stop_distance': 0.34,
            'front_slow_distance': 0.68,
            'scan_timeout': 0.80,
        }],
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
        declare_use_rviz,
        # Execute only when a launch service actually starts. Keeping this out
        # of generate_launch_description() lets `ros2 launch --show-args` and
        # other launch-file introspection work while the stack is running.
        OpaqueFunction(function=_acquire_instance_lock),
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
        velocity_smoother_node,
        lifecycle_nav_node,
        virtual_goal_node,
        stm32_bridge_node,
        mqtt_bridge_node,
        indoor_map_bridge_node,
        indoor_mission_manager_node,
        map_saver_node,
        cmd_vel_monitor,
        rviz_node,
    ])
