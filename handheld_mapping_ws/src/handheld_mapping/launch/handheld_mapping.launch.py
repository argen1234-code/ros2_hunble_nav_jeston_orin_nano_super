#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    lidar_model = LaunchConfiguration('lidar_model')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    lidar_port = LaunchConfiguration('lidar_port')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization')

    declare_lidar_model = DeclareLaunchArgument(
        'lidar_model', default_value='X4',
        description='YDLidar model: X4 (128000bps), X2 (115200), '
                    'G1/G2/TminiPro (230400), TG (512000). '
                    'Only used when lidar_params_file is empty')

    declare_lidar_params = DeclareLaunchArgument(
        'lidar_params_file', default_value='',
        description='Path to YDLidar parameter file — overrides lidar_model if set')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('handheld_mapping'), 'params', 'slam_gmapping.yaml'
        ]),
        description='Path to slam_gmapping parameter file')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='LiDAR serial port')

    # ── Parameter file substitution ──────────────────────────────────
    # Map model name to params file (only one key can be active at once)
    lidar_params_effective = PathJoinSubstitution([
        FindPackageShare('handheld_mapping'), 'params',
        ['ydlidar_', lidar_model, '.yaml']])

    # ── LiDAR driver node ─────────────────────────────────────────────
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[lidar_params_effective, {'port': lidar_port}],
        namespace='/',
    )

    # ── TF: base_link → laser_frame ──────────────────────────────────
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1',
                   'base_link', 'laser_frame'],
    )

    # ── TF: base_footprint → base_link ────────────────────────────────
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'base_footprint', 'base_link'],
    )

    # ── Laser odometry node ──────────────────────────────────────────
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

    # ── SLAM gmapping node ───────────────────────────────────────────
    slam_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[slam_params_file],
    )

    # ── Map saver node ───────────────────────────────────────────────
    map_saver_node = Node(
        package='handheld_mapping',
        executable='map_saver',
        name='map_saver',
        output='screen',
    )

    # ── RViz2 ────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('handheld_mapping'), 'config', 'handheld_mapping.rviz'
        ])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_lidar_model,
        declare_lidar_params,
        declare_lidar_port,
        declare_slam_params,
        ydlidar_node,
        tf_base_laser,
        tf_footprint_base,
        laser_odom_node,
        slam_node,
        map_saver_node,
        rviz_node,
    ])
