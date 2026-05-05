from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping', 
            executable='slam_gmapping', 
            output='screen', 
            parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")]),launch_ros.actions.Node(
     package='tf2_ros',
     executable='static_transform_publisher',
     name='base_link_to_base_laser',
     arguments=['-0.0046412', '0' , '0.094079','0','0','0','base_link','laser_frame']
    ),launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    ),launch_ros.actions.Node(
     package='tf2_ros',
     executable='static_transform_publisher',
     name='base_link_foot',
     arguments=['-0.0', '0' , '0.0','0','0','0','base_footprint','base_link']),
    ])

