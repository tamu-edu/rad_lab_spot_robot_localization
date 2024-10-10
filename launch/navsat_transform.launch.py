from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'navsat_transform.yaml')],
            remappings=[
                ('/imu/data', '/gx5/imu_with_covariance'),
                ('/gps/fix', '/gx5/gnss1/fix_corrected_frameid'),
                ('/odometry/filtered', '/odometry/filtered'),
            ],
        ),
    ])
