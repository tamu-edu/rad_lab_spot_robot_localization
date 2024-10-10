from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to your rosbag file
    rosbag_path = '/home/annanya/Documents/data/rosbags/all_topics'  # Replace with the path to your rosbag file

    # Declare the path to the rosbag as a launch argument so it can be easily modified if needed
    declare_rosbag_path = DeclareLaunchArgument(
        'rosbag_path',
        default_value=rosbag_path,
        description='Path to the rosbag file to be played'
    )

    # Define the Python node to be launched from the same package
    python_node = launch_ros.actions.Node(
        package='robot_localization',  # Replace with your package name
        executable='topic_republisher.py',  # Use the Python script's executable name without the '.py' extension
        name='sensor_fusion_node',
        output='screen',
        parameters=[]  # Add parameters if required
    )

    # Execute process to play the rosbag
    play_rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('rosbag_path')],
        output='screen'
    )

    # Define the ekf_node to be launched
    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
    )

    return LaunchDescription([
        declare_rosbag_path,
        play_rosbag,
        python_node,
        ekf_node,
    ])
