from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'ai_bot'
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf.yaml' # your config file name.
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[config_file]
    )

    return LaunchDescription([
        ekf_node,
    ])

