import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('nmpc_tracker')
    rviz_config = os.path.join(pkg_dir, 'config', 'view_bot.rviz')

    return LaunchDescription([
        Node(
            package='nmpc_tracker',
            executable='simulator',
            name='sim_node'
        ),
        Node(
            package='nmpc_tracker',
            executable='controller',
            name='nmpc_node'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
