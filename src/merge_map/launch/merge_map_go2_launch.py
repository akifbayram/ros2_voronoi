import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_count',
            default_value='1',
            description='Number of robots in the system'
        ),
        Node(
            package='merge_map',
            executable='merge_map_go2',
            output='screen',
            parameters=[{'robot_count': LaunchConfiguration('robot_count'), 'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('merge_map'), 'config', 'merge_map.rviz')],
            parameters=[{'use_sim_time': False}]
        ),
    ])
