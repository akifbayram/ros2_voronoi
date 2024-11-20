# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Robot names
    name1 = "tb3_0"
    name2 = "tb3_1"

    # Get the new model paths
    urdf_path1 = os.path.join(get_package_share_directory('voronoi'), 'models', name1, 'model.sdf')
    urdf_path2 = os.path.join(get_package_share_directory('voronoi'), 'models', name2, 'model.sdf')

    # Launch configuration variables specific to simulation
    x_pose1 = LaunchConfiguration('x_pose1', default='0.0')
    y_pose1 = LaunchConfiguration('y_pose1', default='0.0')
    x_pose2 = LaunchConfiguration('x_pose2', default='2.0')
    y_pose2 = LaunchConfiguration('y_pose2', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd1 = DeclareLaunchArgument(
        'x_pose1', default_value='0.0',
        description='Specify x position of robot 1')

    declare_y_position_cmd1 = DeclareLaunchArgument(
        'y_pose1', default_value='0.0',
        description='Specify y position of robot 1')

    declare_x_position_cmd2 = DeclareLaunchArgument(
        'x_pose2', default_value='2.0',
        description='Specify x position of robot 2')

    declare_y_position_cmd2 = DeclareLaunchArgument(
        'y_pose2', default_value='0.0',
        description='Specify y position of robot 2')

    # Node to spawn robot 1
    start_gazebo_ros_spawner_cmd1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', name1,
            '-file', urdf_path1,
            '-x', x_pose1,
            '-y', y_pose1,
            '-z', '0.01'
        ],
        output='screen',
    )

    # Node to spawn robot 2
    start_gazebo_ros_spawner_cmd2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', name2,
            '-file', urdf_path2,
            '-x', x_pose2,
            '-y', y_pose2,
            '-z', '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd1)
    ld.add_action(declare_y_position_cmd1)
    ld.add_action(declare_x_position_cmd2)
    ld.add_action(declare_y_position_cmd2)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd1)
    ld.add_action(start_gazebo_ros_spawner_cmd2)

    return ld
