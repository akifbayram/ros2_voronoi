import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def add_prefix_to_urdf(urdf_text, prefix):
    # Add prefix to link and joint names in the URDF
    urdf_text = re.sub(r'(<link\s+name=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
    urdf_text = re.sub(r'(<joint\s+name=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
    urdf_text = re.sub(r'(<parent\s+link=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
    urdf_text = re.sub(r'(<child\s+link=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
    return urdf_text

def launch_setup(context, *args, **kwargs):
    # Retrieve the robot_count value
    robot_count_str = LaunchConfiguration('robot_count').perform(context)
    try:
        robot_count = int(robot_count_str)
    except ValueError:
        raise ValueError(f"robot_count must be an integer, got '{robot_count_str}'")

    if robot_count < 1:
        raise ValueError("robot_count must be at least 1")

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    voronoi_pkg = get_package_share_directory('voronoi')
    merge_map_pkg = get_package_share_directory('merge_map')

    world = os.path.join(voronoi_pkg, 'worlds', 'simple_env_1.world')
    robot_desc_path = os.path.join(turtlebot3_gazebo_pkg, "urdf", "turtlebot3_burger.urdf")

    # Gazebo Server and Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # Merge Map Command
    merge_map_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(merge_map_pkg, 'launch', 'merge_map_launch.py')
        ),
        # Pass the robot_count as a substitution, not as a resolved string
        launch_arguments={'robot_count': LaunchConfiguration('robot_count')}.items(),
    )

    nodes = []

    for i in range(robot_count):
        name = f'tb3_{i}'
        urdf_path = os.path.join(voronoi_pkg, 'models', name, 'model.sdf')

        # Read and modify URDF for the robot
        with open(robot_desc_path, 'r') as infp:
            robot_desc = infp.read()
        robot_desc = add_prefix_to_urdf(robot_desc, f'{name}/')

        # Spawn Robot
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', name,
                '-file', urdf_path,
                '-x', str(-2.75 + i * 0.5),  # Stagger spawn positions on x-axis
                '-y', str(-2.0 + i * 0.5),   # Stagger spawn positions on y-axis
                '-z', '0.01',
                '-robot_namespace', name,
            ],
            output='screen',
        )

        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
            }],
        )

        # SLAM Toolbox
        async_slam_toolbox = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='async_slam_toolbox_node',
            namespace=name,
            parameters=[{
                'use_sim_time': True,
                'odom_frame': f'{name}/odom',
                'base_frame': f'{name}/base_footprint',
                'scan_topic': 'scan',
                'map_frame': f'{name}/map',
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.3,
                'map_update_interval': 1.0,
                'publish_period': 1.0,
                'resolution': 0.05,
            }],
            remappings=[
                ("/map", "map"),
                ("/map_metadata", "map_metadata"),
                ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
                ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
            ],
            output='screen',
        )

        # Static Transform Publisher
        static_transform_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_{name}',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'{name}/map'],
            output='screen',
        )

        # RViz for each robot (optional, can be customized or unified)
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name=f'rviz_{name}',
            namespace=name,
            output='screen',
            arguments=['-d', os.path.join(voronoi_pkg, 'rviz', f'{name}.rviz')],
            parameters=[{'use_sim_time': True}],
        )

        # Add nodes to the list
        nodes.extend([
            spawn_robot,
            robot_state_publisher,
            async_slam_toolbox,
            static_transform_publisher,
            rviz  # Remove or comment out if individual RViz instances are not needed
        ])

    # Create a list to hold all launch actions
    launch_actions = [
        gzserver_cmd,
        gzclient_cmd,
        merge_map_cmd,
    ] + nodes

    return launch_actions

def generate_launch_description():
    return LaunchDescription([
        # Declare the robot_count launch argument
        DeclareLaunchArgument(
            'robot_count',
            default_value='2',
            description='Number of robots in the simulation'
        ),
        # Use OpaqueFunction to dynamically generate nodes based on robot_count
        OpaqueFunction(function=launch_setup)
    ])
