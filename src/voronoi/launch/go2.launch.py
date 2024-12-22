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

    voronoi_pkg = get_package_share_directory('voronoi')
    merge_map_pkg = get_package_share_directory('merge_map')
    # nav2_pkg = get_package_share_directory('nav2_bringup')  # Ensure Nav2 is installed

    robot_desc_path = os.path.join(voronoi_pkg, "models", "go2", "go2.urdf")

    # Merge Map Command
    merge_map_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(merge_map_pkg, 'launch', 'merge_map_go2_launch.py')
        ),
        # Pass the robot_count as a substitution, not as a resolved string
        launch_arguments={'robot_count': LaunchConfiguration('robot_count')}.items(),
    )

    nodes = []

    for i in range(robot_count):
        name = f'robot{i}'
        urdf_path = os.path.join(voronoi_pkg, 'models', 'go2', 'go2.urdf')

        # Read and modify URDF for the robot
        with open(robot_desc_path, 'r') as infp:
            robot_desc = infp.read()
        robot_desc = add_prefix_to_urdf(robot_desc, f'{name}/')

        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_go2',
            namespace=name,
            output='screen',
            parameters=[{
                'use_sim_time': False,
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
                'use_sim_time': False,
                'odom_frame': f'{name}/odom',
                'base_frame': f'{name}/base_link',
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

        # Include Nav2 for the robot
        # nav2_launch = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')  # Adjust path if different
        #     ),
        #     launch_arguments={
        #         'namespace': name,
        #         'use_namespace': 'True',
        #         'slam': 'True',  # Assuming SLAM is handled by slam_toolbox
        #         'map': '',  # Ensure map is correctly set or passed
        #         'use_sim_time': 'True',
        #         'autostart': 'True',
        #         'params_file': os.path.join(voronoi_pkg, 'params',  f'{name}_nav2_params.yaml'),  # Create per-robot params
        #     }.items(),
        # )

        # Add nodes to the list
        nodes.extend([
            # spawn_robot,
            robot_state_publisher,
            async_slam_toolbox,
            static_transform_publisher,
            # nav2_launch,  # Add Nav2 launch
            # rviz  # Remove or comment out if individual RViz instances are not needed
        ])

    # Create a list to hold all launch actions
    launch_actions = [
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
