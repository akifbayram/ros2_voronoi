import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name1 = "tb3_0"
    name2 = "tb3_1"
    
    merge_map_launch_path = os.path.join(get_package_share_directory('merge_map'), 'launch', 'merge_map_launch.py')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "urdf", "turtlebot3_burger.urdf")
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_house.world')
    world = os.path.join(get_package_share_directory('voronoi'), 'worlds', 'simple_env_1.world')
    urdf_path1 = os.path.join(get_package_share_directory('voronoi'), 'models', name1, 'model.sdf')
    urdf_path2 = os.path.join(get_package_share_directory('voronoi'), 'models', name2, 'model.sdf')

    def add_prefix_to_urdf(urdf_text, prefix):
        import re
        # Add prefix to link names
        urdf_text = re.sub(r'(<link\s+name=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
        # Add prefix to joint names
        urdf_text = re.sub(r'(<joint\s+name=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
        # Add prefix to parent and child in joints
        urdf_text = re.sub(r'(<parent\s+link=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
        urdf_text = re.sub(r'(<child\s+link=\")([^\"]+)\"', r'\1' + prefix + r'\2"', urdf_text)
        return urdf_text

    # Read and modify URDF for Robot 1
    with open(robot_desc_path, 'r') as infp:
        robot_desc1 = infp.read()
    robot_desc1 = add_prefix_to_urdf(robot_desc1, name1 + '/')

    # Read and modify URDF for Robot 2
    with open(robot_desc_path, 'r') as infp:
        robot_desc2 = infp.read()
    robot_desc2 = add_prefix_to_urdf(robot_desc2, name2 + '/')

    # 1. ROBOT 1 LAUNCH NODES
    spawn_robot1 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name1, 
            '-file', urdf_path1, 
            '-x', '-2.5', 
            '-y', '-2', 
            '-z', '0.01',
            '-robot_namespace', name1,
        ],
        output='screen'
    )

    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name1,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc1
        }]
    )

    async_slam_toolbox1 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
            'odom_frame': name1 + '/odom',
            'base_frame': name1 + '/base_footprint',
            'scan_topic': 'scan',
            'map_frame': name1 + '/map',
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
    # nav2_1 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'autostart': 'true',
    #         'use_namespace': 'True',
    #         'namespace': name1,
    #         'slam': 'True',
    #         'map': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'maps', 'turtlebot_world.yaml'),
    #         'params_file': os.path.join(get_package_share_directory('voronoi'), 'params', 'nav2_multirobot_params_1.yaml')
    #     }.items()
    # )
    rviz1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('voronoi'), 'rviz', name1 + '.rviz')]
    )
    
    # 2. ROBOT 2 LAUNCH NODES
    spawn_robot2 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name2, 
            '-file', urdf_path2, 
            '-x', '-2.5', 
            '-y', '-1', 
            '-z', '0.01',
            '-robot_namespace', name2,
        ],
        output='screen'
    )

    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name2,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc2
        }]
    )
    
    async_slam_toolbox2 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
            'odom_frame': name2 + '/odom',
            'base_frame': name2 + '/base_footprint',
            'scan_topic': 'scan',
            'map_frame': name2 + '/map',
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
    # nav2_2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'autostart': 'true',
    #         'use_namespace': 'True',
    #         'namespace': name2,
    #         'slam': 'True',
    #         'map': os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'maps', 'turtlebot_world.yaml'),
    #         'params_file': os.path.join(get_package_share_directory('voronoi'), 'params', 'nav2_multirobot_params_2.yaml')
    #     }.items()
    # )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('voronoi'), 'rviz', name2 + '.rviz')]
    )

    # GAZEBO SERVER AND CLIENT
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': "true", 'extra_gazebo_args': 'verbose'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose': "true"}.items()
    )    

    # MAP MERGE
    merge_map_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(merge_map_launch_path),
        launch_arguments={}.items()
    )
        # For Robot 1
    static_transform_publisher1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_' + name1,
        arguments=['0', '0', '0', '0', '0', '0', 'map', name1 + '/map'],
        output='screen'
    )

    # For Robot 2
    static_transform_publisher2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_' + name2,
        arguments=['0', '0', '0', '0', '0', '0', 'map', name2 + '/map'],
        output='screen'
    )

    # Add to LaunchDescription
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_robot1)
    ld.add_action(robot_state_publisher1)
    ld.add_action(async_slam_toolbox1)
    # ld.add_action(nav2_1)
    # ld.add_action(rviz1)
    ld.add_action(spawn_robot2)
    ld.add_action(robot_state_publisher2)
    ld.add_action(async_slam_toolbox2)
    # ld.add_action(nav2_2)
    # ld.add_action(rviz2)
    ld.add_action(merge_map_cmd)
    ld.add_action(static_transform_publisher1)
    ld.add_action(static_transform_publisher2)
    return ld