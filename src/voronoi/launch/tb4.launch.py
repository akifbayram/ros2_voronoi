from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Define namespaces for each robot
    namespace_tb4_0 = LaunchConfiguration('namespace_tb4_0', default='tb4_0')
    namespace_tb4_1 = LaunchConfiguration('namespace_tb4_1', default='tb4_1')
    
    # Define paths for each launch file
    ignition_bringup_path = os.path.join(
        FindPackageShare('turtlebot4_ignition_bringup').find('turtlebot4_ignition_bringup'),
        'launch',
        'turtlebot4_ignition.launch.py'
    )
    
    spawn_path = os.path.join(
        FindPackageShare('turtlebot4_ignition_bringup').find('turtlebot4_ignition_bringup'),
        'launch',
        'turtlebot4_spawn.launch.py'
    )
    
    rviz_path = os.path.join(
        FindPackageShare('turtlebot4_viz').find('turtlebot4_viz'),
        'launch',
        'view_robot.launch.py'
    )
    
    nav2_path = os.path.join(
        FindPackageShare('turtlebot4_navigation').find('turtlebot4_navigation'),
        'launch',
        'nav2.launch.py'
    )
    
    slam_path = os.path.join(
        FindPackageShare('turtlebot4_navigation').find('turtlebot4_navigation'),
        'launch',
        'slam.launch.py'
    )

    ignition_bringup_tb4_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ignition_bringup_path),
        launch_arguments={'namespace': namespace_tb4_0}.items()
    )
    
    rviz_tb4_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_path),
        launch_arguments={'namespace': namespace_tb4_0}.items()
    )
    
    nav2_tb4_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_path),
        launch_arguments={'namespace': namespace_tb4_0}.items()
    )
    
    slam_tb4_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_path),
        launch_arguments={'namespace': namespace_tb4_0}.items()
    )

    # Launch actions for tb4_1 (second robot) with an initial 16-second delay, plus 4-second intervals between each action
    spawn_tb4_1 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_path),
                launch_arguments={'namespace': namespace_tb4_1, 'x': '0.0', 'y': '1.0'}.items()
            )
        ]
    )
    
    rviz_tb4_1 = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_path),
                launch_arguments={'namespace': namespace_tb4_1}.items()
            )
        ]
    )
    
    nav2_tb4_1 = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_path),
                launch_arguments={'namespace': namespace_tb4_1}.items()
            )
        ]
    )
    
    slam_tb4_1 = TimerAction(
        period=16.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_path),
                launch_arguments={'namespace': namespace_tb4_1}.items()
            )
        ]
    )

    return LaunchDescription([
        # Set environment variable for TurtleBot setup
        SetEnvironmentVariable(name='TURTLEBOT4_SETUP_BASH', value='/etc/turtlebot4/setup.bash'),
        
        # Launch for tb4_0 without delay
        ignition_bringup_tb4_0,
        rviz_tb4_0,
        nav2_tb4_0,
        slam_tb4_0,
        
        # Launch for tb4_1 with initial 10-second delay, then 4-second intervals
        spawn_tb4_1,
        rviz_tb4_1,
        nav2_tb4_1,
        slam_tb4_1,
    ])
