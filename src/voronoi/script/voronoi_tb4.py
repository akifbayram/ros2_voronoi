import rclpy
from rclpy.node import Node
import tf_transformations
import tf2_ros
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, radians, sqrt, pi, atan2
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pcl2
import numpy as np
import math
import random
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
import logging
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from typing import Dict, List, Tuple, Set, Optional
from dataclasses import dataclass

@dataclass
class RobotState:
    position: Point
    rotation: float
    current_goal: Optional[Tuple[float, float]]
    is_goal_active: bool
    frontier: Optional[Tuple[float, float]]
    information_nodes: Set[Tuple[float, float]]
    no_frontier_counter: int = 0  # New attribute

class MultiRobotExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_explorer')
        
        # Set up logging
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info('Initializing Multi-Robot Explorer')
        
        # Parameters
        self.declare_parameter('robot_namespaces', ['tb4_0', 'tb4_1'])
        self.declare_parameter('information_node_radius', 2.0)
        self.lambda_value = 0.7
        self.min_frontier_size = 3
        self.min_distance_threshold = 0.5
        self.goal_tolerance = 0.5
        
        # Get robot namespaces
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.information_node_radius = self.get_parameter('information_node_radius').value
        self.get_logger().info(f'Configured for robots: {self.robot_namespaces}')
        
        # Initialize robot states
        self.robots: Dict[str, RobotState] = {}
        for namespace in self.robot_namespaces:
            self.robots[namespace] = RobotState(
                position=Point(),
                rotation=0.0,
                current_goal=None,
                is_goal_active=False,
                frontier=None,
                information_nodes=set()
            )
        
        # Shared state
        self.map_data = None
        self.exploration_complete = False
        self.start_time = None
        self.no_frontier_threshold = 5  # Number of consecutive checks with no frontiers
        
        # TF setup
        self.get_logger().info('Setting up TF listener')
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create publishers and subscribers for each robot
        self.setup_robot_interfaces()
        
        # Create timers
        self.create_timer(0.1, self.exploration_loop)
        self.create_timer(1.0, self.publish_visualizations)
        
        self.get_logger().info('Multi-Robot Explorer initialization complete')

    def setup_robot_interfaces(self):
        """Set up publishers, subscribers, and action clients for each robot"""
        self.get_logger().info('Setting up robot interfaces')
        
        self.marker_pubs = {}
        self.marker_array_pubs = {}  # New dictionary for MarkerArray publishers
        self.frontier_pubs = {}
        self.goal_pubs = {}
        self.voronoi_pubs = {}
        self.nav_clients = {}
        
        for namespace in self.robot_namespaces:
            self.get_logger().debug(f'Setting up interfaces for {namespace}')
            
            # Publishers
            self.marker_pubs[namespace] = self.create_publisher(
                Marker, f'/{namespace}/frontier/exploration_markers', 10)
            
            self.marker_array_pubs[namespace] = self.create_publisher(
                MarkerArray, f'/{namespace}/frontier/exploration_info_markers', 10)  # New publisher for MarkerArray
            
            self.frontier_pubs[namespace] = self.create_publisher(
                PointCloud2, f'/{namespace}/frontier/frontier_points', 10)
            self.goal_pubs[namespace] = self.create_publisher(
                MarkerArray, f'/{namespace}/frontier/exploration_goals', 10)
            self.voronoi_pubs[namespace] = self.create_publisher(
                MarkerArray, f'/{namespace}/frontier/voronoi_partitions', 10)
                
            # Subscribers
            self.create_subscription(
                LaserScan, 
                f'/{namespace}/scan',
                lambda msg, ns=namespace: self.laser_callback(msg, ns),
                10
            )
            self.create_subscription(
                Odometry,
                f'/{namespace}/odom',
                lambda msg, ns=namespace: self.odom_callback(msg, ns),
                10
            )
            
            # Navigation action client
            self.nav_clients[namespace] = ActionClient(
                self, 
                NavigateToPose, 
                f'/{namespace}/navigate_to_pose'
            )
            self.get_logger().info(f'Waiting for {namespace} navigation action server...')
            if not self.nav_clients[namespace].wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f'Action server not available for {namespace}')
                continue  # Changed from return to continue to try other robots
            self.get_logger().info(f'Connected to {namespace} navigation action server')
            
        # Single map subscriber
        self.create_subscription(
            OccupancyGrid,
            '/tb4_0/map',
            self.map_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )
        
        self.get_logger().info('Robot interfaces setup complete')

    def laser_callback(self, msg: LaserScan, namespace: str):
        self.get_logger().debug(f'Received laser scan for {namespace}')
        # Process laser data if needed

    def odom_callback(self, msg: Odometry, namespace: str):
        self.get_logger().debug(f'Received odometry for {namespace}')
        robot = self.robots[namespace]
        robot.position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        robot.rotation = tf_transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )[2]
        self.get_logger().debug(
            f'{namespace} position: ({robot.position.x:.2f}, {robot.position.y:.2f})'
        )

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received new map update')
        self.map_data = msg
        self.get_logger().debug(
            f'Map dimensions: {msg.info.width}x{msg.info.height}, '
            f'resolution: {msg.info.resolution}'
        )

    def get_voronoi_partitions(self, robot_positions: List[Tuple[float, float]]) -> List[List[Tuple[float, float]]]:
        """Calculate Voronoi partitions based on robot positions"""
        if not self.map_data or not robot_positions:
            self.get_logger().warn('Cannot calculate Voronoi partitions: missing map or robot positions')
            return []
            
        self.get_logger().debug('Calculating Voronoi partitions')
        partitions = []
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        # Calculate Voronoi regions
        for i, robot_pos in enumerate(robot_positions):
            partition = []
            points_checked = 0
            points_added = 0
            
            for y in range(height):
                for x in range(width):
                    points_checked += 1
                    point_x = x * resolution + origin_x
                    point_y = y * resolution + origin_y
                    
                    # Check if point is closest to current robot
                    dist = math.hypot(point_x - robot_pos[0], point_y - robot_pos[1])
                    closest = True
                    
                    for j, other_pos in enumerate(robot_positions):
                        if i != j:
                            other_dist = math.hypot(
                                point_x - other_pos[0],
                                point_y - other_pos[1]
                            )
                            if other_dist < dist:
                                closest = False
                                break
                    
                    if closest:
                        partition.append((point_x, point_y))
                        points_added += 1
            
            self.get_logger().debug(
                f'Partition {i}: checked {points_checked} points, '
                f'added {points_added} points'
            )
            partitions.append(partition)
        
        return partitions

    def check_information_node_needed(self, robot: RobotState) -> bool:
        """Check if robot needs to drop an information node"""
        # Always allow dropping information nodes for single robot scenarios
        return True

    def drop_information_node(self, robot: RobotState):
        """Drop an information node at robot's current position"""
        node_pos = (round(robot.position.x, 2), round(robot.position.y, 2))  # Rounded for consistency
        robot.information_nodes.add(node_pos)
        self.get_logger().info(
            f'Dropped information node at {node_pos}, '
            f'total nodes: {len(robot.information_nodes)}'
        )

    def get_frontiers_in_partition(self, partition: List[Tuple[float, float]]) -> List[Tuple[int, int]]:
        """Get frontier points within a Voronoi partition"""
        if not self.map_data:
            self.get_logger().warn('Cannot get frontiers: no map data')
            return []
            
        self.get_logger().debug('Searching for frontiers in partition')
        frontiers = []
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        # Convert partition points to map coordinates
        partition_cells = set()
        for px, py in partition:
            map_x = int((px - origin_x) / resolution)
            map_y = int((py - origin_y) / resolution)
            if 0 <= map_x < width and 0 <= map_y < height:
                partition_cells.add((map_y, map_x))
        
        self.get_logger().debug(f'Partition contains {len(partition_cells)} cells')
        
        # Find frontiers within partition
        map_array = np.array(self.map_data.data).reshape((height, width))
        for i, j in partition_cells:
            if map_array[i, j] == 0:  # Free cell
                neighbors = [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]
                for ni, nj in neighbors:
                    if (0 <= ni < height and 0 <= nj < width and 
                        map_array[ni, nj] == -1):  # Unknown cell
                        frontiers.append((i, j))
                        break
        
        self.get_logger().debug(f'Found {len(frontiers)} frontier points')
        return frontiers

    def select_best_frontier(self, robot: RobotState, frontiers: List[Tuple[int, int]]) -> Optional[Tuple[float, float]]:
        """Select best frontier point based on distance and information gain"""
        if not frontiers:
            self.get_logger().debug('No frontiers available')
            return None
            
        self.get_logger().debug(f'Selecting best frontier from {len(frontiers)} points')
        best_score = float('-inf')
        best_frontier = None
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        for i, j in frontiers:
            # Convert to world coordinates
            frontier_x = j * resolution + origin_x
            frontier_y = i * resolution + origin_y
            
            # Calculate distance
            distance = math.hypot(
                frontier_x - robot.position.x,
                frontier_y - robot.position.y
            )
            
            if distance < self.min_distance_threshold:
                continue
            
            # Calculate information gain
            info_gain = len(self.get_unknown_neighbors(i, j))
            
            # Combine metrics
            score = self.lambda_value * info_gain - (1 - self.lambda_value) * distance
            
            self.get_logger().debug(
                f'Frontier at ({frontier_x:.2f}, {frontier_y:.2f}): '
                f'distance={distance:.2f}, info_gain={info_gain}, score={score:.2f}'
            )
            
            if score > best_score:
                best_score = score
                best_frontier = (frontier_x, frontier_y)
        
        if best_frontier:
            self.get_logger().info(
                f'Selected frontier at {best_frontier} with score {best_score:.2f}'
            )
        else:
            self.get_logger().warn('No suitable frontier found')
            
        return best_frontier

    def get_unknown_neighbors(self, i: int, j: int) -> List[Tuple[int, int]]:
        """Get unknown neighboring cells"""
        if not self.map_data:
            return []
            
        unknown = []
        height = self.map_data.info.height
        width = self.map_data.info.width
        map_array = np.array(self.map_data.data).reshape((height, width))
        
        neighbors = [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]
        for ni, nj in neighbors:
            if (0 <= ni < height and 0 <= nj < width and 
                map_array[ni, nj] == -1):
                unknown.append((ni, nj))
        
        return unknown

    def send_goal(self, namespace: str, x: float, y: float):
        """Send navigation goal to specific robot"""
        self.get_logger().info(f'Sending goal to {namespace}: ({x:.2f}, {y:.2f})')
        robot = self.robots[namespace]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        angle = math.atan2(y - robot.position.y, x - robot.position.x)
        q = tf_transformations.quaternion_from_euler(0, 0, angle)
        goal_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        try:
            robot.is_goal_active = True
            future = self.nav_clients[namespace].send_goal_async(
                goal_msg,
                feedback_callback=lambda msg: self.feedback_callback(msg, namespace)
            )
            future.add_done_callback(
                lambda f: self.goal_response_callback(f, namespace)
            )
        except Exception as e:
            self.get_logger().error(f'Failed to send goal for {namespace}: {str(e)}')
            robot.is_goal_active = False

    def goal_response_callback(self, future, namespace: str):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f'Goal rejected for {namespace}')
                self.robots[namespace].is_goal_active = False
                return

            self.get_logger().info(f'Goal accepted for {namespace}')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.goal_result_callback(f, namespace)
            )
        except Exception as e:
            self.get_logger().error(f'Error in goal response callback for {namespace}: {str(e)}')
            self.robots[namespace].is_goal_active = False

    def goal_result_callback(self, future, namespace: str):
        try:
            result = future.result()
            status = result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Goal succeeded for {namespace}')
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(f'Goal aborted for {namespace}')
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn(f'Goal canceled for {namespace}')
            else:
                self.get_logger().warn(f'Goal failed for {namespace} with status: {status}')
            self.robots[namespace].is_goal_active = False

            # Additional verification: Check robot's current position vs. goal
            if self.robots[namespace].current_goal:
                distance = math.hypot(
                    self.robots[namespace].position.x - self.robots[namespace].current_goal[0],
                    self.robots[namespace].position.y - self.robots[namespace].current_goal[1]
                )
                if distance > self.goal_tolerance:
                    self.get_logger().warn(f'Robot {namespace} did not reach the goal. Resetting goal state.')
                    self.robots[namespace].is_goal_active = False
                    self.robots[namespace].current_goal = None
                    # Optionally, reassign a new goal or handle as needed

        except Exception as e:
            self.get_logger().error(f'Error in goal result callback for {namespace}: {str(e)}')
            self.robots[namespace].is_goal_active = False

    def feedback_callback(self, feedback_msg, namespace: str):
        robot = self.robots[namespace]
        if not robot.current_goal:
            return
            
        try:
            distance = math.hypot(
                robot.current_goal[0] - robot.position.x,
                robot.current_goal[1] - robot.position.y
            )
            
            self.get_logger().debug(
                f'{namespace} distance to goal: {distance:.2f}'
            )
            
            if distance <= self.goal_tolerance:
                self.get_logger().info(f'Goal reached for {namespace}')
                robot.current_goal = None
                robot.is_goal_active = False
        except Exception as e:
            self.get_logger().error(f'Error in feedback callback for {namespace}: {str(e)}')

    def exploration_loop(self):
        """Main exploration loop implementing Algorithm 1"""
        if not self.map_data:
            self.get_logger().debug('Waiting for map data...')
            return

        if self.exploration_complete:
            self.get_logger().debug('Exploration already completed.')
            return

        try:
            if self.start_time is None:
                self.start_time = time.perf_counter()
                self.get_logger().info('Starting exploration')

            # Get robot positions for Voronoi partitioning
            robot_positions = []
            for namespace, robot in self.robots.items():
                if not hasattr(robot.position, 'x'):
                    self.get_logger().warn(f'No position data for {namespace} yet')
                    return
                robot_positions.append((robot.position.x, robot.position.y))

            self.get_logger().debug(f'Robot positions: {robot_positions}')

            # Calculate Voronoi partitions
            partitions = self.get_voronoi_partitions(robot_positions)
            if not partitions:
                self.get_logger().warn('Failed to calculate Voronoi partitions')
                return

            # Process each robot
            all_frontiers_empty = True
            for namespace, partition in zip(self.robot_namespaces, partitions):
                self.get_logger().debug(f'Processing robot: {namespace}')
                robot = self.robots[namespace]

                # Step 3-5: Check and drop information nodes
                if self.check_information_node_needed(robot):
                    self.drop_information_node(robot)

                # Step 6-9: Handle frontier assignment
                if not robot.is_goal_active:
                    # Get frontiers in robot's partition
                    frontiers = self.get_frontiers_in_partition(partition)
                    if frontiers:
                        all_frontiers_empty = False
                        # Select best frontier
                        next_frontier = self.select_best_frontier(robot, frontiers)
                        if next_frontier:
                            robot.frontier = next_frontier
                            robot.current_goal = next_frontier
                            self.send_goal(namespace, next_frontier[0], next_frontier[1])

                # Step 11-15: Handle frontier reaching
                if robot.frontier and robot.current_goal:
                    distance = math.hypot(
                        robot.position.x - robot.frontier[0],
                        robot.position.y - robot.frontier[1]
                    )
                    if distance <= self.goal_tolerance:
                        self.drop_information_node(robot)
                        robot.frontier = None
                        robot.current_goal = None

            # Check if exploration is complete
            if all_frontiers_empty:
                for robot in self.robots.values():
                    robot.no_frontier_counter += 1
                    if robot.no_frontier_counter >= self.no_frontier_threshold:
                        self.exploration_complete = True
                        self.get_logger().info(
                            'Multi-robot exploration completed after multiple checks with no frontiers!'
                        )
            else:
                for robot in self.robots.values():
                    robot.no_frontier_counter = 0  # Reset counter if frontiers are found

        except Exception as e:
            self.get_logger().error(f'Error in exploration loop: {str(e)}')

    def publish_visualizations(self):
        """Publish visualization markers for all robots"""
        if not self.map_data:
            return
        
        try:
            # Get robot positions for Voronoi partitioning
            robot_positions = [
                (robot.position.x, robot.position.y)
                for robot in self.robots.values()
            ]
            
            # Calculate Voronoi partitions
            partitions = self.get_voronoi_partitions(robot_positions)
            
            # Publish visualizations for each robot
            for namespace, partition in zip(self.robot_namespaces, partitions):
                robot = self.robots[namespace]
                
                # Publish Voronoi partition as LINE_LIST
                marker_array = MarkerArray()
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f'{namespace}_voronoi'
                marker.id = 0
                marker.type = Marker.LINE_LIST  # Changed to LINE_LIST
                marker.action = Marker.ADD
                marker.scale.x = 0.05
                marker.color.a = 0.5
                marker.color.r = random.random()
                marker.color.g = random.random()
                marker.color.b = random.random()
                
                # Pair points to form line segments
                sorted_partition = sorted(partition, key=lambda p: (p[0], p[1]))
                for i in range(0, len(sorted_partition)-1, 2):
                    p1 = Point(x=sorted_partition[i][0], y=sorted_partition[i][1], z=0.0)
                    p2 = Point(x=sorted_partition[i+1][0], y=sorted_partition[i+1][1], z=0.0)
                    marker.points.extend([p1, p2])
                
                marker_array.markers.append(marker)
                self.voronoi_pubs[namespace].publish(marker_array)
                
                # Publish current goal
                if robot.current_goal:
                    goal_marker = Marker()
                    goal_marker.header.frame_id = 'map'
                    goal_marker.header.stamp = self.get_clock().now().to_msg()
                    goal_marker.ns = f'{namespace}_goal'
                    goal_marker.id = 0
                    goal_marker.type = Marker.SPHERE
                    goal_marker.action = Marker.ADD
                    goal_marker.pose.position.x = robot.current_goal[0]
                    goal_marker.pose.position.y = robot.current_goal[1]
                    goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
                    goal_marker.color.r = 0.0
                    goal_marker.color.g = 1.0
                    goal_marker.color.b = 0.0
                    goal_marker.color.a = 1.0
                    
                    self.marker_pubs[namespace].publish(goal_marker)  # Correct publisher for Marker
                
                # Publish information nodes
                info_marker_array = MarkerArray()
                for i, node in enumerate(robot.information_nodes):
                    info_marker = Marker()
                    info_marker.header.frame_id = 'map'
                    info_marker.header.stamp = self.get_clock().now().to_msg()
                    info_marker.ns = f'{namespace}_info_nodes'
                    info_marker.id = i
                    info_marker.type = Marker.CYLINDER
                    info_marker.action = Marker.ADD
                    info_marker.pose.position.x = node[0]
                    info_marker.pose.position.y = node[1]
                    info_marker.pose.position.z = 0.1
                    info_marker.scale.x = info_marker.scale.y = 0.2
                    info_marker.scale.z = 0.1
                    info_marker.color.r = 1.0
                    info_marker.color.g = 0.0
                    info_marker.color.b = 1.0
                    info_marker.color.a = 0.7
                    
                    info_marker_array.markers.append(info_marker)
                
                if info_marker_array.markers:
                    self.marker_array_pubs[namespace].publish(info_marker_array)  # Correct publisher for MarkerArray
        
        except Exception as e:
            self.get_logger().error(f'Error in publish_visualizations: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        explorer = MultiRobotExplorer()
        explorer.get_logger().info('Multi-Robot Explorer node started')
        rclpy.spin(explorer)
    except Exception as e:
        logging.error(f'Error in main: {str(e)}')
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()