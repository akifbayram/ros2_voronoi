import rclpy
from rclpy.node import Node
import tf_transformations
import tf2_ros
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from typing import Dict, List, Tuple, Set, Optional
from dataclasses import dataclass, field
import heapq
import scipy.interpolate as si  # For B-spline interpolation

@dataclass
class RobotState:
    position: Point
    rotation: float
    current_goal: Optional[Tuple[float, float]]
    is_goal_active: bool
    frontier: Optional[Tuple[float, float]]
    information_nodes: Set[Tuple[float, float]]
    no_frontier_counter: int = 0  # Counter for no frontiers
    waiting_for_map_update: bool = False  # Flag to wait for map update
    map_update_wait_start: Optional[float] = None  # Time when the robot started waiting
    laser_scan: Optional[LaserScan] = None  # Store laser scan data
    path: List[Point] = field(default_factory=list)  # List to store the robot's traversed path
    planned_path: List[Tuple[float, float]] = field(default_factory=list)
    path_index: int = 0  # Index for path following
    goals_history: List[Marker] = field(default_factory=list)  # History of goal markers

class MultiRobotExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_explorer')

        # Set up logging
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().info('Initializing Voronoi Explorer')

        # Parameters
        self.declare_parameter('robot_namespaces', ['tb3_0', 'tb3_1'])
        self.declare_parameter('information_node_radius', 2.0)
        self.lambda_value = 0.7
        self.min_frontier_size = 3
        self.min_distance_threshold = 0.5
        self.goal_tolerance = 0.15
        self.lookahead_distance = 0.15
        self.speed = 0.1  # Adjusted speed to match the first code block

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
        self.no_frontier_threshold = 5
        self.last_map_update_time = self.get_clock().now()

        # TF setup
        self.get_logger().info('Setting up TF listener')
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.setup_robot_interfaces()

        # Initialize Publishers for the additional functionalities (per robot)
        self.planned_path_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.robot_path_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.goal_marker_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.goals_marker_pubs: Dict[str, rclpy.publisher.Publisher] = {}

        for namespace in self.robot_namespaces:
            # Publisher for planned path
            self.planned_path_pubs[namespace] = self.create_publisher(
                Path, f'/{namespace}/planned_path', 10)
            # Publisher for robot traversed path
            self.robot_path_pubs[namespace] = self.create_publisher(
                Path, f'/{namespace}/robot_path', 10)
            # Publisher for current goal marker
            self.goal_marker_pubs[namespace] = self.create_publisher(
                Marker, f'/{namespace}/goal_marker', 10)
            # Publisher for goals history marker array
            self.goals_marker_pubs[namespace] = self.create_publisher(
                MarkerArray, f'/{namespace}/goals_marker_array', 10)

        # Create timer for exploration loop
        self.create_timer(0.1, self.exploration_loop)

        self.get_logger().info('Voronoi Explorer initialization complete')

    def setup_robot_interfaces(self):
        """Set up publishers, subscribers, and action clients for each robot"""
        self.get_logger().info('Setting up robot interfaces')

        self.goal_pubs = {}
        self.cmd_vel_pubs = {}

        for namespace in self.robot_namespaces:
            self.get_logger().debug(f'Setting up interfaces for {namespace}')

            # Goal publisher
            self.goal_pubs[namespace] = self.create_publisher(
                PoseStamped, f'/{namespace}/goal_pose', 10)
            self.get_logger().debug(f'Created publisher for {namespace}/goal_pose')

            # cmd_vel publisher
            self.cmd_vel_pubs[namespace] = self.create_publisher(
                Twist, f'/{namespace}/cmd_vel', 10)
            self.get_logger().debug(f'Created publisher for {namespace}/cmd_vel')

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

            # Create timers for path following
            self.create_timer(0.1, self.make_follow_goal_callback(namespace))

        # Single map subscriber
        self.create_subscription(
            OccupancyGrid,
            '/merge_map',
            self.map_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        self.get_logger().info('Robot interfaces setup complete')

    def make_follow_goal_callback(self, namespace):
        def callback():
            self.follow_goal(namespace)
        return callback

    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        t0 = +2.0 * (w * z + x * y)
        t1 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t0, t1)
        return yaw_z

    def heuristic(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def astar(self, array, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data = data + [start]
                data = data[::-1]
                return data
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:
                        if array[neighbor[0]][neighbor[1]] != 0:
                            continue
                    else:
                        continue
                else:
                    continue
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        # If no path to goal was found, return closest path to goal
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                dist = self.heuristic(node, goal)
                if dist < closest_dist:
                    closest_node = node
                    closest_dist = dist
            if closest_node is not None:
                data = []
                while closest_node in came_from:
                    data.append(closest_node)
                    closest_node = came_from[closest_node]
                data = data + [start]
                data = data[::-1]
                return data
        return None  # Return None if no path is found

    def bspline_planning(self, path, sn=100):
        try:
            array = np.array(path)
            x = array[:, 0]
            y = array[:, 1]
            N = 2
            t = range(len(x))
            x_tup = si.splrep(t, x, k=N)
            y_tup = si.splrep(t, y, k=N)

            x_list = list(x_tup)
            xl = x.tolist()
            x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

            y_list = list(y_tup)
            yl = y.tolist()
            y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

            ipl_t = np.linspace(0.0, len(x) - 1, sn)
            rx = si.splev(ipl_t, x_list)
            ry = si.splev(ipl_t, y_list)
            path = [(rx[i], ry[i]) for i in range(len(rx))]
        except Exception as e:
            self.get_logger().error(f'B-spline planning error: {str(e)}')
            path = path
        return path

    def pure_pursuit(self, current_x, current_y, current_heading, path, index):
        lookahead_distance = self.lookahead_distance
        closest_point = None
        v = self.speed
        for i in range(index, len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            if lookahead_distance < distance:
                closest_point = (x, y)
                index = i
                break
        if closest_point is not None:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
            desired_steering_angle = target_heading - current_heading
        else:
            target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
            desired_steering_angle = target_heading - current_heading
            index = len(path) - 1
        if desired_steering_angle > math.pi:
            desired_steering_angle -= 2 * math.pi
        elif desired_steering_angle < -math.pi:
            desired_steering_angle += 2 * math.pi
        if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
            sign = 1 if desired_steering_angle > 0 else -1
            desired_steering_angle = sign * math.pi/4
            v = 0.0
        return v, desired_steering_angle, index

    def follow_goal(self, namespace):
        robot = self.robots[namespace]
        if robot.is_goal_active and robot.planned_path:
            current_heading = robot.rotation
            current_x = robot.position.x
            current_y = robot.position.y

            # Normal path following when no obstacles
            v, w, robot.path_index = self.pure_pursuit(
                current_x, current_y, current_heading,
                robot.planned_path, robot.path_index
            )

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_vel_pubs[namespace].publish(twist)

            # Publish the robot's traversed path
            self.publish_robot_path(namespace)

            # Check if we have reached the end of the planned path
            if robot.path_index >= len(robot.planned_path) - 1:
                distance_to_goal = math.hypot(
                    robot.current_goal[0] - robot.position.x,
                    robot.current_goal[1] - robot.position.y
                )

                if distance_to_goal < self.goal_tolerance:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pubs[namespace].publish(twist)
                    self.get_logger().info(f'Robot {namespace} reached its goal.')
                    robot.is_goal_active = False
                    robot.frontier = None
                    robot.current_goal = None
                    robot.planned_path = []
                    robot.path_index = 0

                    # Publish the reached goal marker
                    self.publish_goal_marker(namespace, robot.current_goal[0], robot.current_goal[1])
                    self.create_goal_marker(namespace, robot.current_goal[0], robot.current_goal[1])
                    self.publish_goal_markers(namespace)

                    # Introduce a delay to wait for the map to update
                    robot.waiting_for_map_update = True
                    robot.map_update_wait_start = self.get_clock().now()
        elif robot.waiting_for_map_update:
            time_waiting = (self.get_clock().now() - robot.map_update_wait_start).nanoseconds / 1e9
            if time_waiting >= 1.0:  # Wait for 1 second
                robot.waiting_for_map_update = False
                self.get_logger().info(f'Robot {namespace} finished waiting for map update.')

    def laser_callback(self, msg: LaserScan, namespace: str):
        # Store the latest laser scan data
        robot = self.robots[namespace]
        robot.laser_scan = msg

    def odom_callback(self, msg: Odometry, namespace: str):
        robot = self.robots[namespace]
        robot.position = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        robot.rotation = self.euler_from_quaternion(quat)
        self.get_logger().debug(f'{namespace} position updated to: ({robot.position.x}, {robot.position.y})')

        # Update traversed path
        if not robot.path:
            robot.path.append(Point(x=robot.position.x, y=robot.position.y, z=robot.position.z))
        else:
            last_point = robot.path[-1]
            dx = robot.position.x - last_point.x
            dy = robot.position.y - last_point.y
            distance = math.hypot(dx, dy)
            if distance > 0.05:
                robot.path.append(Point(x=robot.position.x, y=robot.position.y, z=robot.position.z))
                # Optionally limit the traversed path length
                if len(robot.path) > 1000:
                    robot.path.pop(0)  # Remove the oldest point

        # Publish the traversed path
        # Note: Do NOT publish to 'planned_path' here
        # Use 'robot_path_pubs' to publish to 'robot_path'
        self.publish_robot_path(namespace)

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.last_map_update_time = self.get_clock().now()
        self.get_logger().debug('Map updated at timestamp: {}'.format(msg.header.stamp))

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

        # Prepare meshgrid of map coordinates
        map_x = np.linspace(origin_x, origin_x + (width * resolution), width)
        map_y = np.linspace(origin_y, origin_y + (height * resolution), height)
        mesh_x, mesh_y = np.meshgrid(map_x, map_y)
        points = np.vstack((mesh_x.flatten(), mesh_y.flatten())).T

        # Assign each point to the nearest robot
        robot_positions_array = np.array(robot_positions)
        distances = np.linalg.norm(points[:, np.newaxis, :] - robot_positions_array[np.newaxis, :, :], axis=2)
        nearest_robot_indices = np.argmin(distances, axis=1)

        # Create partitions
        for i in range(len(robot_positions)):
            partition_indices = np.where(nearest_robot_indices == i)[0]
            partition_points = points[partition_indices]
            partitions.append(partition_points.tolist())

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
        map_array = np.array(self.map_data.data).reshape((height, width))
        partition_cells = set()
        for px, py in partition:
            map_x = int((px - origin_x) / resolution)
            map_y = int((py - origin_y) / resolution)
            if 0 <= map_x < width and 0 <= map_y < height:
                partition_cells.add((map_y, map_x))

        self.get_logger().debug(f'Partition contains {len(partition_cells)} cells')

        # Find frontiers within partition
        for i, j in partition_cells:
            if map_array[i, j] == 0:  # Free cell
                neighbors = [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]
                for ni, nj in neighbors:
                    if (0 <= ni < height and 0 <= nj < width and
                        map_array[ni, nj] == -1):  # Unknown cell
                        frontiers.append((i, j))
                        break

        self.get_logger().info(f'Frontiers found: {len(frontiers)}')
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
        """Send navigation goal to specific robot by planning a path"""
        self.get_logger().info(f'Sending goal to {namespace}: ({x:.2f}, {y:.2f})')
        robot = self.robots[namespace]
        robot.current_goal = (x, y)
        robot.is_goal_active = True

        # Plan path using A*
        path = self.plan_path(robot, (x, y))
        if path:
            robot.planned_path = path
            robot.path_index = 0
            self.get_logger().info(f'Robot {namespace} path planned with {len(path)} points.')

            # Publish the planned path
            self.publish_planned_path(namespace, path)

            # Optionally, publish a goal marker
            self.publish_goal_marker(namespace, x, y)
            self.create_goal_marker(namespace, x, y)
            self.publish_goal_markers(namespace)
        else:
            self.get_logger().warn(f'Robot {namespace} could not plan a path to the goal.')
            robot.is_goal_active = False
            robot.current_goal = None

    def plan_path(self, robot: RobotState, goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Plan a path from the robot's current position to the goal using A* and smooth it"""
        if not self.map_data:
            self.get_logger().warn('Cannot plan path: no map data')
            return None

        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        width = self.map_data.info.width
        height = self.map_data.info.height

        map_array = np.array(self.map_data.data).reshape((height, width))
        # Occupied cells are marked with 100, unknown with -1
        # For path planning, we consider occupied cells as 1 and free cells as 0
        obstacle_map = np.zeros_like(map_array)
        obstacle_map[map_array == 100] = 1
        obstacle_map[map_array == -1] = 1  # Treat unknown as obstacles

        # Convert robot position and goal to map indices
        start_x = int((robot.position.x - origin_x) / resolution)
        start_y = int((robot.position.y - origin_y) / resolution)
        goal_x = int((goal[0] - origin_x) / resolution)
        goal_y = int((goal[1] - origin_y) / resolution)

        # Ensure indices are within bounds
        start_x = np.clip(start_x, 0, width - 1)
        start_y = np.clip(start_y, 0, height - 1)
        goal_x = np.clip(goal_x, 0, width - 1)
        goal_y = np.clip(goal_y, 0, height - 1)

        start = (start_y, start_x)
        goal = (goal_y, goal_x)

        path_indices = self.astar(obstacle_map, start, goal)
        if path_indices is None:
            return None  # No path found

        # Convert path indices back to world coordinates
        path = []
        for idx in path_indices:
            y, x = idx
            wx = x * resolution + origin_x
            wy = y * resolution + origin_y
            path.append((wx, wy))

        # Smooth the path using B-spline
        path = self.bspline_planning(path)

        return path

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

            # Check if the map has been updated recently
            time_since_last_map_update = (self.get_clock().now() - self.last_map_update_time).nanoseconds / 1e9
            if time_since_last_map_update > 1.0:  # Wait for at least 1 second after the last map update
                self.get_logger().info('Waiting for map to update...')
                return

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
                robot = self.robots[namespace]

                if robot.waiting_for_map_update:
                    time_since_map_update = (self.last_map_update_time - robot.map_update_wait_start).nanoseconds / 1e9
                    if time_since_map_update > 0:
                        robot.waiting_for_map_update = False
                        self.get_logger().info(f'{namespace} detected map update. Resuming exploration.')
                    else:
                        self.get_logger().info(f'{namespace} is still waiting for map update.')
                        continue  # Skip to the next robot

                if not robot.is_goal_active:
                    self.get_logger().info(f'{namespace} is not active. Searching for new goal.')
                    # Get frontiers in robot's partition
                    frontiers = self.get_frontiers_in_partition(partition)
                    if frontiers:
                        all_frontiers_empty = False
                        # Select best frontier
                        next_frontier = self.select_best_frontier(robot, frontiers)
                        if next_frontier:
                            robot.frontier = next_frontier
                            self.send_goal(namespace, next_frontier[0], next_frontier[1])
                    else:
                        self.get_logger().info(f'No frontiers found for {namespace}.')
                else:
                    self.get_logger().info(f'{namespace} is active and moving towards the goal.')

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

    # Integrated Functions

    def publish_planned_path(self, namespace: str, path: List[Tuple[float, float]]):
        """Publish the planned path for a specific robot."""
        path_msg = Path()
        path_msg.header.frame_id = f'{namespace}/map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        poses = []
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = f'{namespace}/map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Assuming no rotation
            poses.append(pose)
        path_msg.poses = poses
        self.planned_path_pubs[namespace].publish(path_msg)

    def publish_goal_marker(self, namespace: str, goal_x: float, goal_y: float):
        """Publish a single goal marker for a specific robot."""
        marker = Marker()
        marker.header.frame_id = f'{namespace}/map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0  # Always use 0 for the current goal
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.goal_marker_pubs[namespace].publish(marker)

    def publish_robot_path(self, namespace: str):
        """Publish the traversed path followed by a specific robot."""
        robot = self.robots[namespace]
        path_msg = Path()
        path_msg.header.frame_id = f'{namespace}/map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        poses = []
        for p in robot.path:
            pose = PoseStamped()
            pose.header.frame_id = f'{namespace}/map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = p.x
            pose.pose.position.y = p.y
            pose.pose.position.z = p.z
            pose.pose.orientation.w = 1.0 
            poses.append(pose)
        path_msg.poses = poses
        self.robot_path_pubs[namespace].publish(path_msg)

    def create_goal_marker(self, namespace: str, goal_x: float, goal_y: float):
        """Create and store a goal marker for a specific robot."""
        marker = Marker()
        marker.header.frame_id = f'{namespace}/map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goals_history'
        marker.id = len(self.robots[namespace].goals_history) 
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.robots[namespace].goals_history.append(marker)

    def publish_goal_markers(self, namespace: str):
        """Publish all goal markers history for a specific robot."""
        marker_array = MarkerArray()
        marker_array.markers = self.robots[namespace].goals_history
        self.goals_marker_pubs[namespace].publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    try:
        explorer = MultiRobotExplorer()
        explorer.get_logger().info('Voronoi Explorer node started')
        rclpy.spin(explorer)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
