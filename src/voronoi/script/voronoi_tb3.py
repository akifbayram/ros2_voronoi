import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped, Pose
from nav_msgs.msg import OccupancyGrid, Path, GridCells
from sensor_msgs.msg import LaserScan
import tf2_ros
from math import cos, radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import math
import threading
import time
import random
import heapq
import scipy.interpolate as si
from visualization_msgs.msg import Marker, MarkerArray

# Parameters for navigation
LOOKAHEAD_DISTANCE = 0.25
SPEED = 0.15
EXPANSION_SIZE = 1  # For obstacle expansion in the map
LAMBDA_VALUE = 0.83
TARGET_EXPLORE_RATE = 0.90
REACHED_THRESHOLD = 0.3

def euler_from_quaternion_func(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def heuristic(a, b, logger=None):
    h = np.linalg.norm(np.array(a) - np.array(b))
    if logger:
        logger.debug(f"Heuristic between {a} and {b}: {h}")
    return h

def astar(array, start, goal, logger=None):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal, logger)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    if logger:
        logger.debug(f"A* Start: {start}, Goal: {goal}")
    while oheap:
        current = heapq.heappop(oheap)[1]
        if logger:
            logger.debug(f"A* Current node: {current}")
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data = data[::-1]
            if logger:
                logger.info(f"A* Path found: {data}")
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor, logger)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        if logger:
                            logger.debug(f"A* Skipping obstacle at {neighbor}")
                        continue
                else:
                    if logger:
                        logger.debug(f"A* Neighbor {neighbor} out of Y bounds")
                    continue
            else:
                if logger:
                    logger.debug(f"A* Neighbor {neighbor} out of X bounds")
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal, logger)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                if logger:
                    logger.debug(f"A* Adding neighbor {neighbor} with fscore {fscore[neighbor]}")
    if logger:
        logger.warning("A* failed to find a path")
    return False

def bspline_planning(array, sn, logger=None):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 3  # Degree of the spline
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)
        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0] * (N + 1)
        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0] * (N + 1)
        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_tup)
        ry = si.splev(ipl_t, y_tup)
        path = [(rx[i], ry[i]) for i in range(len(rx))]
        if logger:
            logger.debug(f"Bspline path generated with {len(path)} points")
    except Exception as e:
        if logger:
            logger.error(f"Bspline planning error: {e}")
        path = array
    return path

def pure_pursuit(current_x, current_y, current_heading, path, index, lookahead_distance, speed, logger=None):
    closest_point = None
    v = speed
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if logger:
            logger.debug(f"Pure Pursuit: Checking point {i} at ({x}, {y}) with distance {distance}")
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
        if logger:
            logger.debug(f"Pure Pursuit: Closest point found at {closest_point} with target heading {target_heading}")
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1
        if logger:
            logger.debug(f"Pure Pursuit: No point found, using last path point with heading {target_heading}")
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi / 6 or desired_steering_angle < -math.pi / 6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi / 4
        v = 0.0
        if logger:
            logger.debug(f"Pure Pursuit: Steering angle {desired_steering_angle} exceeds limit, setting angular velocity and stopping")
    if logger:
        logger.debug(f"Pure Pursuit: v={v}, omega={desired_steering_angle}, index={index}")
    return v, desired_steering_angle, index

def costmap(data, width, height, resolution, logger=None):
    data = np.array(data).reshape(height, width)
    if logger:
        logger.debug("Costmap: Initial costmap processed")

    wall = np.where(data == 100)
    expansion_size = EXPANSION_SIZE
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = wall[0] + i
            y = wall[1] + j
            x = np.clip(x, 0, height - 1)
            y = np.clip(y, 0, width - 1)
            data[x, y] = 100
    if logger:
        logger.debug(f"Costmap: Obstacles expanded by {EXPANSION_SIZE} cells")
    return data

class Robot:
    def __init__(self, robot_name, node, other_robots_positions, lock, tf_buffer):
        self.robot_name = robot_name
        self.node = node
        self.position = Point()
        self.rotation = 0.0
        self.arr_info_node = True  # Start by needing a target
        self.laser_crashed_value = False
        self.next_target_node = []
        self.record_info_node = []
        self.communication_max_range = 6.0
        self.laser_msg_range_max = None
        self.laser_values = None
        self.laser_msg = None
        self.record_info_node = [(self.position.x, self.position.y)]

        self.other_robots_positions = other_robots_positions  # Shared among robots
        self.lock = lock  # For thread-safe access to shared data

        # Use the shared tf_buffer
        self.tf_buffer = tf_buffer

        # Publishers and Subscribers
        self.cmd_vel_pub = node.create_publisher(Twist, '/' + self.robot_name + '/cmd_vel', 10)
        self.laser_sub = node.create_subscription(LaserScan, '/' + self.robot_name + '/scan', self.laser_callback, 10)
        
        # Subscription to the robot's partition
        self.partition_sub = node.create_subscription(GridCells, '/' + self.robot_name + '/partition', self.partition_callback, 10)
        self.partition_cells = []  # To store the partition cells

        # Visualization Publishers
        self.planned_path_pub = node.create_publisher(Path, '/' + self.robot_name + '/planned_path', 10)
        self.goal_marker_pub = node.create_publisher(Marker, '/' + self.robot_name + '/goal_current', 10)
        self.robot_path_pub = node.create_publisher(Path, '/' + self.robot_name + '/robot_path', 10)
        self.goals_marker_pub = node.create_publisher(MarkerArray, '/' + self.robot_name + '/goals_history', 10)
        self.goals_history = []

        # For path planning and navigation
        self.path = []
        self.path_index = 0
        self.traversed_path = []

        # Other variables
        self.arrived = False

        # Timer for controlling the robot
        self.timer_period = 0.1  # seconds
        self.timer = node.create_timer(self.timer_period, self.timer_callback)

        self.node.get_logger().info(f"{self.robot_name}: Robot initialized.")

    def laser_callback(self, msg):
        self.laser_msg = msg  # Store the entire message for angle_min, angle_increment
        self.laser_msg_range_max = msg.range_max
        self.laser_values = msg.ranges
        self.node.get_logger().debug(f"{self.robot_name}: Laser scan received with {len(msg.ranges)} ranges.")

    def partition_callback(self, msg):
        self.partition_cells = msg.cells
        self.node.get_logger().debug(f"{self.robot_name}: Partition received with {len(msg.cells)} cells.")

    def update_pose(self):
        """
        Update the robot's current position and orientation using tf2 transformations.
        """
        # Attempt to get the transform from 'map' to the robot's base frame
        base_frames = [self.robot_name + '/base_footprint', self.robot_name + '/base_link']
        found_transform = False
        trans = None
        for base_frame in base_frames:
            try:
                trans = self.tf_buffer.lookup_transform('map', base_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
                found_transform = True
                self.node.get_logger().debug(f"{self.robot_name}: Transform found using frame '{base_frame}'.")
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.node.get_logger().warn(f"{self.robot_name}: Could not get transform from 'map' to '{base_frame}': {e}")
                continue
        if not found_transform:
            self.node.get_logger().error(f"{self.robot_name}: Could not find any valid base frame.")
            # Handle error, e.g., stop the robot
            return False
        # Extract position and orientation
        self.position.x = trans.transform.translation.x
        self.position.y = trans.transform.translation.y
        self.position.z = trans.transform.translation.z
        q = trans.transform.rotation
        self.rotation = euler_from_quaternion_func(q.x, q.y, q.z, q.w)
        self.node.get_logger().debug(f"{self.robot_name}: Updated pose to x={self.position.x}, y={self.position.y}, yaw={self.rotation}")
        # Update shared position
        with self.lock:
            self.other_robots_positions[self.robot_name] = (self.position.x, self.position.y)
        return True

    def timer_callback(self):
        # Update pose
        if not self.update_pose():
            # Could not update pose, skip this cycle
            self.node.get_logger().warn(f"{self.robot_name}: Skipping cycle due to pose update failure.")
            return
        # Main control loop
        self.check_for_collisions()
        if self.arr_info_node or not self.next_target_node:
            self.select_target()
        elif self.reached_target():
            self.handle_reached_target()
        else:
            if not self.path:
                self.plan_path()
            else:
                self.follow_path()
                self.publish_robot_path()

    def check_for_collisions(self):
        # Check for potential collisions
        if self.laser_values:
            obstacle_detected = False
            for distance in self.laser_values:
                if distance < 0.2:
                    obstacle_detected = True
                    break
            if obstacle_detected:
                self.laser_crashed_value = True
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.node.get_logger().warn(f"{self.robot_name}: Obstacle detected within 0.2 meters. Stopping.")
                # Optionally, implement avoidance or re-planning here instead of stopping
                self.arr_info_node = True
                # Optionally, you can return here to prioritize collision avoidance
        else:
            self.node.get_logger().debug(f"{self.robot_name}: No laser data available.")

    def select_target(self):
        """
        Select the next target for the robot.
        Priority is given to frontier-based exploration. If no frontiers are available,
        the existing method based on laser scans and Min-Omega heuristic is used.
        """
        frontiers = self.detect_frontiers()
        if frontiers:
            # Select a frontier goal
            self.next_target_node = self.select_frontier_goal(frontiers)
            self.record_info_node.append(self.next_target_node)
            self.node.get_logger().info(f"{self.robot_name}: Frontier-based target selected at ({self.next_target_node[0]}, {self.next_target_node[1]})")
            self.arr_info_node = False
            self.arrived = False
            self.path = []
            self.path_index = 0
            # Publish goal marker
            self.publish_goal_marker(self.next_target_node[0], self.next_target_node[1])
            # Create and store goal marker
            self.create_goal_marker(self.next_target_node[0], self.next_target_node[1])
        else:
            # Fallback to existing method based on laser scans
            self.node.get_logger().debug(f"{self.robot_name}: No frontiers detected. Falling back to laser-based target selection.")
            option_target_point = []
            if self.laser_values is not None and self.laser_msg is not None:
                angle_increment = self.laser_msg.angle_increment
                angle_min = self.laser_msg.angle_min
                for i, distance in enumerate(self.laser_values):
                    if distance == float('inf') or distance >= self.laser_msg_range_max:
                        angle = i * angle_increment + angle_min + self.rotation
                        target_x = self.position.x + self.laser_msg_range_max * math.cos(angle)
                        target_y = self.position.y + self.laser_msg_range_max * math.sin(angle)
                        option_target_point.append([target_x, target_y])
                self.node.get_logger().debug(f"{self.robot_name}: {len(option_target_point)} potential target points generated from laser scan.")
                # Use Voronoi algorithm
                voronoi_points = self.voronoi_select_point(option_target_point)
                if voronoi_points:
                    # Use Min-Omega heuristic to select the next target
                    self.next_target_node = self.get_min_Omega_distance_point(voronoi_points)
                    self.record_info_node.append(self.next_target_node)
                    self.node.get_logger().info(f"{self.robot_name}: Laser-based target selected at ({self.next_target_node[0]}, {self.next_target_node[1]})")
                    self.arr_info_node = False
                    self.arrived = False
                    self.path = []
                    self.path_index = 0
                    # Publish goal marker
                    self.publish_goal_marker(self.next_target_node[0], self.next_target_node[1])
                    # Create and store goal marker
                    self.create_goal_marker(self.next_target_node[0], self.next_target_node[1])
                else:
                    self.node.get_logger().warning(f"{self.robot_name}: No valid Min-Omega point found. Using all option points.")
                    if option_target_point:
                        # Use Min-Omega heuristic on all option points
                        self.next_target_node = self.get_min_Omega_distance_point(option_target_point)
                        self.record_info_node.append(self.next_target_node)
                        self.node.get_logger().info(f"{self.robot_name}: Laser-based target selected at ({self.next_target_node[0]}, {self.next_target_node[1]})")
                        self.arr_info_node = False
                        self.arrived = False
                        self.path = []
                        self.path_index = 0
                        # Publish goal marker
                        self.publish_goal_marker(self.next_target_node[0], self.next_target_node[1])
                        # Create and store goal marker
                        self.create_goal_marker(self.next_target_node[0], self.next_target_node[1])
                    else:
                        self.node.get_logger().error(f"{self.robot_name}: No option target points available from laser scan.")
            else:
                self.node.get_logger().warning(f"{self.robot_name}: Waiting for laser scan data to generate target points.")

    def detect_frontiers(self):
        """
        Detect frontiers within the robot's partition.
        Frontiers are groups of free cells adjacent to unknown cells.
        """
        frontiers = set()

        if self.node.map_data is None or not self.partition_cells:
            self.node.get_logger().debug(f"{self.robot_name}: Cannot detect frontiers, map or partition not available.")
            return list(frontiers)

        # Map data: 0=free, 100=occupied, -1=unknown
        # Iterate through partition cells
        for cell in self.partition_cells:
            x = cell.x
            y = cell.y
            # Convert (x, y) to map indices
            j = int((x - self.node.origin_x) / self.node.resolution)
            i = int((y - self.node.origin_y) / self.node.resolution)

            if i < 0 or i >= self.node.map_height or j < 0 or j >= self.node.map_width:
                continue

            if self.node.map_data[i][j] != 0:
                continue  # Not a free cell

            # Check 8-connected neighbors for unknown cells
            for di in [-1, 0, 1]:
                for dj in [-1, 0, 1]:
                    if di == 0 and dj == 0:
                        continue
                    ni = i + di
                    nj = j + dj
                    if ni < 0 or ni >= self.node.map_height or nj < 0 or nj >= self.node.map_width:
                        continue
                    if self.node.map_data[ni][nj] == -1:
                        frontiers.add((x, y))
                        break  # No need to check other neighbors for this cell

        self.node.get_logger().debug(f"{self.robot_name}: Detected {len(frontiers)} individual frontiers.")

        # Group frontiers into connected clusters
        frontier_clusters = self.group_frontiers(frontiers)
        self.node.get_logger().debug(f"{self.robot_name}: Grouped into {len(frontier_clusters)} frontier clusters.")

        return frontier_clusters

    def group_frontiers(self, frontiers, connectivity=8):
        """
        Group frontiers into connected clusters using a simple flood-fill algorithm.
        Returns a list of clusters, each cluster is a list of (x, y) tuples.
        """
        clusters = []
        frontiers = set(frontiers)

        while frontiers:
            current = frontiers.pop()
            cluster = [current]
            queue = [current]

            while queue:
                point = queue.pop(0)
                x, y = point
                # Check neighboring points
                neighbors = [
                    (x + self.node.resolution, y),
                    (x - self.node.resolution, y),
                    (x, y + self.node.resolution),
                    (x, y - self.node.resolution),
                    (x + self.node.resolution, y + self.node.resolution),
                    (x + self.node.resolution, y - self.node.resolution),
                    (x - self.node.resolution, y + self.node.resolution),
                    (x - self.node.resolution, y - self.node.resolution),
                ]

                for neighbor in neighbors:
                    if neighbor in frontiers:
                        frontiers.remove(neighbor)
                        cluster.append(neighbor)
                        queue.append(neighbor)

            clusters.append(cluster)

        return clusters

    def select_frontier_goal(self, frontier_clusters):
        """
        Select the best frontier cluster to set as the goal.
        Prioritize larger frontiers, and among those, select the closest one.
        """
        if not frontier_clusters:
            self.node.get_logger().error(f"{self.robot_name}: No frontier clusters available for selection.")
            return None

        # Calculate size and centroid of each cluster
        frontier_info = []
        for cluster in frontier_clusters:
            size = len(cluster)
            centroid_x = sum([point[0] for point in cluster]) / size
            centroid_y = sum([point[1] for point in cluster]) / size
            distance = math.hypot(centroid_x - self.position.x, centroid_y - self.position.y)
            frontier_info.append({
                'size': size,
                'centroid': (centroid_x, centroid_y),
                'distance': distance
            })

        # Sort clusters by size descending, then by distance ascending
        frontier_info.sort(key=lambda x: (-x['size'], x['distance']))

        # Select the first cluster's centroid as the goal
        best_frontier = frontier_info[0]
        self.node.get_logger().debug(f"{self.robot_name}: Selected frontier with size {best_frontier['size']} at {best_frontier['centroid']} with distance {best_frontier['distance']}")
        return best_frontier['centroid']

    def follow_path(self):
        # Implement pure pursuit to follow the path
        if self.path_index >= len(self.path):
            self.node.get_logger().debug(f"{self.robot_name}: Reached end of path.")
            return

        # Check for obstacles blocking the path
        if self.is_path_obstructed():
            self.node.get_logger().info(f"{self.robot_name}: Path obstructed, re-planning.")
            self.arr_info_node = True
            self.next_target_node = []
            self.path = []
            self.path_index = 0
            # **Do not call plan_path() here**
            # Allow timer_callback to handle re-planning
            return

        twist = Twist()
        v, omega, self.path_index = pure_pursuit(
            self.position.x, self.position.y, self.rotation,
            self.path, self.path_index, LOOKAHEAD_DISTANCE, SPEED, self.node.get_logger()
        )
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)
        self.node.get_logger().debug(f"{self.robot_name}: Published cmd_vel with linear.x={v}, angular.z={omega}")

        if self.path_index >= len(self.path) - 1:
            self.arr_info_node = True
            self.arrived = True
            self.path = []
            self.node.get_logger().info(f"{self.robot_name}: Completed path following.")

    def handle_reached_target(self):
        self.arr_info_node = True
        self.arrived = True
        self.node.get_logger().info(f"{self.robot_name}: Reached target at {self.next_target_node}.")
        # Publish goal marker history
        self.publish_goal_markers()

    def get_min_Omega_distance_point(self, option_target_points):
        """
        Use Min-Omega heuristic to select the best target point.
        """
        min_Omega = float('inf')
        best_point = None
        # Last target position
        if len(self.record_info_node) >= 1:
            last_target = self.record_info_node[-1]
        else:
            # If no previous target, use current position
            last_target = (self.position.x, self.position.y)
        for point in option_target_points:
            # Distance between last target and candidate point (d_ik)
            d_ik = math.hypot(point[0] - last_target[0], point[1] - last_target[1])
            # Distance between current position and candidate point (phi_ik)
            phi_ik = math.hypot(point[0] - self.position.x, point[1] - self.position.y)
            # Calculate Omega
            Omega = LAMBDA_VALUE * d_ik + (1 - LAMBDA_VALUE) * phi_ik
            self.node.get_logger().debug(f"{self.robot_name}: Omega for point ({point[0]}, {point[1]}): {Omega}")
            if Omega < min_Omega:
                min_Omega = Omega
                best_point = point
        if best_point is None:
            self.node.get_logger().error(f"{self.robot_name}: No valid point found using Min-Omega heuristic.")
            # Fallback to a random point
            best_point = random.choice(option_target_points)
        return best_point

    def voronoi_select_point(self, option_target_point):
        # Implement Voronoi selection based on other robots' positions
        voronoi_option_target_point = []
        with self.lock:
            other_robot_positions = [pos for name, pos in self.other_robots_positions.items() if name != self.robot_name]
        if self.node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.node.get_logger().debug(f"{self.robot_name}: Other robots positions for Voronoi: {other_robot_positions}")
        if not other_robot_positions:
            return option_target_point
        for point in option_target_point:
            is_closest = True
            distance_to_self = sqrt(pow(point[0] - self.position.x, 2) + pow(point[1] - self.position.y, 2))
            for other_pos in other_robot_positions:
                distance_to_other = sqrt(pow(point[0] - other_pos[0], 2) + pow(point[1] - other_pos[1], 2))
                if distance_to_other < distance_to_self:
                    is_closest = False
                    break
            if is_closest:
                voronoi_option_target_point.append(point)
        self.node.get_logger().debug(f"{self.robot_name}: Voronoi selected {len(voronoi_option_target_point)} points.")
        return voronoi_option_target_point if voronoi_option_target_point else option_target_point

    def reached_target(self):
        # Hardcoded distance threshold to consider the target reached
        REACHED_THRESHOLD = 0.3  # Distance in meters

        if self.next_target_node:
            distance = sqrt(pow(self.position.x - self.next_target_node[0], 2) + pow(self.position.y - self.next_target_node[1], 2))
            self.node.get_logger().debug(f"{self.robot_name}: Distance to target: {distance}")
            if distance < REACHED_THRESHOLD:
                self.node.get_logger().info(f"{self.robot_name}: Reached target within {REACHED_THRESHOLD} meters.")
                return True
        return False


    def find_nearest_free_cell(self, map_array, y, x, search_radius=5):
        """
        Find the nearest free cell to the given (y, x) position within the search_radius.
        Returns the (y, x) tuple of the nearest free cell or None if not found.
        """
        for radius in range(1, search_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    new_y = y + dy
                    new_x = x + dx
                    if 0 <= new_y < map_array.shape[0] and 0 <= new_x < map_array.shape[1]:
                        if map_array[new_y][new_x] == 0:
                            return (new_y, new_x)
        return None

    def plan_path(self):
        # Use A* to plan path to next_target_node
        if self.node.map_data is None:
            self.node.get_logger().error(f"{self.robot_name}: No map data available for path planning.")
            return
        # Ensure next_target_node is not empty
        if not self.next_target_node:
            self.node.get_logger().error(f"{self.robot_name}: No target node available for path planning.")
            return
        # Compute required map bounds to include current position and target
        map_x_min = self.node.origin_x
        map_y_min = self.node.origin_y
        map_x_max = self.node.origin_x + self.node.map_width * self.node.resolution
        map_y_max = self.node.origin_y + self.node.map_height * self.node.resolution

        x_coords = [self.position.x, self.next_target_node[0], map_x_min, map_x_max]
        y_coords = [self.position.y, self.next_target_node[1], map_y_min, map_y_max]

        new_x_min = min(x_coords)
        new_x_max = max(x_coords)
        new_y_min = min(y_coords)
        new_y_max = max(y_coords)

        new_map_width = int((new_x_max - new_x_min) / self.node.resolution) + 1
        new_map_height = int((new_y_max - new_y_min) / self.node.resolution) + 1

        if self.node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.node.get_logger().debug(f"{self.robot_name}: Planning path with map bounds x:[{new_x_min}, {new_x_max}], y:[{new_y_min}, {new_y_max}], resolution={self.node.resolution}")

        # Create new map array initialized to -1 (unknown)
        new_map_array = np.full((new_map_height, new_map_width), -1, dtype=int)

        # Compute offsets
        offset_x = int((self.node.origin_x - new_x_min) / self.node.resolution)
        offset_y = int((self.node.origin_y - new_y_min) / self.node.resolution)

        # Copy known map data into new map array
        try:
            # Ensure indices do not exceed the new map size
            end_y = min(offset_y + self.node.map_height, new_map_height)
            end_x = min(offset_x + self.node.map_width, new_map_width)
            new_map_array[offset_y:end_y, offset_x:end_x] = self.node.map_data[:end_y - offset_y, :end_x - offset_x]
            self.node.get_logger().debug(f"{self.robot_name}: Map data copied to new map array with offsets x={offset_x}, y={offset_y}.")
        except Exception as e:
            self.node.get_logger().error(f"{self.robot_name}: Error copying map data: {e}")
            return

        # Convert current position and target to new map indices
        current_x_index = int((self.position.x - new_x_min) / self.node.resolution)
        current_y_index = int((self.position.y - new_y_min) / self.node.resolution)
        target_x_index = int((self.next_target_node[0] - new_x_min) / self.node.resolution)
        target_y_index = int((self.next_target_node[1] - new_y_min) / self.node.resolution)

        if self.node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.node.get_logger().debug(f"{self.robot_name}: Current index: ({current_y_index}, {current_x_index}), Target index: ({target_y_index}, {target_x_index})")

        map_array = new_map_array

        # Handle current position inside an obstacle
        if map_array[current_y_index][current_x_index] == 1:
            self.node.get_logger().warn(f"{self.robot_name}: Current position is inside an obstacle. Attempting to find the nearest free cell.")
            nearest_free = self.find_nearest_free_cell(map_array, current_y_index, current_x_index)
            if nearest_free:
                current_y_index, current_x_index = nearest_free
                self.node.get_logger().info(f"{self.robot_name}: Nearest free cell found at ({current_y_index}, {current_x_index}).")
            else:
                self.node.get_logger().error(f"{self.robot_name}: No free cell found near the current position.")
                return

        # Handle target position inside an obstacle
        if map_array[target_y_index][target_x_index] == 1:
            self.node.get_logger().warn(f"{self.robot_name}: Target position is inside an obstacle. Attempting to find the nearest free cell.")
            nearest_free = self.find_nearest_free_cell(map_array, target_y_index, target_x_index)
            if nearest_free:
                target_y_index, target_x_index = nearest_free
                self.node.get_logger().info(f"{self.robot_name}: Nearest free cell found for target at ({target_y_index}, {target_x_index}).")
            else:
                self.node.get_logger().error(f"{self.robot_name}: No free cell found near the target position.")
                return

        path_indices = astar(map_array, (current_y_index, current_x_index), (target_y_index, target_x_index), self.node.get_logger())

        if path_indices:
            self.path = [(x * self.node.resolution + new_x_min, y * self.node.resolution + new_y_min) for y, x in path_indices]
            self.path = bspline_planning(self.path, len(self.path) * 5, self.node.get_logger())
            self.path_index = 0
            self.node.get_logger().info(f"{self.robot_name}: Path planned with {len(self.path)} waypoints.")
            # Publish planned path
            self.publish_planned_path(self.path)
        else:
            self.node.get_logger().error(f"{self.robot_name}: Path planning failed using A*.")

    def is_path_obstructed(self):
        # Check if there are obstacles ahead along the path
        if self.laser_values is None or self.laser_msg is None:
            self.node.get_logger().debug(f"{self.robot_name}: No laser data available to check for path obstruction.")
            return False  # Cannot determine, assume no obstruction

        # Get the angle to the next point in the path
        if self.path_index >= len(self.path):
            self.node.get_logger().debug(f"{self.robot_name}: Path index {self.path_index} out of bounds.")
            return False  # No path ahead

        next_point = self.path[self.path_index]
        angle_to_next_point = math.atan2(next_point[1] - self.position.y, next_point[0] - self.position.x)
        angle_diff = angle_to_next_point - self.rotation

        # Normalize angle_diff to [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Determine indices in laser scan that correspond to the direction ahead
        angle_increment = self.laser_msg.angle_increment
        angle_min = self.laser_msg.angle_min

        index_center = int((angle_diff - angle_min) / angle_increment)
        angle_range = math.radians(20)  # +/- 20 degrees
        index_range = int(angle_range / angle_increment)

        # Check laser scan readings in the range ahead
        for i in range(index_center - index_range, index_center + index_range):
            if 0 <= i < len(self.laser_values):
                distance = self.laser_values[i]
                if distance < 0.5 and distance > 0.0:  # Obstacle within 0.5 meters
                    self.node.get_logger().warn(f"{self.robot_name}: Obstacle detected at distance {distance} meters within path ahead.")
                    return True
        return False

    def publish_planned_path(self, path):
        """Publish the planned path for this robot."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        poses = []
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Assuming no rotation
            poses.append(pose)
        path_msg.poses = poses
        self.planned_path_pub.publish(path_msg)
        self.node.get_logger().debug(f"{self.robot_name}: Planned path published with {len(path)} poses.")

    def publish_goal_marker(self, goal_x, goal_y):
        """Publish a single goal marker for this robot."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.node.get_clock().now().to_msg()
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
        self.goal_marker_pub.publish(marker)
        self.node.get_logger().debug(f"{self.robot_name}: Goal marker published at ({goal_x}, {goal_y}).")

    def create_goal_marker(self, goal_x, goal_y):
        """Create and store a goal marker for this robot."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = 'goals_history'
        marker.id = len(self.goals_history)
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
        self.goals_history.append(marker)
        self.node.get_logger().debug(f"{self.robot_name}: Goal marker added to history at ({goal_x}, {goal_y}).")

    def publish_goal_markers(self):
        """Publish all goal markers history for this robot."""
        marker_array = MarkerArray()
        marker_array.markers = self.goals_history
        self.goals_marker_pub.publish(marker_array)
        self.node.get_logger().debug(f"{self.robot_name}: Published {len(marker_array.markers)} goal markers.")

    def publish_robot_path(self):
        """Publish the traversed path followed by this robot."""
        self.traversed_path.append(Point(x=self.position.x, y=self.position.y, z=self.position.z))

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        poses = []
        for p in self.traversed_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position = p
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        path_msg.poses = poses
        self.robot_path_pub.publish(path_msg)
        self.node.get_logger().debug(f"{self.robot_name}: Traversed path published with {len(poses)} poses.")

class MultiRobotControl(Node):
    def __init__(self):
        super().__init__('multi_robot_control_node')
        self.robot_names = ['tb3_0', 
                            'tb3_1', 
                            'tb3_2', 
                            # 'tb3_3'
                            ]
        self.robots = {}
        self.other_robots_positions = {}  # Shared positions among robots
        self.lock = threading.Lock()  # Lock for thread-safe access

        # Initialize a single tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        # Allow time for the listener to fill its buffer
        time.sleep(1.0)

        for robot_name in self.robot_names:
            self.other_robots_positions[robot_name] = (0.0, 0.0)
            # Pass the shared tf_buffer to each Robot
            self.robots[robot_name] = Robot(robot_name, self, self.other_robots_positions, self.lock, self.tf_buffer)

        # Map subscription
        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.get_logger().info("MultiRobotControl node initialized and subscribed to /map.")

        # Partition publishers
        self.partition_publishers = {}
        for robot_name in self.robot_names:
            self.partition_publishers[robot_name] = self.create_publisher(GridCells, f'/{robot_name}/partition', 10)

        # Timer for publishing partitions
        self.partition_timer = self.create_timer(1.0, self.publish_partitions)  # Every 1 second

        # Target explored region rate
        self.target_explored_region_rate = TARGET_EXPLORE_RATE
        self.exploration_completed = False

        # Timer to track exploration duration
        self.exploration_start_time = None
        self.exploration_end_time = None

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Process map data
        processed_map = costmap(msg.data, self.map_width, self.map_height, self.resolution, self.get_logger())
        self.map_data = processed_map
        self.get_logger().debug("Map data received and processed.")

        # Initialize exploration start time if not already set
        if self.exploration_start_time is None:
            self.exploration_start_time = self.get_clock().now()
            self.get_logger().info("Exploration started. Timer initialized.")

        # Use map bounds as room bounds
        min_j = 0
        max_j = self.map_width
        min_i = 0
        max_i = self.map_height

        # Total cells within room
        total_cells = self.map_data.size

        # Explored cells: cells that are free (0) or occupied (1)
        explored_cells = np.count_nonzero(self.map_data >= 0)

        # Compute explored rate
        if total_cells > 0:
            current_explored_rate = explored_cells / total_cells
            self.get_logger().info(f"Explored Region Rate: {current_explored_rate*100:.2f}%")

            # Check if target explored rate is achieved
            if current_explored_rate >= self.target_explored_region_rate:
                if not self.exploration_completed:
                    self.exploration_completed = True
                    self.exploration_end_time = self.get_clock().now()
                    elapsed_time = (self.exploration_end_time - self.exploration_start_time).nanoseconds / 1e9
                    self.get_logger().info(f"Target explored region rate of {self.target_explored_region_rate*100:.2f}% achieved.")
                    self.get_logger().info(f"Exploration completed in {elapsed_time:.2f} seconds.")
                    self.handle_exploration_completion()
        else:
            self.get_logger().warn("Room boundaries result in zero cells.")

    def handle_exploration_completion(self):
        """
        Handle the event when the target explored region rate is achieved.
        Actions can include stopping all robots, logging information, etc.
        """
        self.get_logger().info("Exploration target reached. Initiating shutdown sequence.")

        # Stop all robots by publishing zero velocities
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        for robot_name, robot in self.robots.items():
            robot.cmd_vel_pub.publish(stop_twist)
            self.get_logger().info(f"Published stop command to {robot_name}.")

        # Optionally, perform other actions such as saving logs, shutting down the node, etc.
        # For example, shutting down after stopping the robots:
        self.get_logger().info("All robots stopped. Shutting down the node.")
        rclpy.shutdown()

    def publish_partitions(self):
        """Compute and publish the partitions for each robot."""
        if self.map_data is None:
            self.get_logger().warning("Map data not available for partitioning.")
            return

        # Get robot positions
        with self.lock:
            robot_positions = self.other_robots_positions.copy()

        # Initialize partitions
        partitions = {robot_name: [] for robot_name in self.robot_names}

        # Loop over map cells
        for i in range(self.map_height):
            for j in range(self.map_width):
                if self.map_data[i][j] == 0:  # Free space
                    # Compute cell position
                    x = self.origin_x + (j + 0.5) * self.resolution
                    y = self.origin_y + (i + 0.5) * self.resolution

                    # Find closest robot
                    min_distance = float('inf')
                    closest_robot = None

                    for robot_name, position in robot_positions.items():
                        dx = position[0] - x
                        dy = position[1] - y
                        distance = dx * dx + dy * dy  # Use squared distance

                        if distance < min_distance:
                            min_distance = distance
                            closest_robot = robot_name

                    # Add cell to partition
                    if closest_robot is not None:
                        partitions[closest_robot].append(Point(x=x, y=y, z=0.0))

        # For each robot, publish GridCells
        for robot_name, cells in partitions.items():
            grid_cells_msg = GridCells()
            grid_cells_msg.header.frame_id = 'map'
            grid_cells_msg.header.stamp = self.get_clock().now().to_msg()
            grid_cells_msg.cell_width = self.resolution
            grid_cells_msg.cell_height = self.resolution
            grid_cells_msg.cells = cells
            self.partition_publishers[robot_name].publish(grid_cells_msg)
            self.get_logger().debug(f"Published partition for {robot_name} with {len(cells)} cells.")

def main(args=None):
    rclpy.init(args=args)
    multi_robot_control = MultiRobotControl()
    try:
        rclpy.spin(multi_robot_control)
    except KeyboardInterrupt:
        multi_robot_control.get_logger().info("Shutting down MultiRobotControl node.")
    finally:
        # Clean up
        for robot in multi_robot_control.robots.values():
            robot.cmd_vel_pub.destroy()
            robot.planned_path_pub.destroy()
            robot.goal_marker_pub.destroy()
            robot.robot_path_pub.destroy()
            robot.goals_marker_pub.destroy()
        for publisher in multi_robot_control.partition_publishers.values():
            publisher.destroy()
        multi_robot_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
