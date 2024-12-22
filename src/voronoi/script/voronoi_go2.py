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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Parameters for navigation
LOOKAHEAD_DISTANCE = 0.25
SPEED = 0.15
EXPANSION_SIZE = 0  # For obstacle expansion in the map
LAMBDA_VALUE = 1
MAX_RANGE = 10.0  # Maximum finite range to cap LaserScan data

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

#############################################
# Heuristic
#############################################
def heuristic(a, b, logger=None):
    """
    Euclidean distance can work fine, but 
    we handle consistent diagonal cost via the same.
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    h = math.sqrt(dx**2 + dy**2)
    if logger:
        logger.debug(f"Heuristic between {a} and {b}: {h}")
    return h

#############################################
# A* Path Finding
#############################################
def astar(array, start, goal, logger=None):
    """
    A* with 8-direction moves. Diagonal steps have cost sqrt(2). 
    """
    neighbors = [
        (0, 1, 1.0),   # up
        (0, -1, 1.0),  # down
        (1, 0, 1.0),   # right
        (-1, 0, 1.0),  # left
        (1, 1, math.sqrt(2)),   
        (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (-1, -1, math.sqrt(2))
    ]

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
            data.reverse()
            if logger:
                logger.info(f"A* Path found: {data}")
            return data

        close_set.add(current)

        for dx, dy, step_cost in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < array.shape[0] and 0 <= neighbor[1] < array.shape[1]):
                continue
            if array[neighbor[0]][neighbor[1]] == 1:
                continue

            tentative_g_score = gscore[current] + step_cost

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal, logger)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    if logger:
        logger.warning("A* failed to find a path")
    return False

#############################################
# B-spline Planning
#############################################
def bspline_planning(array, sn, logger=None):
    """
    We create a B-spline from the path array. If that fails, we fallback.
    """
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 3  # cubic spline
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        # Extend knots to maintain continuity at endpoints
        x_list = list(x_tup)
        y_list = list(y_tup)
        xl = x.tolist()
        yl = y.tolist()
        x_list[1] = xl + [xl[-1]] * (N + 1)
        y_list[1] = yl + [yl[-1]] * (N + 1)
        x_tup = tuple(x_list)
        y_tup = tuple(y_list)

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

#############################################
# Pure Pursuit
#############################################
def pure_pursuit(current_x, current_y, current_heading, path, index, lookahead_distance, speed, logger=None):
    """
    Instead of fully stopping on large steering, reduce linear speed proportionally.
    """
    closest_point = None
    v = speed

    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if distance > lookahead_distance:
            closest_point = (x, y)
            index = i
            break

    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        index = len(path) - 1

    desired_steering_angle = target_heading - current_heading
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi

    max_steer = math.pi / 3.0  # 60 degrees
    if abs(desired_steering_angle) > max_steer:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * max_steer
        v = speed * 0.2
    else:
        steer_ratio = abs(desired_steering_angle) / max_steer
        v = speed * (1 - 0.8 * steer_ratio)

    if logger:
        logger.debug(f"Pure Pursuit: v={v:.3f}, omega={desired_steering_angle:.3f}, index={index}")

    return v, desired_steering_angle, index

#############################################
# Costmap
#############################################
def costmap(data, width, height, resolution, logger=None):
    """
    Improved obstacle inflation using BFS within EXPANSION_SIZE.
    """
    data = np.array(data).reshape(height, width)
    if logger:
        logger.debug("Costmap: Initial costmap processed")

    occupied_indices = np.argwhere(data == 100)
    from collections import deque
    queue = deque()
    visited = np.full((height, width), False, dtype=bool)
    
    for oy, ox in occupied_indices:
        queue.append((oy, ox, 0))
        visited[oy, ox] = True

    while queue:
        cy, cx, dist = queue.popleft()
        if dist > EXPANSION_SIZE:
            continue
        data[cy, cx] = 100
        for ny, nx in [(cy-1, cx), (cy+1, cx), (cy, cx-1), (cy, cx+1),
                       (cy-1, cx-1), (cy-1, cx+1), (cy+1, cx-1), (cy+1, cx+1)]:
            if 0 <= ny < height and 0 <= nx < width and not visited[ny, nx]:
                visited[ny, nx] = True
                queue.append((ny, nx, dist + 1))

    if logger:
        logger.debug(f"Costmap: Obstacles inflated by {EXPANSION_SIZE} cells using BFS.")
    return data

class Robot:
    def __init__(self, robot_name, node, other_robots_positions, lock, tf_buffer):
        self.robot_name = robot_name
        self.node = node
        self.position = Point()
        self.rotation = 0.0
        self.arr_info_node = True
        self.laser_crashed_value = False
        self.next_target_node = []
        self.record_info_node = []
        self.communication_max_range = 6.0
        self.laser_msg_range_max = None
        self.laser_values = None
        self.laser_msg = None
        self.record_info_node = [(self.position.x, self.position.y)]

        self.other_robots_positions = other_robots_positions
        self.lock = lock
        self.tf_buffer = tf_buffer

        qos_profile_scan = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.cmd_vel_pub = node.create_publisher(Twist, '/' + self.robot_name + '/cmd_vel', 10)
        self.laser_sub = node.create_subscription(LaserScan, '/' + self.robot_name + '/scan', self.laser_callback, qos_profile_scan)
        self.planned_path_pub = node.create_publisher(Path, '/' + self.robot_name + '/planned_path', 10)
        self.goal_marker_pub = node.create_publisher(Marker, '/' + self.robot_name + '/goal_current', 10)
        self.robot_path_pub = node.create_publisher(Path, '/' + self.robot_name + '/robot_path', 10)
        self.goals_marker_pub = node.create_publisher(MarkerArray, '/' + self.robot_name + '/goals_history', 10)
        self.goals_history = []

        self.path = []
        self.path_index = 0
        self.traversed_path = []
        self.arrived = False

        self.timer_period = 0.1
        self.timer = node.create_timer(self.timer_period, self.timer_callback)

        self.node.get_logger().info(f"{self.robot_name}: Robot initialized.")

    def laser_callback(self, msg):
        self.laser_msg = msg
        self.laser_msg_range_max = msg.range_max
        self.laser_values = msg.ranges
        self.node.get_logger().debug(f"{self.robot_name}: Laser scan received with {len(msg.ranges)} ranges.")

    def update_pose(self):
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
            return False

        self.position.x = trans.transform.translation.x
        self.position.y = trans.transform.translation.y
        self.position.z = trans.transform.translation.z
        q = trans.transform.rotation
        self.rotation = euler_from_quaternion_func(q.x, q.y, q.z, q.w)
        self.node.get_logger().debug(f"{self.robot_name}: Updated pose to x={self.position.x}, y={self.position.y}, yaw={self.rotation}")

        with self.lock:
            self.other_robots_positions[self.robot_name] = (self.position.x, self.position.y)
        return True

    def timer_callback(self):
        if not self.update_pose():
            self.node.get_logger().warn(f"{self.robot_name}: Skipping cycle due to pose update failure.")
            return

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
        if self.laser_values:
            obstacle_detected = any(d < 0.2 for d in self.laser_values if d > 0.0)
            if obstacle_detected:
                self.laser_crashed_value = True
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.node.get_logger().warn(f"{self.robot_name}: Obstacle detected <0.2m. Stopping and waiting for new target.")
                self.arr_info_node = True
        else:
            self.node.get_logger().debug(f"{self.robot_name}: No laser data available.")

    def select_target(self):
        option_target_point = []
        if self.laser_values is not None and self.laser_msg is not None:
            angle_increment = self.laser_msg.angle_increment
            angle_min = self.laser_msg.angle_min
            for i, distance in enumerate(self.laser_values):
                if distance == float('inf') or distance >= self.laser_msg_range_max:
                    angle = i * angle_increment + angle_min + self.rotation
                    # Cap the distance to MAX_RANGE to prevent infinity
                    effective_range = min(self.laser_msg_range_max, MAX_RANGE)
                    target_x = self.position.x + effective_range * math.cos(angle)
                    target_y = self.position.y + effective_range * math.sin(angle)
                    option_target_point.append([target_x, target_y])
            self.node.get_logger().debug(f"{self.robot_name}: {len(option_target_point)} potential target points generated from laser scan.")
            voronoi_points = self.voronoi_select_point(option_target_point)
            if voronoi_points:
                self.next_target_node = self.get_min_Omega_distance_point(voronoi_points)
                if self.next_target_node is not None:
                    self.record_info_node.append(self.next_target_node)
                    self.node.get_logger().info(f"{self.robot_name}: New target position selected at ({self.next_target_node[0]}, {self.next_target_node[1]})")
                    self.arr_info_node = False
                    self.arrived = False
                    self.path = []
                    self.path_index = 0
                    self.publish_goal_marker(self.next_target_node[0], self.next_target_node[1])
                    self.create_goal_marker(self.next_target_node[0], self.next_target_node[1])
                else:
                    self.node.get_logger().error(f"{self.robot_name}: Failed to select a valid target point.")
            else:
                self.node.get_logger().warning(f"{self.robot_name}: No valid Voronoi point found. Using all option points.")
                finite_option_points = [p for p in option_target_point if np.isfinite(p[0]) and np.isfinite(p[1])]
                if finite_option_points:
                    self.next_target_node = self.get_min_Omega_distance_point(finite_option_points)
                    if self.next_target_node is not None:
                        self.record_info_node.append(self.next_target_node)
                        self.node.get_logger().info(f"{self.robot_name}: New target position selected at ({self.next_target_node[0]}, {self.next_target_node[1]})")
                        self.arr_info_node = False
                        self.arrived = False
                        self.path = []
                        self.path_index = 0
                        self.publish_goal_marker(self.next_target_node[0], self.next_target_node[1])
                        self.create_goal_marker(self.next_target_node[0], self.next_target_node[1])
                else:
                    self.node.get_logger().error(f"{self.robot_name}: No finite option target points available from laser scan.")
        else:
            self.node.get_logger().warning(f"{self.robot_name}: Waiting for laser scan data to generate target points.")

    def follow_path(self):
        if self.path_index >= len(self.path):
            self.node.get_logger().debug(f"{self.robot_name}: Reached end of path.")
            return

        if self.is_path_obstructed():
            self.node.get_logger().info(f"{self.robot_name}: Path obstructed, re-planning.")
            self.arr_info_node = True
            self.next_target_node = []
            self.path = []
            self.path_index = 0
            return

        twist = Twist()
        v, omega, self.path_index = pure_pursuit(
            self.position.x, self.position.y, self.rotation,
            self.path, self.path_index, LOOKAHEAD_DISTANCE, SPEED, self.node.get_logger()
        )
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)
        self.node.get_logger().debug(f"{self.robot_name}: Published cmd_vel with linear.x={v:.3f}, angular.z={omega:.3f}")

        if self.path_index >= len(self.path) - 1:
            self.arr_info_node = True
            self.arrived = True
            self.path = []
            self.node.get_logger().info(f"{self.robot_name}: Completed path following.")

    def handle_reached_target(self):
        self.arr_info_node = True
        self.arrived = True
        self.node.get_logger().info(f"{self.robot_name}: Reached target at {self.next_target_node}.")
        self.publish_goal_markers()

    def get_min_Omega_distance_point(self, option_target_points):
        min_Omega = float('inf')
        best_point = None
        if len(self.record_info_node) >= 1:
            last_target = self.record_info_node[-1]
        else:
            last_target = (self.position.x, self.position.y)
        for point in option_target_points:
            # Skip points with infinite coordinates
            if not np.isfinite(point[0]) or not np.isfinite(point[1]):
                self.node.get_logger().warn(f"{self.robot_name}: Skipping invalid point with inf coordinates: {point}")
                continue
            d_ik = math.hypot(point[0] - last_target[0], point[1] - last_target[1])
            phi_ik = math.hypot(point[0] - self.position.x, point[1] - self.position.y)
            Omega = LAMBDA_VALUE * d_ik + (1 - LAMBDA_VALUE) * phi_ik
            self.node.get_logger().debug(f"{self.robot_name}: Omega for point ({point[0]}, {point[1]}): {Omega}")
            if Omega < min_Omega:
                min_Omega = Omega
                best_point = point
        if best_point is None:
            self.node.get_logger().error(f"{self.robot_name}: No valid point found using Min-Omega heuristic.")
            # Safeguard: Select a default finite point or skip target selection
            finite_points = [p for p in option_target_points if np.isfinite(p[0]) and np.isfinite(p[1])]
            if finite_points:
                best_point = random.choice(finite_points)
                self.node.get_logger().info(f"{self.robot_name}: Fallback to a finite random target point: {best_point}")
            else:
                self.node.get_logger().error(f"{self.robot_name}: No finite points available to select a target.")
                return None
        return best_point

    def voronoi_select_point(self, option_target_point):
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
        if self.next_target_node:
            distance = sqrt(pow(self.position.x - self.next_target_node[0], 2) + pow(self.position.y - self.next_target_node[1], 2))
            self.node.get_logger().debug(f"{self.robot_name}: Distance to target: {distance}")
            if distance < 0.5:
                return True
        return False

    def find_nearest_free_cell(self, map_array, y, x, search_radius=5):
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
        if self.node.map_data is None:
            self.node.get_logger().error(f"{self.robot_name}: No map data available for path planning.")
            return
        if not self.next_target_node:
            self.node.get_logger().error(f"{self.robot_name}: No target node available for path planning.")
            return

        # Check for infinite coordinates
        if not np.isfinite(self.next_target_node[0]) or not np.isfinite(self.next_target_node[1]):
            self.node.get_logger().error(f"{self.robot_name}: Target position contains infinite values: {self.next_target_node}")
            return

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

        new_map_array = np.full((new_map_height, new_map_width), -1, dtype=int)

        offset_x = int((self.node.origin_x - new_x_min) / self.node.resolution)
        offset_y = int((self.node.origin_y - new_y_min) / self.node.resolution)

        try:
            end_y = min(offset_y + self.node.map_height, new_map_height)
            end_x = min(offset_x + self.node.map_width, new_map_width)
            new_map_array[offset_y:end_y, offset_x:end_x] = self.node.map_data[:end_y - offset_y, :end_x - offset_x]
            self.node.get_logger().debug(f"{self.robot_name}: Map data copied to new map array with offsets x={offset_x}, y={offset_y}.")
        except Exception as e:
            self.node.get_logger().error(f"{self.robot_name}: Error copying map data: {e}")
            return

        current_x_index = int((self.position.x - new_x_min) / self.node.resolution)
        current_y_index = int((self.position.y - new_y_min) / self.node.resolution)
        target_x_index = int((self.next_target_node[0] - new_x_min) / self.node.resolution)
        target_y_index = int((self.next_target_node[1] - new_y_min) / self.node.resolution)

        if self.node.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.node.get_logger().debug(f"{self.robot_name}: Current index: ({current_y_index}, {current_x_index}), Target index: ({target_y_index}, {target_x_index})")

        map_array = new_map_array

        if map_array[current_y_index][current_x_index] == 1:
            self.node.get_logger().warn(f"{self.robot_name}: Current position is inside an obstacle. Attempting to find the nearest free cell.")
            nearest_free = self.find_nearest_free_cell(map_array, current_y_index, current_x_index)
            if nearest_free:
                current_y_index, current_x_index = nearest_free
                self.node.get_logger().info(f"{self.robot_name}: Nearest free cell found at ({current_y_index}, {current_x_index}).")
            else:
                self.node.get_logger().error(f"{self.robot_name}: No free cell found near the current position.")
                return

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
            self.path = [(x * self.node.resolution + new_x_min, y * self.node.resolution + new_y_min) for (y, x) in path_indices]
            self.path = bspline_planning(self.path, len(self.path) * 5, self.node.get_logger())
            self.path_index = 0
            self.node.get_logger().info(f"{self.robot_name}: Path planned with {len(self.path)} waypoints.")
            self.publish_planned_path(self.path)
        else:
            self.node.get_logger().error(f"{self.robot_name}: Path planning failed using A*.")

    def is_path_obstructed(self):
        if self.laser_values is None or self.laser_msg is None:
            self.node.get_logger().debug(f"{self.robot_name}: No laser data available to check for path obstruction.")
            return False

        if self.path_index >= len(self.path):
            self.node.get_logger().debug(f"{self.robot_name}: Path index {self.path_index} out of bounds.")
            return False

        next_point = self.path[self.path_index]
        angle_to_next_point = math.atan2(next_point[1] - self.position.y, next_point[0] - self.position.x)
        angle_diff = angle_to_next_point - self.rotation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        angle_increment = self.laser_msg.angle_increment
        angle_min = self.laser_msg.angle_min
        index_center = int((angle_diff - angle_min) / angle_increment)
        angle_range = math.radians(20)
        index_range = int(angle_range / angle_increment)

        for i in range(index_center - index_range, index_center + index_range):
            if 0 <= i < len(self.laser_values):
                distance = self.laser_values[i]
                if 0.0 < distance < 0.5:
                    self.node.get_logger().warn(f"{self.robot_name}: Obstacle detected at distance {distance}m in path direction.")
                    return True
        return False

    def publish_planned_path(self, path):
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
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        path_msg.poses = poses
        self.planned_path_pub.publish(path_msg)
        self.node.get_logger().debug(f"{self.robot_name}: Planned path published with {len(path)} poses.")

    def publish_goal_marker(self, goal_x, goal_y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
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
        marker_array = MarkerArray()
        marker_array.markers = self.goals_history
        self.goals_marker_pub.publish(marker_array)
        self.node.get_logger().debug(f"{self.robot_name}: Published {len(marker_array.markers)} goal markers.")

    def publish_robot_path(self):
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
        self.robot_names = ['robot0']
        self.robots = {}
        self.other_robots_positions = {}
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        time.sleep(1.0)

        for robot_name in self.robot_names:
            self.other_robots_positions[robot_name] = (0.0, 0.0)
            self.robots[robot_name] = Robot(robot_name, self, self.other_robots_positions, self.lock, self.tf_buffer)

        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.get_logger().info("MultiRobotControl node initialized and subscribed to /map.")

        self.partition_publishers = {}
        for robot_name in self.robot_names:
            self.partition_publishers[robot_name] = self.create_publisher(GridCells, f'/{robot_name}/partition', 10)

        # Timer for publishing partitions
        self.partition_timer = self.create_timer(1.0, self.publish_partitions)

    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        processed_map = costmap(msg.data, self.map_width, self.map_height, self.resolution, self.get_logger())
        self.map_data = processed_map
        self.get_logger().debug("Map data received and processed.")

    def publish_partitions(self):
        if self.map_data is None:
            self.get_logger().warning("Map data not available for partitioning.")
            return

        with self.lock:
            robot_positions = self.other_robots_positions.copy()

        partitions = {robot_name: [] for robot_name in self.robot_names}

        for i in range(self.map_height):
            for j in range(self.map_width):
                if self.map_data[i][j] == 0:
                    x = self.origin_x + (j + 0.5) * self.resolution
                    y = self.origin_y + (i + 0.5) * self.resolution

                    min_distance = float('inf')
                    closest_robot = None

                    for robot_name, position in robot_positions.items():
                        dx = position[0] - x
                        dy = position[1] - y
                        distance = dx * dx + dy * dy

                        if distance < min_distance:
                            min_distance = distance
                            closest_robot = robot_name

                    if closest_robot is not None:
                        partitions[closest_robot].append(Point(x=x, y=y, z=0.0))

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
