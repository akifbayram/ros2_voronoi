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
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

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

        self.action_client = ActionClient(self.node, NavigateToPose, f'/{self.robot_name}/navigate_to_pose')
        self.goal_handle = None

        self.traversed_path = []
        self.arrived = False

        self.timer_period = 1.0  # Increased timer period as Nav2 handles control
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
                    self.publish_goal_marker(self.next_target_node[0], self.next_target_node[1])
                    self.create_goal_marker(self.next_target_node[0], self.next_target_node[1])
                    self.send_goal_to_nav2(self.next_target_node)
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
                        self.publish_goal_marker(self.next_target_node[0], self.next_target_node[1])
                        self.create_goal_marker(self.next_target_node[0], self.next_target_node[1])
                        self.send_goal_to_nav2(self.next_target_node)
                else:
                    self.node.get_logger().error(f"{self.robot_name}: No finite option target points available from laser scan.")
        else:
            self.node.get_logger().warning(f"{self.robot_name}: Waiting for laser scan data to generate target points.")

    def send_goal_to_nav2(self, target):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f"{self.robot_name}: Nav2 action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward

        self.node.get_logger().info(f"{self.robot_name}: Sending navigation goal to Nav2 at ({target[0]}, {target[1]})")

        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.robot_name}: Goal rejected by Nav2.")
            return

        self.node.get_logger().info(f"{self.robot_name}: Goal accepted by Nav2, waiting for result...")
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can process feedback if needed
        self.node.get_logger().debug(f"{self.robot_name}: Received feedback from Nav2.")

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.node.get_logger().info(f"{self.robot_name}: Goal was aborted by Nav2.")
        elif status == 5:
            self.node.get_logger().info(f"{self.robot_name}: Goal was canceled.")
        elif status == 3:
            self.node.get_logger().info(f"{self.robot_name}: Goal succeeded!")
            self.arrived = True
            self.arr_info_node = True
            self.publish_goal_markers()
        else:
            self.node.get_logger().info(f"{self.robot_name}: Goal finished with status: {status}")

    def reached_target(self):
        if self.next_target_node:
            distance = math.hypot(
                self.position.x - self.next_target_node[0],
                self.position.y - self.next_target_node[1]
            )
            self.node.get_logger().debug(f"{self.robot_name}: Distance to target: {distance}")
            if distance < 0.5:
                return True
        return False

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

class MultiRobotControl(Node):
    def __init__(self):
        super().__init__('multi_robot_control_node')
        self.robot_names = ['robot0']  # Add more robot names as needed
        self.robots = {}
        self.other_robots_positions = {}
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        time.sleep(1.0)  # Allow some time for TF buffer to fill

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

        # Assuming Nav2 is handling costmap, so we can store the map data if needed
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.get_logger().debug("Map data received.")

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
