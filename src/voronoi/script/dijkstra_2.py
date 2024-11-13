#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial import Voronoi
import math
import tf2_ros
import tf_transformations
from sklearn.cluster import DBSCAN

class VoronoiExplorer(Node):
    def __init__(self):
        super().__init__('voronoi_explorer')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 0),
                ('num_robots', 1),
                ('sensing_range', 1.3),
                ('communication_range', 5.0),
                ('lambda_param', 0.8),
                ('robot_name', 'tb3_0'),
                ('min_frontier_size', 1),
                ('frontier_cluster_size', 0.1),
            ]
        )
        
        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value
        self.sensing_range = self.get_parameter('sensing_range').value
        self.comm_range = self.get_parameter('communication_range').value
        self.lambda_param = self.get_parameter('lambda_param').value
        self.robot_name = self.get_parameter('robot_name').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.frontier_cluster_size = self.get_parameter('frontier_cluster_size').value
        
        # Robot state
        self.position = None
        self.orientation = None
        self.neighbors = {}  # Dict to store neighbor positions
        self.frontier_points = []
        self.info_nodes = []
        self.current_target = None
        self.map_data = None
        self.robot_ready = False
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, f'/{self.robot_name}/visualization', 10)
        self.frontier_pub = self.create_publisher(
            MarkerArray, f'/{self.robot_name}/frontiers', 10)
            
        # Subscribers
        self.create_subscription(
            Odometry, f'/{self.robot_name}/odom', 
            self.odom_callback, sensor_qos)
        self.create_subscription(
            LaserScan, f'/{self.robot_name}/scan',
            self.scan_callback, sensor_qos)
        self.create_subscription(
            OccupancyGrid, '/map',
            self.map_callback, 10)
            
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for main exploration loop
        self.create_timer(0.1, self.exploration_loop)
        
        self.get_logger().info(f'Voronoi Explorer initialized for {self.robot_name}')

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        
        if not self.robot_ready:
            self.robot_ready = True
            self.info_nodes.append((self.position.x, self.position.y))
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.robot_name}/base_link'
        t.transform.translation.x = self.position.x
        t.transform.translation.y = self.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.orientation
        
        self.tf_broadcaster.sendTransform(t)
        self.broadcast_position()

    def scan_callback(self, msg):
        """Process laser scan data to find frontiers"""
        if not self.robot_ready or self.position is None:
            return
            
        # Convert laser scan to points and find frontiers
        ranges = np.array(msg.ranges)
        
        # Create angles array with same length as ranges
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid measurements
        valid_idx = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        valid_ranges = ranges[valid_idx]
        valid_angles = angles[valid_idx]
        
        # Convert to cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # Transform to map frame
        points = np.column_stack((x, y))
        
        # Transform points to map frame
        points_map = np.zeros_like(points)
        for i, (px, py) in enumerate(points):
            # Apply robot's position and orientation
            quat = [self.orientation.x, self.orientation.y, 
                   self.orientation.z, self.orientation.w]
            yaw = tf_transformations.euler_from_quaternion(quat)[2]
            
            # Rotate point
            c = np.cos(yaw)
            s = np.sin(yaw)
            points_map[i, 0] = px * c - py * s + self.position.x
            points_map[i, 1] = px * s + py * c + self.position.y
            
        self.find_frontiers(points_map)

    def find_frontiers(self, scan_points):
        """Find frontier points from laser scan"""
        if len(scan_points) < self.min_frontier_size:
            return
            
        # Create a mask of points that are potential frontiers
        frontiers = []
        
        # Group nearby points using DBSCAN
        clustering = DBSCAN(eps=self.frontier_cluster_size, 
                          min_samples=self.min_frontier_size).fit(scan_points)
        
        # Process each cluster
        for cluster_id in np.unique(clustering.labels_):
            if cluster_id == -1:  # Skip noise points
                continue
                
            # Get points in this cluster
            cluster_points = scan_points[clustering.labels_ == cluster_id]
            
            # Calculate cluster centroid
            centroid = np.mean(cluster_points, axis=0)
            
            # Check if this could be a frontier
            if self.is_potential_frontier(centroid):
                frontiers.append(centroid)
        
        self.frontier_points = frontiers
        self.publish_frontiers()

    def is_potential_frontier(self, point):
        """Check if a point could be a frontier between explored and unexplored space"""
        if self.map_data is None:
            return True
            
        # Convert point to map coordinates
        mx = int((point[0] - self.map_data.info.origin.position.x) / 
                self.map_data.info.resolution)
        my = int((point[1] - self.map_data.info.origin.position.y) / 
                self.map_data.info.resolution)
        
        # Check if point is within map bounds
        if (mx < 0 or mx >= self.map_data.info.width or 
            my < 0 or my >= self.map_data.info.height):
            return False
        
        # Get an area around the point
        radius = 3  # cells
        min_x = max(0, mx - radius)
        max_x = min(self.map_data.info.width - 1, mx + radius)
        min_y = max(0, my - radius)
        max_y = min(self.map_data.info.height - 1, my + radius)
        
        # Check cells in the area
        has_unknown = False
        has_free = False
        
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                cell = self.map_data.data[y * self.map_data.info.width + x]
                if cell == -1:  # Unknown
                    has_unknown = True
                elif cell == 0:  # Free
                    has_free = True
                    
                if has_unknown and has_free:
                    return True
                    
        return False

    def is_unexplored(self, x, y):
        """Check if a point is in unexplored territory"""
        if self.map_data is None:
            return True
            
        # Convert world coordinates to map coordinates
        mx = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check if point is within map bounds
        if mx < 0 or mx >= self.map_data.info.width or my < 0 or my >= self.map_data.info.height:
            return False
            
        # Get cell value
        cell = self.map_data.data[my * self.map_data.info.width + mx]
        return cell == -1  # -1 indicates unexplored

    def map_callback(self, msg):
        """Store map data for planning"""
        self.map_data = msg

    def broadcast_position(self):
        """Broadcast position to other robots"""
        # In a real implementation, this would use ROS2 services or custom messages
        # For now, we'll just simulate communication with neighbors
        pass

    def compute_voronoi_partition(self):
        """Compute Voronoi partition based on current robot positions"""
        if not self.neighbors:
            return None
            
        # Collect all robot positions including self
        positions = [(self.position.x, self.position.y)]
        for pos in self.neighbors.values():
            positions.append((pos.x, pos.y))
            
        # Compute Voronoi diagram
        vor = Voronoi(np.array(positions))
        return vor

    def point_in_voronoi_cell(self, point, vor, cell_index):
        """Check if point is in the robot's Voronoi cell"""
        point_arr = np.array([point[0], point[1]])
        center = vor.points[cell_index]
        
        # Check if point is closer to this cell's center than any other
        for i, other_center in enumerate(vor.points):
            if i != cell_index:
                if np.linalg.norm(point_arr - center) > np.linalg.norm(point_arr - other_center):
                    return False
        return True

    def compute_utility(self, frontier_point):
        """Compute utility function for a frontier point"""
        # Distance from current position to frontier
        d_ik = math.sqrt(
            (frontier_point[0] - self.position.x) ** 2 +
            (frontier_point[1] - self.position.y) ** 2
        )
                     
        # Distance from frontier to initial position
        phi_ik = math.sqrt(
            (frontier_point[0] - self.info_nodes[0][0]) ** 2 +
            (frontier_point[1] - self.info_nodes[0][1]) ** 2
        )
                     
        return self.lambda_param * d_ik + (1 - self.lambda_param) * phi_ik

    def select_next_frontier(self):
        """Select next frontier point with minimum utility"""
        if not self.frontier_points:
            return None
            
        # Get valid frontiers within Voronoi cell
        vor = self.compute_voronoi_partition()
        valid_frontiers = self.frontier_points if vor is None else [
            f for f in self.frontier_points 
            if self.point_in_voronoi_cell(f, vor, 0)
        ]
        
        if not valid_frontiers:
            return None
            
        # Find frontier with minimum utility
        utilities = [self.compute_utility(f) for f in valid_frontiers]
        return valid_frontiers[np.argmin(utilities)]

    def target_reached(self):
        """Check if current target has been reached"""
        if self.current_target is None or self.position is None:
            return False
            
        distance = math.sqrt(
            (self.current_target[0] - self.position.x) ** 2 +
            (self.current_target[1] - self.position.y) ** 2
        )
        return distance < 0.2  # 20cm threshold

    def move_to_target(self):
        """Generate velocity commands to move to current target"""
        if self.current_target is None:
            return
            
        # Calculate target relative to robot's position
        dx = self.current_target[0] - self.position.x
        dy = self.current_target[1] - self.position.y
        
        # Get current orientation as euler angles
        orientation_q = self.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - yaw
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Generate Twist message
        cmd = Twist()
        
        # Angular velocity
        cmd.angular.z = 0.5 * angle_diff
        
        # Linear velocity (reduced when turning)
        distance = math.sqrt(dx*dx + dy*dy)
        cmd.linear.x = 0.2 * distance * (1 - abs(angle_diff) / math.pi)
        
        # Apply velocity limits
        cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
        
        self.cmd_vel_pub.publish(cmd)

    def exploration_loop(self):
        """Main exploration loop"""
        if not self.robot_ready:
            return
            
        # If no target or target reached, select new frontier
        if self.current_target is None or self.target_reached():
            if self.target_reached():
                self.deploy_info_node()
                
            self.current_target = self.select_next_frontier()
            if self.current_target is None:
                # No frontiers left, exploration complete
                self.get_logger().info('No frontiers available')
                # Stop the robot
                self.cmd_vel_pub.publish(Twist())
                return
                
        # Move towards target
        self.move_to_target()

    def deploy_info_node(self):
        """Deploy new information node at current position"""
        self.info_nodes.append((self.position.x, self.position.y))
        self.publish_visualization()

    def publish_visualization(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # Add markers for info nodes
        for i, node in enumerate(self.info_nodes):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = node[0]
            marker.pose.position.y = node[1]
            marker.pose.position.z = 0.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def publish_frontiers(self):
        """Publish frontier visualization markers"""
        marker_array = MarkerArray()
        
        # Add markers for frontier points
        for i, frontier in enumerate(self.frontier_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier[0]
            marker.pose.position.y = frontier[1]
            marker.pose.position.z = 0.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            marker_array.markers.append(marker)
            
            # Add a text marker with distance
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = i + 1000  # Offset to avoid ID conflicts
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = frontier[0]
            text_marker.pose.position.y = frontier[1]
            text_marker.pose.position.z = 0.5
            
            # Calculate distance to frontier
            dist = math.sqrt(
                (frontier[0] - self.position.x) ** 2 +
                (frontier[1] - self.position.y) ** 2
            )
            text_marker.text = f"{dist:.1f}m"
            
            text_marker.scale.z = 0.2  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.8
            marker_array.markers.append(text_marker)
        
        self.frontier_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = VoronoiExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()