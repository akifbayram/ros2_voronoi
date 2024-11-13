#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetEntityState, SetEntityState_Request
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf_transformations
import tf2_ros
from tf2_ros import TransformException
from math import atan2, sqrt, pow
import numpy as np
import math
import random

# Import your TensorFlow and Keras dependencies as needed
import tensorflow
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input
from keras.optimizers import Adam, get
import keras.backend as K

class InfoGetter:
    def __init__(self):
        self.msg = None
        self.event = threading.Event()

    def callback(self, msg):
        self.msg = msg
        self.event.set()

    def get_msg(self, timeout=None):
        self.event.wait(timeout)
        return self.msg

class GameState(Node):
    def __init__(self):
        super().__init__('voronoi_node')

        self.lambda_value = 1
        self.move_cmd = Twist()

        self.point_pub = self.create_publisher(Point, 'point_topic', 10)

        # tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        self.robot_name = ['tb3_0', 'tb3_1', 'tb3_2']
        self.position = {'tb3_0': Point(), 'tb3_1': Point(), 'tb3_2': Point()}
        self.rotation = {'tb3_0': 0.0, 'tb3_1': 0.0, 'tb3_2': 0.0}

        self.record_info_node = {'tb3_0': [], 'tb3_1': [], 'tb3_2': []}
        self.next_target_node = {'tb3_0': [], 'tb3_1': [], 'tb3_2': []}
        self.arr_info_node = {'tb3_0': False, 'tb3_1': False, 'tb3_2': False}
        self.thread_is_alive = {'tb3_0': False, 'tb3_1': False, 'tb3_2': False}
        self.done = False
        self.communication_max_range = 6
        self.detect_info_node = False
        self.laser_crashed_value = {'tb3_0': False, 'tb3_1': False, 'tb3_2': False}
        self.map_merge_data = OccupancyGrid()
        self.map1_free_num = 6251.0
        self.target_explored_region_rate = 0.8
        self.rate = self.create_rate(10)
        self.crash_indicator = 0
        self.state_num = 28
        self.action_num = 2
        self.observation_space = np.empty(self.state_num)
        self.action_space = np.empty(self.action_num)
        self.laser_reward = 0
        self.model_index = 10

        # Initialize service clients
        self.reset_simulation_client = self.create_client(Empty, '/gazebo/reset_simulation')
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Wait for the services to be available
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/reset_simulation service...')
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state service...')

        # Create action clients for each robot
        self.move_base_clients = {}
        for name in self.robot_name:
            self.move_base_clients[name] = ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')

        # Start the map data handling thread
        threading.Thread(target=self.map_data_handle, daemon=True).start()

    def get_init_info_node(self):
        for name in self.robot_name:
            laser_ig = InfoGetter()
            self.create_subscription(LaserScan, f'/{name}/scan', laser_ig.callback, 10)
            laser_msg = laser_ig.get_msg()
            if laser_msg is None:
                self.get_logger().warn(f'No laser data received for {name}')
                continue
            self.laser_msg_range_max = laser_msg.range_max
            laser_values = laser_msg.ranges
            option_target_point = []
            for j in range(len(laser_values)):
                if laser_values[j] == float('inf'):
                    theta = j * laser_msg.angle_increment + (math.pi / 2 - laser_msg.angle_max)
                    option_target_point_x = self.position[name].x + (self.laser_msg_range_max * math.sin(theta))
                    option_target_point_y = self.position[name].y - (self.laser_msg_range_max * math.cos(theta))
                    option_target_point.append([option_target_point_x, option_target_point_y])
            option_target_point = self.voronoi_select_point(name, option_target_point)
            if option_target_point:
                self.next_target_node[name] = random.choice(option_target_point)
                self.record_info_node[name].append(self.next_target_node[name])
            else:
                self.get_logger().warn(f'No valid target points found for {name}')

    def reset(self):
        self.laser_crashed_value = {'tb3_0': False, 'tb3_1': False, 'tb3_2': False}
        self.rate.sleep()
        self.crash_indicator = 0

        # Define initial positions
        initial_positions = {
            'tb3_0': (-3.0, -1.5),
            'tb3_1': (-3.0, 0.0),
            'tb3_2': (-3.0, 1.5)
        }

        for name in self.robot_name:
            state_msg = ModelState()
            state_msg.model_name = name
            state_msg.pose.position.x = initial_positions[name][0]
            state_msg.pose.position.y = initial_positions[name][1]
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.w = 1.0

            # Call the service to set model state
            request = SetEntityState_Request()
            request.state = state_msg

            future = self.set_entity_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Successfully reset {name}')
            else:
                self.get_logger().error(f'Failed to reset {name}: {future.exception()}')

        # Get positions of robots
        for name in self.robot_name:
            self.set_tf(name)
            time.sleep(0.1)
            self.position[name], self.rotation[name] = self.get_odom(name)

        # Stop robots
        for name in self.robot_name:
            pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.rate.sleep()
            pub.publish(self.move_cmd)
            self.rate.sleep()

        # Get initial target nodes
        self.get_init_info_node()
        for name in self.robot_name:
            if self.next_target_node[name]:
                self.get_logger().info(f"{name}: target position {self.next_target_node[name][0]}, {self.next_target_node[name][1]}")

        initial_state = np.ones(self.state_num)
        initial_state[self.state_num - 1] = 0
        initial_state[self.state_num - 2] = 0
        initial_state[self.state_num - 3] = 0
        initial_state[self.state_num - 4] = 0

        self.rate.sleep()
        initial_state = [initial_state] * 3
        return initial_state

    def set_tf(self, robot_name):
        # No equivalent in ROS2; tf2 handles transforms differently
        pass

    def get_odom(self, robot_name):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(f'{robot_name}/{self.odom_frame}', f'{robot_name}/{self.base_frame}', now, timeout=rclpy.duration.Duration(seconds=1.0))
            position = trans.transform.translation
            rotation_quat = trans.transform.rotation
            rotation = tf_transformations.euler_from_quaternion([
                rotation_quat.x,
                rotation_quat.y,
                rotation_quat.z,
                rotation_quat.w
            ])
            return Point(x=position.x, y=position.y, z=position.z), rotation[2]
        except TransformException as ex:
            self.get_logger().warn(f'TF Exception: {ex}')
            return Point(), 0.0

    def line_distance(self, x0, y0, x1, y1):
        return sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)

    def num_robot_site(self, robot_name):
        option_site = []
        for name in self.robot_name:
            if robot_name != name:
                distance = self.line_distance(self.position[robot_name].x, self.position[robot_name].y, self.position[name].x, self.position[name].y)
                if distance <= self.communication_max_range:
                    option_site.append(name)
        return option_site

    def voronoi_select_point(self, robot_name, option_target_point):
        option_site = self.num_robot_site(robot_name)
        voronoi_option_target_point = []
        if len(option_site) == 0:
            return option_target_point
        for point in option_target_point:
            j = 0
            for name in option_site:
                distance = self.line_distance(self.position[name].x, self.position[name].y, point[0], point[1])
                if distance > self.laser_msg_range_max:
                    j += 1
                if j == len(option_site):
                    voronoi_option_target_point.append(point)
        return voronoi_option_target_point

    def get_min_Omega_distance_point(self, robot_name, option_target_point):
        min_Omega = float('inf')
        min_point = None
        for point in option_target_point:
            Omega = self.lambda_value * self.line_distance(self.record_info_node[robot_name][0][0], self.record_info_node[robot_name][0][1], point[0], point[1]) + \
                    (1 - self.lambda_value) * self.line_distance(self.position[robot_name].x, self.position[robot_name].y, point[0], point[1])
            if Omega < min_Omega:
                min_Omega = Omega
                min_point = point
        return min_point

    def get_record_next_info_node(self, robot_name, option_target_point):
        if self.arr_info_node[robot_name]:
            option_target_point = self.voronoi_select_point(robot_name, option_target_point)
            if len(option_target_point) == 0:
                return False
            else:
                min_point = self.get_min_Omega_distance_point(robot_name, option_target_point)
                if min_point:
                    self.next_target_node[robot_name] = min_point
                    self.get_logger().info(f"{robot_name}: target position {self.next_target_node[robot_name][0]}, {self.next_target_node[robot_name][1]}")
                    self.record_info_node[robot_name].append(self.next_target_node[robot_name])
                    self.arr_info_node[robot_name] = False
                    return True
        return False

    def map_data_handle(self):
        start_time = time.time()
        while rclpy.ok():
            map_ig = InfoGetter()
            self.create_subscription(OccupancyGrid, '/map_merge/map', map_ig.callback, 10)
            map_msg = map_ig.get_msg()
            if map_msg is None:
                self.get_logger().warn('No map data received')
                continue
            self.map_merge_data.data = map_msg.data

            free_num = sum(1 for data in self.map_merge_data.data if data == 0)
            explored_region_rate = free_num / self.map1_free_num
            self.get_logger().info(f'Explored region rate: {explored_region_rate}')
            time.sleep(1)
            if explored_region_rate >= self.target_explored_region_rate:
                self.done = True
                break
            if explored_region_rate > 0.9:
                end_time = time.time()
                self.get_logger().info(f"Time taken for exploration: {end_time - start_time} seconds when lambda is {self.lambda_value}")
                break

    def game_step(self, robot_name):
        if self.thread_is_alive[robot_name]:
            return

        move_base_client = self.move_base_clients[robot_name]
        if not move_base_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Move base action server not available for {robot_name}')
            return

        laser_ig = InfoGetter()
        self.create_subscription(LaserScan, f'/{robot_name}/scan', laser_ig.callback, 10)

        self.position[robot_name], self.rotation[robot_name] = self.get_odom(robot_name)
        turtlebot_x_previous = self.position[robot_name].x
        turtlebot_y_previous = self.position[robot_name].y

        if self.line_distance(self.position[robot_name].x, self.position[robot_name].y, self.next_target_node[robot_name][0], self.next_target_node[robot_name][1]) < 0.5:
            self.arr_info_node[robot_name] = True
        else:
            self.arr_info_node[robot_name] = False

        option_target_point = []
        if self.arr_info_node[robot_name]:
            laser_msg = laser_ig.get_msg()
            if laser_msg:
                laser_values = laser_msg.ranges
                for i, distance in enumerate(laser_values):
                    if distance == float('inf'):
                        theta = (i + 1) * laser_msg.angle_increment
                        option_target_point_x = turtlebot_x_previous + (self.laser_msg_range_max * math.sin(theta))
                        option_target_point_y = turtlebot_y_previous - (self.laser_msg_range_max * math.cos(theta))
                        option_target_point.append([option_target_point_x, option_target_point_y])
                if option_target_point:
                    self.get_record_next_info_node(robot_name, option_target_point)
                else:
                    self.rate.sleep()
                    self.laser_crashed_value[robot_name] = True

        laser_msg = laser_ig.get_msg()
        if laser_msg:
            for distance in laser_msg.ranges:
                if distance < 0.3:
                    self.laser_crashed_value[robot_name] = True
                if distance < 0.2:
                    self.laser_crashed_value[robot_name] = True
                    self.reset()
                    return

        target_x = self.next_target_node[robot_name][0]
        target_y = self.next_target_node[robot_name][1]

        self.thread_is_alive[robot_name] = True
        threading.Thread(target=self.send_goal_async, args=(move_base_client, robot_name, target_x, target_y), daemon=True).start()

    def send_goal_async(self, move_base_client, robot_name, target_x, target_y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        angle_to_goal = atan2(target_y - self.position[robot_name].y, target_x - self.position[robot_name].x)
        quat = tf_transformations.quaternion_from_euler(0, 0, angle_to_goal)
        goal_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.get_logger().info(f"{robot_name}: Sending goal to position ({target_x}, {target_y})")
        send_goal_future = move_base_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        if send_goal_future.result():
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().info(f'{robot_name}: Goal rejected')
                self.thread_is_alive[robot_name] = False
                return

            self.get_logger().info(f'{robot_name}: Goal accepted')
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'{robot_name}: Goal succeeded!')
            else:
                self.get_logger().info(f'{robot_name}: Goal failed with status: {result.status}')
        else:
            self.get_logger().error(f'{robot_name}: Failed to send goal')
        self.thread_is_alive[robot_name] = False

def main(args=None):
    rclpy.init(args=args)

    # TensorFlow session initialization if needed
    sess = tensorflow.compat.v1.Session()
    K.set_session(sess)

    game_state = GameState()
    game_state.get_logger().info("GameState initialized!")

    game_state.reset()
    game_state.get_logger().info("GameState reset!")

    try:
        while rclpy.ok():
            for k in range(3):
                game_state.game_step(game_state.robot_name[k])
            rclpy.spin_once(game_state, timeout_sec=0.1)
    except KeyboardInterrupt:
        game_state.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        game_state.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
