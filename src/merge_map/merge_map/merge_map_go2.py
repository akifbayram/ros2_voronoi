import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

def merge_maps(map_list):
    if not map_list:
        return None

    if len(map_list) == 1:  # If there's only one map, return it directly
        return map_list[0]

    merged_map = OccupancyGrid()
    merged_map.header = map_list[0].header
    merged_map.header.frame_id = 'map'

    # Determine the bounds of the merged map
    min_x = min(m.info.origin.position.x for m in map_list)
    min_y = min(m.info.origin.position.y for m in map_list)
    max_x = max(m.info.origin.position.x + (m.info.width * m.info.resolution) for m in map_list)
    max_y = max(m.info.origin.position.y + (m.info.height * m.info.resolution) for m in map_list)

    # Set merged map info
    merged_map.info.resolution = min(m.info.resolution for m in map_list)
    merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
    merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

    for map_msg in map_list:
        for y in range(map_msg.info.height):
            for x in range(map_msg.info.width):
                i = x + y * map_msg.info.width
                if map_msg.data[i] == -1:
                    continue
                merged_x = int(np.floor((map_msg.info.origin.position.x + x * map_msg.info.resolution - min_x) / merged_map.info.resolution))
                merged_y = int(np.floor((map_msg.info.origin.position.y + y * map_msg.info.resolution - min_y) / merged_map.info.resolution))
                merged_i = merged_x + merged_y * merged_map.info.width

                if merged_map.data[merged_i] == -1:
                    merged_map.data[merged_i] = map_msg.data[i]

    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Dictionary to store maps by robot namespace
        self.maps = {}

        # Subscribe to a configurable list of map topics
        robot_count = self.declare_parameter('robot_count', 1).get_parameter_value().integer_value
        for i in range(robot_count):
            robot_namespace = f'robot{i}'
            map_topic = f'/{robot_namespace}/map'
            self.create_subscription(OccupancyGrid, map_topic, self.map_callback(robot_namespace), 10)
            self.get_logger().info(f'Subscribed to {map_topic}')

    def map_callback(self, robot_namespace):
        def callback(msg):
            self.maps[robot_namespace] = msg
            if len(self.maps) > 0:  # Publish when at least one map is available
                merged_map = merge_maps(list(self.maps.values()))
                if merged_map:
                    self.publisher.publish(merged_map)
        return callback

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
   
