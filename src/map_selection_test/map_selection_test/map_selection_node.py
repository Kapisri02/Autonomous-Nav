#!/usr/bin/env python3

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class MapSelectionNode(Node):

    def __init__(self):
        super().__init__('map_selection_node')

        self.declare_parameter('map_yaml', '')
        map_yaml = self.get_parameter('map_yaml').value

        if not map_yaml:
            self.get_logger().error(
                'map_yaml parameter was not provided.'
            )
            return

        self.map_yaml = os.path.abspath(map_yaml)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.publish_map()

    def publish_map(self):

        try:
            with open(self.map_yaml, 'r') as file:
                config = yaml.safe_load(file)
        except Exception as error:
            self.get_logger().error(
                f'Failed to read map YAML: {error}'
            )
            return

        image_name = config.get('image')

        if not image_name:
            self.get_logger().error(
                'The YAML file does not contain an image entry.'
            )
            return

        if os.path.isabs(image_name):
            image_path = image_name
        else:
            image_path = os.path.join(
                os.path.dirname(self.map_yaml),
                image_name
            )

        image_path = os.path.abspath(image_path)

        try:
            from PIL import Image
            image = Image.open(image_path).convert('L')
        except Exception as error:
            self.get_logger().error(
                f'Failed to load map image: {error}'
            )
            return

        width, height = image.size
        pixels = list(image.getdata())

        resolution = float(config['resolution'])
        origin = config.get(
            'origin',
            [0.0, 0.0, 0.0]
        )

        negate = int(config.get('negate', 0))
        occupied_thresh = float(
            config.get('occupied_thresh', 0.65)
        )
        free_thresh = float(
            config.get('free_thresh', 0.196)
        )

        occupancy_data = []

        for pixel in pixels:

            if negate:
                value = pixel / 255.0
            else:
                value = (255.0 - pixel) / 255.0

            if value > occupied_thresh:
                occupancy_data.append(100)

            elif value < free_thresh:
                occupancy_data.append(0)

            else:
                occupancy_data.append(-1)

        map_msg = OccupancyGrid()

        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height

        map_msg.info.origin.position.x = float(origin[0])
        map_msg.info.origin.position.y = float(origin[1])
        map_msg.info.origin.position.z = 0.0

        yaw = float(origin[2])

        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = math.sin(
            yaw / 2.0
        )
        map_msg.info.origin.orientation.w = math.cos(
            yaw / 2.0
        )

        map_msg.data = occupancy_data

        self.map_pub.publish(map_msg)

        self.get_logger().info(
            f'Published map: {width} x {height} cells'
        )

        self.get_logger().info(
            f'Resolution: {resolution} m/cell'
        )

        self.get_logger().info(
            f'Origin: {origin}'
        )

    def goal_callback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y

        q = msg.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'Selected point: X={x:.3f} m, Y={y:.3f} m'
        )

        self.get_logger().info(
            f'Yaw={math.degrees(yaw):.2f} degrees'
        )

        self.get_logger().info(
            '----------------------------------------'
        )

def main(args=None):

    rclpy.init(args=args)

    node = MapSelectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
