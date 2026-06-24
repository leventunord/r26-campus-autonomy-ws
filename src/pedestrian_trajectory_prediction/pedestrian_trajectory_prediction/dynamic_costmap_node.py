#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DynamicCostmapNode(Node):
    def __init__(self):
        super().__init__('pedestrian_dynamic_costmap_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('current_topic', '/pedestrian/current_poses')
        self.declare_parameter('predicted_topic', '/pedestrian/predicted_poses')
        self.declare_parameter('output_topic', '/pedestrian/dynamic_costmap')
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('current_radius', 0.60)
        self.declare_parameter('predicted_radius', 0.40)
        self.declare_parameter('current_cost', 100)
        self.declare_parameter('predicted_cost', 80)
        self.declare_parameter('publish_rate', 5.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.current_topic = self.get_parameter('current_topic').value
        self.predicted_topic = self.get_parameter('predicted_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.map_frame = self.get_parameter('map_frame').value

        self.current_radius = float(self.get_parameter('current_radius').value)
        self.predicted_radius = float(self.get_parameter('predicted_radius').value)
        self.current_cost = int(self.get_parameter('current_cost').value)
        self.predicted_cost = int(self.get_parameter('predicted_cost').value)

        self.latest_map = None
        self.latest_current = None
        self.latest_predicted = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        normal_qos = QoSProfile(depth=10)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos
        )

        self.current_sub = self.create_subscription(
            PoseArray,
            self.current_topic,
            self.current_callback,
            normal_qos
        )

        self.predicted_sub = self.create_subscription(
            PoseArray,
            self.predicted_topic,
            self.predicted_callback,
            normal_qos
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            map_qos
        )

        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_dynamic_costmap)

        self.get_logger().info(
            f'Dynamic costmap node started: '
            f'{self.current_topic} + {self.predicted_topic} -> {self.output_topic}'
        )

    def map_callback(self, msg):
        self.latest_map = msg

    def current_callback(self, msg):
        self.latest_current = msg

    def predicted_callback(self, msg):
        self.latest_predicted = msg

    def transform_points_to_map(self, pose_array):
        if pose_array is None:
            return []

        source_frame = pose_array.header.frame_id
        if source_frame == '':
            source_frame = 'base_link'

        points = []

        if source_frame == self.map_frame:
            for pose in pose_array.poses:
                points.append((pose.position.x, pose.position.y))
            return points

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Cannot transform {source_frame} to {self.map_frame}: {ex}'
            )
            return []

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = yaw_from_quaternion(tf.transform.rotation)

        c = math.cos(yaw)
        s = math.sin(yaw)

        for pose in pose_array.poses:
            x = pose.position.x
            y = pose.position.y

            x_map = tx + c * x - s * y
            y_map = ty + s * x + c * y

            points.append((x_map, y_map))

        return points

    def world_to_grid(self, x, y, info):
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        origin_yaw = yaw_from_quaternion(info.origin.orientation)

        dx = x - origin_x
        dy = y - origin_y

        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)

        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        mx = int(local_x / res)
        my = int(local_y / res)

        return mx, my

    def draw_disc(self, data, info, x, y, radius, cost):
        width = info.width
        height = info.height
        res = info.resolution

        cx, cy = self.world_to_grid(x, y, info)
        r_cells = int(math.ceil(radius / res))

        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy > r_cells * r_cells:
                    continue

                mx = cx + dx
                my = cy + dy

                if mx < 0 or mx >= width or my < 0 or my >= height:
                    continue

                idx = my * width + mx
                data[idx] = max(data[idx], cost)

    def publish_dynamic_costmap(self):
        if self.latest_map is None:
            return

        info = self.latest_map.info
        width = info.width
        height = info.height

        dynamic_map = OccupancyGrid()
        dynamic_map.header.stamp = self.get_clock().now().to_msg()
        dynamic_map.header.frame_id = self.map_frame
        dynamic_map.info = info

        # -1 means unknown / transparent-like background in RViz.
        data = [-1] * (width * height)

        current_points = self.transform_points_to_map(self.latest_current)
        predicted_points = self.transform_points_to_map(self.latest_predicted)

        for x, y in predicted_points:
            self.draw_disc(
                data,
                info,
                x,
                y,
                self.predicted_radius,
                self.predicted_cost
            )

        for x, y in current_points:
            self.draw_disc(
                data,
                info,
                x,
                y,
                self.current_radius,
                self.current_cost
            )

        dynamic_map.data = data
        self.pub.publish(dynamic_map)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCostmapNode()
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
