import math
import struct
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray


class PedestrianPositionFusionNode(Node):
    """Estimate real-time pedestrian positions from YOLO detections + lidar points.

    Input topics:
      - /yolo/detections: vision_msgs/Detection2DArray from panoramic YOLO detector
      - /lidar_points: sensor_msgs/PointCloud2 from Hesai lidar

    Output topic:
      - /pedestrian/detected_poses: PoseArray of untracked pedestrian positions
    """

    def __init__(self):
        super().__init__('pedestrian_position_fusion_node')

        # Topic parameters
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('lidar_topic', '/lidar_points')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'hesai_lidar')

        # Panoramic image geometry
        self.declare_parameter('image_width', 3840)
        self.declare_parameter('yaw_offset', 0.0)

        # Static transform from hesai_lidar to base_link
        self.declare_parameter('lidar_to_base_x', 0.34058)
        self.declare_parameter('lidar_to_base_y', 0.0)
        self.declare_parameter('lidar_to_base_z', 0.3465)
        self.declare_parameter('lidar_to_base_yaw', math.pi / 2.0)

        # Fusion parameters
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('min_z', -0.8)
        self.declare_parameter('max_z', 2.2)
        self.declare_parameter('angular_gate_min', 0.035)
        self.declare_parameter('angular_gate_padding', 0.030)
        self.declare_parameter('nearest_cluster_width', 0.8)
        self.declare_parameter('min_points_per_detection', 3)

        self.detections_topic = self.get_parameter('detections_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.image_width = int(self.get_parameter('image_width').value)
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.angular_gate_min = float(self.get_parameter('angular_gate_min').value)
        self.angular_gate_padding = float(self.get_parameter('angular_gate_padding').value)
        self.nearest_cluster_width = float(self.get_parameter('nearest_cluster_width').value)
        self.min_points_per_detection = int(self.get_parameter('min_points_per_detection').value)

        self.latest_cloud: Optional[List[Tuple[float, float, float]]] = None
        self.latest_cloud_stamp: Optional[float] = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        normal_qos = QoSProfile(depth=10)

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.cloud_callback,
            sensor_qos,
        )

        self.det_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detection_callback,
            normal_qos,
        )

        self.detected_pose_pub = self.create_publisher(
            PoseArray,
            '/pedestrian/detected_poses',
            10,
        )

        self.get_logger().info(
            'PedestrianPositionFusionNode started: '
            f'{self.detections_topic} + {self.lidar_topic} -> /pedestrian/detected_poses in {self.output_frame}'
        )

    def ros_time_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def cloud_callback(self, msg: PointCloud2):
        points = self.read_xyz_points(msg)

        if msg.header.frame_id and msg.header.frame_id != self.output_frame:
            if msg.header.frame_id == self.lidar_frame:
                points = self.transform_lidar_to_base(points)
            else:
                self.get_logger().warn(
                    f'PointCloud frame is {msg.header.frame_id}, expected {self.output_frame} or {self.lidar_frame}. '
                    'Using raw coordinates. Set output_frame/lidar_frame or transform parameters if needed.',
                    throttle_duration_sec=5.0,
                )

        self.latest_cloud = points
        self.latest_cloud_stamp = self.ros_time_to_sec(msg.header.stamp)

    def read_xyz_points(self, msg: PointCloud2) -> List[Tuple[float, float, float]]:
        offsets = {field.name: field.offset for field in msg.fields}

        if not all(name in offsets for name in ('x', 'y', 'z')):
            self.get_logger().warn(
                'PointCloud2 has no x/y/z fields.',
                throttle_duration_sec=2.0,
            )
            return []

        ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
        point_step = msg.point_step
        data = msg.data

        points: List[Tuple[float, float, float]] = []
        unpack_float = struct.Struct('<f').unpack_from

        for i in range(msg.width * msg.height):
            base = i * point_step

            try:
                x = unpack_float(data, base + ox)[0]
                y = unpack_float(data, base + oy)[0]
                z = unpack_float(data, base + oz)[0]
            except struct.error:
                break

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            r = math.hypot(x, y)

            if self.min_range <= r <= self.max_range and self.min_z <= z <= self.max_z:
                points.append((x, y, z))

        return points

    def transform_lidar_to_base(
        self,
        points: List[Tuple[float, float, float]],
    ) -> List[Tuple[float, float, float]]:
        tx = float(self.get_parameter('lidar_to_base_x').value)
        ty = float(self.get_parameter('lidar_to_base_y').value)
        tz = float(self.get_parameter('lidar_to_base_z').value)
        yaw = float(self.get_parameter('lidar_to_base_yaw').value)

        c = math.cos(yaw)
        s = math.sin(yaw)

        out = []

        for x, y, z in points:
            xb = c * x - s * y + tx
            yb = s * x + c * y + ty
            zb = z + tz
            out.append((xb, yb, zb))

        return out

    def detection_callback(self, msg: Detection2DArray):
        if self.latest_cloud is None or len(self.latest_cloud) == 0:
            self.get_logger().warn(
                'No lidar points received yet. Cannot estimate pedestrian positions.',
                throttle_duration_sec=2.0,
            )
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = self.output_frame
        pose_array.header.stamp = msg.header.stamp

        for det in msg.detections:
            pos = self.estimate_position_from_detection(det)

            if pos is not None:
                mx, my, mz, myaw = pos
                pose = Pose()
                pose.position.x = mx
                pose.position.y = my
                pose.position.z = max(0.0, mz)
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = math.sin(myaw / 2.0)
                pose.orientation.w = math.cos(myaw / 2.0)
                pose_array.poses.append(pose)

        self.detected_pose_pub.publish(pose_array)

    def estimate_position_from_detection(
        self,
        detection,
    ) -> Optional[Tuple[float, float, float, float]]:
        x_center = float(detection.bbox.center.position.x)
        bbox_width = float(detection.bbox.size_x) if detection.bbox.size_x > 0.0 else 80.0

        yaw = -((x_center - (self.image_width / 2.0)) / float(self.image_width)) * 2.0 * math.pi
        yaw += self.yaw_offset
        yaw = self.wrap_angle(yaw)

        angular_gate = max(
            self.angular_gate_min,
            (bbox_width / float(self.image_width)) * math.pi + self.angular_gate_padding,
        )

        candidates = []

        for x, y, z in self.latest_cloud:
            pyaw = math.atan2(y, x)

            if abs(self.angle_diff(pyaw, yaw)) <= angular_gate:
                r = math.hypot(x, y)
                candidates.append((r, x, y, z))

        if len(candidates) < self.min_points_per_detection:
            return None

        candidates.sort(key=lambda p: p[0])
        nearest_range = candidates[0][0]

        cluster = [
            p for p in candidates
            if p[0] <= nearest_range + self.nearest_cluster_width
        ]

        if len(cluster) < self.min_points_per_detection:
            cluster = candidates[:self.min_points_per_detection]

        rs = sorted(p[0] for p in cluster)
        zs = sorted(p[3] for p in cluster)
        mid = len(cluster) // 2

        r_mid = rs[mid]
        z_mid = zs[mid]

        x_on_ray = r_mid * math.cos(yaw)
        y_on_ray = r_mid * math.sin(yaw)

        return (x_on_ray, y_on_ray, z_mid, yaw)

    @staticmethod
    def wrap_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def angle_diff(a: float, b: float) -> float:
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianPositionFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
