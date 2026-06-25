import math
import struct
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class PedestrianPositionFusionNode(Node):
    """Two-pass fusion of YOLO person directions + lidar for robust pedestrian positioning.

    Input topics:
      - /yolo/person_directions: MarkerArray from person_direction_node (Yaw in orientation)
      - /lidar_points: PointCloud2 from Hesai lidar

    Output topic:
      - /pedestrian/detected_poses: PoseArray of untracked pedestrian positions

    Algorithm:
      Pass 1 — Wide-gate candidate collection with Z-layer filtering
      Pass 2 — Density clustering + Yaw refinement from lidar cluster centroid + NMS
    """

    def __init__(self):
        super().__init__('pedestrian_position_fusion_node')

        # ── Topic parameters ──
        self.declare_parameter('directions_topic', '/yolo/person_directions')
        self.declare_parameter('lidar_topic', '/lidar_points')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'hesai_lidar')

        # ── Static transform: hesai_lidar → base_link ──
        self.declare_parameter('lidar_to_base_x', 0.34058)
        self.declare_parameter('lidar_to_base_y', 0.0)
        self.declare_parameter('lidar_to_base_z', 0.3465)
        self.declare_parameter('lidar_to_base_yaw', math.pi / 2.0)

        # ── Height filtering (strict, ignore ground entirely) ──
        self.declare_parameter('min_z', 0.2)
        self.declare_parameter('max_z', 1.8)

        # ── Angular gating ──
        self.declare_parameter('wide_gate', 0.20)           # 搜索扇区半宽 (~11.5度)，确保能罩住偏离的行人

        # ── Density clustering ──
        self.declare_parameter('cluster_width', 0.8)  # metres
        self.declare_parameter('min_cluster_points', 2)

        # ── Output NMS ──
        self.declare_parameter('nms_distance', 0.6)  # metres

        # ── Read all parameters ──
        self.directions_topic = self.get_parameter('directions_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value

        self.min_range = float(self.get_parameter('min_range').value) if self.has_parameter('min_range') else 0.3
        self.max_range = float(self.get_parameter('max_range').value) if self.has_parameter('max_range') else 15.0
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.wide_gate = float(self.get_parameter('wide_gate').value)
        self.cluster_width = float(self.get_parameter('cluster_width').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.nms_distance = float(self.get_parameter('nms_distance').value)

        # Time synchronization buffer
        self.cloud_buffer = deque(maxlen=20)

        # ── Cached lidar data ──
        self.latest_cloud: Optional[List[Tuple[float, float, float]]] = None

        # ── Subscriptions ──
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

        self.dir_sub = self.create_subscription(
            MarkerArray,
            self.directions_topic,
            self.direction_callback,
            normal_qos,
        )

        # ── Publishers ──
        self.detected_pose_pub = self.create_publisher(
            PoseArray,
            '/pedestrian/detected_poses',
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pedestrian/position_markers',
            10,
        )

        self.get_logger().info(
            f'PositionFusionNode started: '
            f'{self.directions_topic} + {self.lidar_topic} → /pedestrian/detected_poses'
        )

    # ───────────────────────── Point cloud handling ─────────────────────────

    def cloud_callback(self, msg: PointCloud2):
        """Parse, filter, and transform incoming point cloud."""
        points = self._read_xyz_points(msg)

        # Transform from hesai_lidar to base_link if needed
        if msg.header.frame_id and msg.header.frame_id != self.output_frame:
            if msg.header.frame_id == self.lidar_frame:
                points = self._transform_lidar_to_base(points)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cloud_buffer.append((stamp_sec, points))

    def _read_xyz_points(self, msg: PointCloud2) -> List[Tuple[float, float, float]]:
        offsets = {field.name: field.offset for field in msg.fields}
        if not all(name in offsets for name in ('x', 'y', 'z')):
            return []

        ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
        point_step = msg.point_step
        data = msg.data
        unpack = struct.Struct('<f').unpack_from

        points: List[Tuple[float, float, float]] = []
        for i in range(msg.width * msg.height):
            base = i * point_step
            try:
                x = unpack(data, base + ox)[0]
                y = unpack(data, base + oy)[0]
                z = unpack(data, base + oz)[0]
            except struct.error:
                break

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            r = math.hypot(x, y)
            if self.min_range <= r <= self.max_range:
                points.append((x, y, z))

        return points

    def _transform_lidar_to_base(
        self, points: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        tx = float(self.get_parameter('lidar_to_base_x').value)
        ty = float(self.get_parameter('lidar_to_base_y').value)
        tz = float(self.get_parameter('lidar_to_base_z').value)
        yaw = float(self.get_parameter('lidar_to_base_yaw').value)
        c, s = math.cos(yaw), math.sin(yaw)

        out = []
        for x, y, z in points:
            xb = c * x - s * y + tx
            yb = s * x + c * y + ty
            zb = z + tz
            out.append((xb, yb, zb))
        return out

    # ────────────────────── Direction (MarkerArray) handling ──────────────────────

    def direction_callback(self, msg: MarkerArray):
        """Main fusion entry point, triggered by each person_directions message."""
        if not msg.markers:
            return

        # Find the first valid timestamp in the markers
        stamp_sec = 0.0
        for m in msg.markers:
            if m.header.stamp.sec != 0 or m.header.stamp.nanosec != 0:
                stamp_sec = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                break
        
        if stamp_sec == 0.0:
            return

        # Find the closest cloud in time
        best_cloud = None
        min_diff = float('inf')
        for c_stamp, c_points in self.cloud_buffer:
            diff = abs(c_stamp - stamp_sec)
            if diff < min_diff:
                min_diff = diff
                best_cloud = c_points

        if best_cloud is None or min_diff > 0.2:
            self.get_logger().warn(
                f'No matching cloud for YOLO detection (time diff: {min_diff:.3f}s). Skipping.',
                throttle_duration_sec=2.0
            )
            return

        self.latest_cloud = best_cloud

        yaw_list: List[float] = []
        for marker in msg.markers:
            self.get_logger().debug(
                f'[DEBUG] Marker ns={marker.ns} action={marker.action} id={marker.id}'
            )
            # Skip the red "robot_front" reference arrow (id 999 or ns='robot_front')
            if marker.ns == 'robot_front':
                continue
            if marker.action == marker.DELETEALL:
                continue
            # Also strictly require it to be a person direction
            if marker.ns != 'person_direction':
                continue
            # Extract yaw from quaternion (z-axis rotation only)
            qz = marker.pose.orientation.z
            qw = marker.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            yaw_list.append(yaw)

        self.get_logger().info(
            f'[DEBUG] Directions received: {len(msg.markers)} markers, {len(yaw_list)} yaws extracted',
            throttle_duration_sec=1.0
        )

        if not yaw_list:
            # No person detected — publish empty PoseArray to clear downstream
            empty = PoseArray()
            empty.header.frame_id = self.output_frame
            empty.header.stamp = msg.markers[0].header.stamp if msg.markers else self.get_clock().now().to_msg()
            self.detected_pose_pub.publish(empty)
            return

        # Run two-pass fusion for each yaw
        raw_results: List[Tuple[float, float, float, float, int]] = []
        for yaw in yaw_list:
            result = self._fuse_single_direction(yaw)
            if result is not None:
                raw_results.append(result)

        # NMS: de-duplicate results that are too close to each other
        final_results = self._nms(raw_results)

        # Build and publish PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = self.output_frame
        # Use the timestamp from the first marker
        for m in msg.markers:
            if m.ns != 'robot_front' and m.action != m.DELETEALL:
                pose_array.header.stamp = m.header.stamp
                break

        for mx, my, mz, refined_yaw, _ in final_results:
            pose = Pose()
            pose.position.x = mx
            pose.position.y = my
            pose.position.z = max(0.0, mz)
            pose.orientation.z = math.sin(refined_yaw / 2.0)
            pose.orientation.w = math.cos(refined_yaw / 2.0)
            pose_array.poses.append(pose)

        self.detected_pose_pub.publish(pose_array)

        # Publish sphere markers for visualization
        marker_array = MarkerArray()
        # Clear previous markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'pedestrian_pos'
        marker_array.markers.append(delete_marker)

        for idx, (mx, my, mz, refined_yaw, _) in enumerate(final_results):
            sphere = Marker()
            sphere.header = pose_array.header
            sphere.ns = 'pedestrian_pos'
            sphere.id = idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = mx
            sphere.pose.position.y = my
            sphere.pose.position.z = max(0.0, mz)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4
            sphere.color.a = 0.85
            sphere.color.r = 0.2
            sphere.color.g = 0.4
            sphere.color.b = 1.0
            marker_array.markers.append(sphere)

        self.marker_pub.publish(marker_array)

    # ────────────────────── Two-pass fusion core ──────────────────────

    def _fuse_single_direction(
        self, yaw_guess: float
    ) -> Optional[Tuple[float, float, float, float, int]]:
        """Fuse a single YOLO direction with lidar points.

        Returns (x, y, z, refined_yaw, num_points) or None.
        """

        # ── Pass 1: Wide-gate candidate collection ──
        candidates: List[Tuple[float, float, float, float]] = []  # (r, x, y, z)

        for x, y, z in self.latest_cloud:
            if z < self.min_z or z > self.max_z:
                continue

            pt_yaw = math.atan2(y, x)
            if abs(self._angle_diff(pt_yaw, yaw_guess)) > self.wide_gate:
                continue

            r = math.hypot(x, y)
            candidates.append((r, x, y, z))

        if len(candidates) < self.min_cluster_points:
            return None  # Too few points — not a person

        # ── Pass 2: Density clustering + Yaw refinement ──

        # Sort by horizontal range
        candidates.sort(key=lambda p: p[0])

        # Find the best semantic cluster
        cluster = self._find_dense_cluster(candidates, yaw_guess)
        if cluster is None:
            return None

        # Refined yaw: angular median of cluster points
        angles = sorted(math.atan2(p[2], p[1]) for p in cluster)
        refined_yaw = angles[len(angles) // 2]

        # Range: median of cluster distances
        ranges = sorted(p[0] for p in cluster)
        r_mid = ranges[len(ranges) // 2]

        # Z: median
        zs = sorted(p[3] for p in cluster)
        z_mid = zs[len(zs) // 2]

        # Final position on the refined ray
        fx = r_mid * math.cos(refined_yaw)
        fy = r_mid * math.sin(refined_yaw)

        return (fx, fy, z_mid, refined_yaw, len(cluster))

    def _find_dense_cluster(
        self, sorted_candidates: List[Tuple[float, float, float, float]], yaw_guess: float
    ) -> Optional[List[Tuple[float, float, float, float]]]:
        """Find the nearest valid cluster using 2D Euclidean clustering and geometric filtering."""
        n = len(sorted_candidates)
        if n < self.min_cluster_points:
            return None

        # 1. 2D Euclidean Clustering (Connected Components)
        clusters = []
        visited = [False] * n
        
        for i in range(n):
            if visited[i]: 
                continue
            
            comp = [sorted_candidates[i]]
            q = [i]
            visited[i] = True
            
            while q:
                curr = q.pop(0)
                cx, cy = sorted_candidates[curr][1], sorted_candidates[curr][2]
                
                for j in range(n):
                    if not visited[j]:
                        jx, jy = sorted_candidates[j][1], sorted_candidates[j][2]
                        # Group points within 0.2m of each other (prevents merging person with wall)
                        if math.hypot(cx - jx, cy - jy) < 0.2:
                            visited[j] = True
                            q.append(j)
                            comp.append(sorted_candidates[j])
            
            clusters.append(comp)

        # 2. Extract valid clusters (minimum points)
        valid_clusters = []
        for comp in clusters:
            if len(comp) < self.min_cluster_points:
                continue
            valid_clusters.append(comp)

        if not valid_clusters:
            return None

        # 3. Choose the cluster that best matches the YOLO semantic ray
        def cluster_score(c):
            cx = sum(p[1] for p in c) / len(c)
            cy = sum(p[2] for p in c) / len(c)
            cyaw = math.atan2(cy, cx)
            angle_err = abs(self._angle_diff(cyaw, yaw_guess))
            dist = math.hypot(cx, cy)
            
            # Score heavily penalizes angular mismatch (1.0 rad error = +5.0 score)
            # Distance is a tie-breaker (1m = +0.1 score)
            return angle_err * 5.0 + dist * 0.1

        valid_clusters.sort(key=cluster_score)
        return valid_clusters[0]

    # ────────────────────── NMS de-duplication ──────────────────────

    def _nms(
        self, results: List[Tuple[float, float, float, float, int]]
    ) -> List[Tuple[float, float, float, float, int]]:
        if len(results) <= 1:
            return results

        # 优先保留距离更近的检测结果。因为相机透视原理，同一个方向不可能有两个人同时被看到（前面的会挡住后面的）。
        results.sort(key=lambda r: math.hypot(r[0], r[1]))

        keep: List[Tuple[float, float, float, float, int]] = []
        for candidate in results:
            cx, cy = candidate[0], candidate[1]
            cyaw = math.atan2(cy, cx)
            is_duplicate = False

            for kept in keep:
                kyaw = math.atan2(kept[1], kept[0])
                dist = math.hypot(cx - kept[0], cy - kept[1])
                angle_diff = abs(self._angle_diff(cyaw, kyaw))

                # 空间上太近，或者在同一个扇形角度内，都认为是重复检测（或者误检到了背后的墙）
                if dist < self.nms_distance or angle_diff < self.wide_gate:
                    is_duplicate = True
                    break

            if not is_duplicate:
                keep.append(candidate)

        return keep

    # ────────────────────── Utilities ──────────────────────

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
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
