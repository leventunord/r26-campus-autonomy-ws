import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs


@dataclass
class Track:
    track_id: int
    history: deque = field(default_factory=lambda: deque(maxlen=30))  # (time_sec, x, y, z)
    last_update: float = 0.0
    missed: int = 0

    # Direction from YOLO detection
    yaw: float = 0.0

    # Direction estimated from recent trajectory tangent
    motion_yaw: float = 0.0

    # Valid only when estimated speed >= min_direction_speed
    direction_valid: bool = False
    speed: float = 0.0

    @property
    def position(self) -> Tuple[float, float, float]:
        if not self.history:
            return (0.0, 0.0, 0.0)
        _, x, y, z = self.history[-1]
        return (x, y, z)


class PedestrianPredictionNode(Node):
    """Maintain tracks and predict future pedestrian positions from detected poses.

    Input topics:
      - /pedestrian/detected_poses: PoseArray of untracked pedestrian positions

    Output topics:
      - /pedestrian/current_poses: PoseArray of current tracked pedestrian positions
      - /pedestrian/predicted_poses: PoseArray of future predicted positions
      - /pedestrian/predictions_flat: Float32MultiArray rows [track_id, t, x, y, vx, vy]
      - /pedestrian/trajectory_markers: MarkerArray for RViz visualization
    """

    def __init__(self):
        super().__init__('pedestrian_prediction_node')

        # Topic parameters
        self.declare_parameter('detected_poses_topic', '/pedestrian/detected_poses')
        self.declare_parameter('output_frame', 'odom')

        # Tracking / prediction parameters
        self.declare_parameter('association_distance', 1.2)
        self.declare_parameter('association_angle', 0.15)
        self.declare_parameter('track_timeout', 1.5)
        self.declare_parameter('history_length', 10)
        self.declare_parameter('prediction_horizon', 4.0)
        self.declare_parameter('prediction_dt', 0.5)
        self.declare_parameter('velocity_smoothing', 0.35)
        self.declare_parameter('max_prediction_speed', 2.0)

        self.declare_parameter('min_direction_speed', 0.1)

        # Smoothing & Output Rate
        self.declare_parameter('position_deadband', 0.3)
        self.declare_parameter('output_fps', 2.0)

        # RViz arrow length
        self.declare_parameter('arrow_length', 0.8)

        self.detected_poses_topic = self.get_parameter('detected_poses_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.association_distance = float(self.get_parameter('association_distance').value)
        self.association_angle = float(self.get_parameter('association_angle').value)
        self.track_timeout = float(self.get_parameter('track_timeout').value)
        self.history_length = int(self.get_parameter('history_length').value)
        self.prediction_horizon = float(self.get_parameter('prediction_horizon').value)
        self.prediction_dt = float(self.get_parameter('prediction_dt').value)
        self.velocity_smoothing = float(self.get_parameter('velocity_smoothing').value)
        self.max_prediction_speed = float(self.get_parameter('max_prediction_speed').value)
        self.min_direction_speed = float(self.get_parameter('min_direction_speed').value)
        self.arrow_length = float(self.get_parameter('arrow_length').value)
        self.position_deadband = float(self.get_parameter('position_deadband').value)
        self.output_fps = float(self.get_parameter('output_fps').value)

        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        normal_qos = QoSProfile(depth=10)

        self.pose_sub = self.create_subscription(
            PoseArray,
            self.detected_poses_topic,
            self.detected_poses_callback,
            normal_qos,
        )

        self.current_pose_pub = self.create_publisher(
            PoseArray,
            '/pedestrian/current_poses',
            10,
        )

        self.pred_pose_pub = self.create_publisher(
            PoseArray,
            '/pedestrian/predicted_poses',
            10,
        )

        self.get_logger().info(
            'PedestrianPredictionNode started: '
            f'{self.detected_poses_topic} -> /pedestrian/* in {self.output_frame}'
        )

        # Output timer (decoupled from input frequency)
        self.output_timer = self.create_timer(
            1.0 / self.output_fps,
            self.publish_outputs_timer_cb
        )

    def _angle_diff(self, a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def publish_outputs_timer_cb(self):
        # Publish at a fixed rate, completely masking input flicker
        self.publish_outputs(self.get_clock().now().to_msg())

    def ros_time_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def detected_poses_callback(self, msg: PoseArray):
        now_sec = self.ros_time_to_sec(msg.header.stamp)

        if now_sec == 0.0:
            now_sec = self.get_clock().now().nanoseconds * 1e-9

        source_frame = msg.header.frame_id
        if not source_frame:
            source_frame = 'base_link'

        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                source_frame,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.05)
            )
        except tf2_ros.TransformException:
            # 离线 Bag 回放时偶尔会出现时间戳对不上的情况，直接静默跳过该帧，不要用最新的 TF（会导致坐标剧烈抖动）
            return

        measurements: List[Tuple[float, float, float, float]] = []

        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = pose
            
            ps_transformed = tf2_geometry_msgs.do_transform_pose_stamped(ps, transform)
            
            mx = ps_transformed.pose.position.x
            my = ps_transformed.pose.position.y
            mz = ps_transformed.pose.position.z
            qz = ps_transformed.pose.orientation.z
            qw = ps_transformed.pose.orientation.w
            myaw = 2.0 * math.atan2(qz, qw)
            
            measurements.append((mx, my, mz, myaw))

        self.update_tracks(measurements, now_sec)
        self.prune_stale_tracks(now_sec)

    def update_tracks(
        self,
        measurements: List[Tuple[float, float, float, float]],
        stamp_sec: float,
    ):
        unmatched_tracks = set(self.tracks.keys())

        for mx, my, mz, myaw in measurements:
            best_id = None
            best_score = float('inf')

            for tid in list(unmatched_tracks):
                tx, ty, _ = self.tracks[tid].position
                dist = math.hypot(mx - tx, my - ty)
                
                # myaw and track.yaw are the absolute line-of-sight angles in odom frame
                angle_diff = abs(self._angle_diff(myaw, self.tracks[tid].yaw))

                # Match if spatially close OR angularly in the same direction
                if dist <= self.association_distance or angle_diff <= self.association_angle:
                    # Prefer tracks that are spatially close
                    score = dist + angle_diff * 5.0
                    if score < best_score:
                        best_score = score
                        best_id = tid

            if best_id is not None:
                track = self.tracks[best_id]
                tx, ty, _ = track.position
                dist = math.hypot(mx - tx, my - ty)
                
                if dist <= self.association_distance:
                    # ── Spatial deadband: ignore tiny movements ──
                    if dist >= self.position_deadband:
                        track.history.append((stamp_sec, mx, my, mz))
                else:
                    # ── Glitch Absorption ──
                    # Matched purely by angle but distance jumped wildly. 
                    # Do nothing to history. Silently swallow the background glitch.
                    pass
                
                # Always update liveness and yaw
                track.last_update = stamp_sec
                track.missed = 0
                track.yaw = myaw
                unmatched_tracks.discard(best_id)
            else:
                tid = self.next_track_id
                self.next_track_id += 1

                hist = deque(maxlen=max(2, self.history_length))
                hist.append((stamp_sec, mx, my, mz))

                self.tracks[tid] = Track(
                    track_id=tid,
                    history=hist,
                    last_update=stamp_sec,
                    yaw=myaw,
                    motion_yaw=myaw,
                )

        for tid in unmatched_tracks:
            self.tracks[tid].missed += 1

    def prune_stale_tracks(self, now_sec: float):
        stale = [
            tid for tid, tr in self.tracks.items()
            if now_sec - tr.last_update > self.track_timeout
        ]

        for tid in stale:
            del self.tracks[tid]

    def estimate_velocity(self, track: Track) -> Tuple[float, float]:
        hist = list(track.history)

        if len(hist) < 2:
            track.direction_valid = False
            track.speed = 0.0
            return (0.0, 0.0)

        recent_window = min(4, len(hist))

        t0, x0, y0, _ = hist[-recent_window]
        t1, x1, y1, _ = hist[-1]

        dt = max(t1 - t0, 1e-3)
        dx = x1 - x0
        dy = y1 - y0

        vx_raw = dx / dt
        vy_raw = dy / dt
        raw_speed = math.hypot(vx_raw, vy_raw)

        if raw_speed < self.min_direction_speed:
            track.direction_valid = False
            track.speed = raw_speed
            return (0.0, 0.0)

        speed = min(raw_speed, self.max_prediction_speed)

        motion_yaw = math.atan2(dy, dx)
        track.motion_yaw = motion_yaw

        vx = speed * math.cos(motion_yaw)
        vy = speed * math.sin(motion_yaw)

        track.direction_valid = True
        track.speed = speed

        return (vx, vy)

    def predict_track(self, track: Track) -> List[Tuple[float, float, float]]:
        x, y, _ = track.position
        vx, vy = self.estimate_velocity(track)

        predictions: List[Tuple[float, float, float]] = []

        if not track.direction_valid:
            return predictions

        t = self.prediction_dt

        while t <= self.prediction_horizon + 1e-6:
            damping = math.exp(
                -self.velocity_smoothing * max(0.0, t - self.prediction_dt)
            )

            px = x + vx * t * damping
            py = y + vy * t * damping

            predictions.append((t, px, py))
            t += self.prediction_dt

        return predictions

    def publish_outputs(self, stamp):
        current = PoseArray()
        current.header.frame_id = self.output_frame
        current.header.stamp = stamp

        predicted = PoseArray()
        predicted.header.frame_id = self.output_frame
        predicted.header.stamp = stamp

        flat = Float32MultiArray()
        flat.data = []

        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0

        for tid, track in sorted(self.tracks.items()):
            x, y, z = track.position

            vx, vy = self.estimate_velocity(track)

            detection_yaw = track.yaw
            motion_yaw = track.motion_yaw if track.direction_valid else track.yaw

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = max(0.0, z)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(detection_yaw / 2.0)
            pose.orientation.w = math.cos(detection_yaw / 2.0)
            current.poses.append(pose)

            preds = self.predict_track(track)

            for t, px, py in preds:
                ppose = Pose()
                ppose.position.x = px
                ppose.position.y = py
                ppose.position.z = 0.35
                ppose.orientation.x = 0.0
                ppose.orientation.y = 0.0
                ppose.orientation.z = math.sin(motion_yaw / 2.0)
                ppose.orientation.w = math.cos(motion_yaw / 2.0)
                predicted.poses.append(ppose)

                flat.data.extend([
                    float(tid),
                    float(t),
                    float(px),
                    float(py),
                    float(vx),
                    float(vy),
                ])

        self.current_pose_pub.publish(current)
        self.pred_pose_pub.publish(predicted)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianPredictionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()