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
        self.declare_parameter('track_timeout', 1.5)
        self.declare_parameter('history_length', 10)
        self.declare_parameter('prediction_horizon', 4.0)
        self.declare_parameter('prediction_dt', 0.5)
        self.declare_parameter('velocity_smoothing', 0.35)
        self.declare_parameter('max_prediction_speed', 2.0)

        # Basic speed threshold for valid motion direction
        self.declare_parameter('min_direction_speed', 0.1)

        # RViz arrow length
        self.declare_parameter('arrow_length', 0.8)

        self.detected_poses_topic = self.get_parameter('detected_poses_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.association_distance = float(self.get_parameter('association_distance').value)
        self.track_timeout = float(self.get_parameter('track_timeout').value)
        self.history_length = int(self.get_parameter('history_length').value)
        self.prediction_horizon = float(self.get_parameter('prediction_horizon').value)
        self.prediction_dt = float(self.get_parameter('prediction_dt').value)
        self.velocity_smoothing = float(self.get_parameter('velocity_smoothing').value)
        self.max_prediction_speed = float(self.get_parameter('max_prediction_speed').value)
        self.min_direction_speed = float(self.get_parameter('min_direction_speed').value)
        self.arrow_length = float(self.get_parameter('arrow_length').value)

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

        self.pred_flat_pub = self.create_publisher(
            Float32MultiArray,
            '/pedestrian/predictions_flat',
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/pedestrian/trajectory_markers',
            10,
        )

        self.get_logger().info(
            'PedestrianPredictionNode started: '
            f'{self.detected_poses_topic} -> /pedestrian/* in {self.output_frame}'
        )

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
                rclpy.duration.Duration(seconds=0.1)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform {source_frame} to {self.output_frame}: {ex}')
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
        self.publish_outputs(msg.header.stamp)

    def update_tracks(
        self,
        measurements: List[Tuple[float, float, float, float]],
        stamp_sec: float,
    ):
        unmatched_tracks = set(self.tracks.keys())

        for mx, my, mz, myaw in measurements:
            best_id = None
            best_dist = float('inf')

            for tid in list(unmatched_tracks):
                tx, ty, _ = self.tracks[tid].position
                dist = math.hypot(mx - tx, my - ty)

                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is not None and best_dist <= self.association_distance:
                track = self.tracks[best_id]
                track.history.append((stamp_sec, mx, my, mz))
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

            markers.markers.extend(
                self.make_track_markers(tid, track, preds, marker_id, stamp)
            )

            marker_id += 100

        self.current_pose_pub.publish(current)
        self.pred_pose_pub.publish(predicted)
        self.pred_flat_pub.publish(flat)
        self.marker_pub.publish(markers)

    def make_track_markers(
        self,
        tid: int,
        track: Track,
        preds: List[Tuple[float, float, float]],
        base_id: int,
        stamp
    ) -> List[Marker]:
        x, y, _ = track.position
        detection_yaw = track.yaw
        motion_yaw = track.motion_yaw if track.direction_valid else track.yaw

        out: List[Marker] = []

        current = Marker()
        current.header.frame_id = self.output_frame
        current.header.stamp = stamp
        current.ns = 'pedestrian_current'
        current.id = base_id
        current.type = Marker.SPHERE
        current.action = Marker.ADD
        current.pose.position.x = x
        current.pose.position.y = y
        current.pose.position.z = 0.35
        current.pose.orientation.x = 0.0
        current.pose.orientation.y = 0.0
        current.pose.orientation.z = math.sin(detection_yaw / 2.0)
        current.pose.orientation.w = math.cos(detection_yaw / 2.0)
        current.scale.x = 0.35
        current.scale.y = 0.35
        current.scale.z = 0.7
        current.color.a = 0.9
        current.color.r = 0.1
        current.color.g = 0.7
        current.color.b = 1.0
        out.append(current)

        direction = Marker()
        direction.header.frame_id = self.output_frame
        direction.header.stamp = stamp
        direction.ns = 'pedestrian_detection_direction'
        direction.id = base_id + 1
        direction.type = Marker.ARROW
        direction.action = Marker.ADD
        direction.scale.x = 0.08
        direction.scale.y = 0.18
        direction.scale.z = 0.25
        direction.color.a = 1.0
        direction.color.r = 1.0
        direction.color.g = 0.2
        direction.color.b = 0.0

        p_start = Point()
        p_start.x = x
        p_start.y = y
        p_start.z = 0.35

        p_end = Point()
        p_end.x = x + self.arrow_length * math.cos(detection_yaw)
        p_end.y = y + self.arrow_length * math.sin(detection_yaw)
        p_end.z = 0.35

        direction.points.append(p_start)
        direction.points.append(p_end)

        if not track.direction_valid:
            direction.color.a = 0.35
            direction.scale.x = 0.04
            direction.scale.y = 0.10
            direction.scale.z = 0.15
            direction.points[1].x = x + 0.35 * math.cos(detection_yaw)
            direction.points[1].y = y + 0.35 * math.sin(detection_yaw)

        out.append(direction)

        text = Marker()
        text.header.frame_id = self.output_frame
        text.header.stamp = stamp
        text.ns = 'pedestrian_id'
        text.id = base_id + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 1.2
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = f'ID {tid}\nv={track.speed:.2f} m/s'
        out.append(text)

        hist = Marker()
        hist.header.frame_id = self.output_frame
        hist.header.stamp = stamp
        hist.ns = 'pedestrian_history'
        hist.id = base_id + 3
        hist.type = Marker.LINE_STRIP
        hist.action = Marker.ADD
        hist.scale.x = 0.05
        hist.color.a = 0.8
        hist.color.r = 0.2
        hist.color.g = 1.0
        hist.color.b = 0.2

        for _, hx, hy, _ in track.history:
            p = Point()
            p.x = hx
            p.y = hy
            p.z = 0.15
            hist.points.append(p)

        out.append(hist)

        fut = Marker()
        fut.header.frame_id = self.output_frame
        fut.header.stamp = stamp
        fut.ns = 'pedestrian_prediction'
        fut.id = base_id + 4
        fut.type = Marker.SPHERE_LIST
        fut.action = Marker.ADD
        fut.scale.x = 0.15
        fut.scale.y = 0.15
        fut.scale.z = 0.15
        fut.color.a = 0.9
        fut.color.r = 1.0
        fut.color.g = 0.8
        fut.color.b = 0.0

        p0 = Point()
        p0.x = x
        p0.y = y
        p0.z = 0.35
        fut.points.append(p0)

        for _, px, py in preds:
            p = Point()
            p.x = px
            p.y = py
            p.z = 0.35
            fut.points.append(p)

        out.append(fut)

        if preds:
            _, last_x, last_y = preds[-1]

            pred_arrow = Marker()
            pred_arrow.header.frame_id = self.output_frame
            pred_arrow.header.stamp = stamp
            pred_arrow.ns = 'pedestrian_motion_prediction_arrow'
            pred_arrow.id = base_id + 5
            pred_arrow.type = Marker.ARROW
            pred_arrow.action = Marker.ADD
            pred_arrow.scale.x = 0.05
            pred_arrow.scale.y = 0.14
            pred_arrow.scale.z = 0.20
            pred_arrow.color.a = 0.8
            pred_arrow.color.r = 1.0
            pred_arrow.color.g = 0.8
            pred_arrow.color.b = 0.0

            ps = Point()
            ps.x = x
            ps.y = y
            ps.z = 0.35

            pe = Point()
            pe.x = last_x
            pe.y = last_y
            pe.z = 0.35

            pred_arrow.points.append(ps)
            pred_arrow.points.append(pe)
            out.append(pred_arrow)

        return out


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