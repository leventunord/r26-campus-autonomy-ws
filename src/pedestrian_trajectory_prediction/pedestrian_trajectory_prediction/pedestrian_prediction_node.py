"""
pedestrian_prediction_node.py — Kalman Filter 跟踪 + 匈牙利算法数据关联

架构:
  输入: /pedestrian/detected_poses (PoseArray, base_link)
  输出:
    /pedestrian/current_poses      — KF 滤波后的当前位置 (odom)
    /pedestrian/predicted_poses    — KF 多步前推预测位置 (odom)
    /pedestrian/tracker_markers    — RViz 可视化 MarkerArray (odom)
      · 青色实心球    — CONFIRMED track 当前位置
      · 青色速度箭头  — KF 估计的速度方向与大小
      · 橙色折线      — KF 预测轨迹
      · 白色文字      — Track ID
      · 灰色半透明球  — TENTATIVE track（调试用）

跟踪器设计:
  - 状态向量: [px, py, vx, vy]^T  (恒速 CV 模型)
  - 数据关联: 匈牙利算法 + 马氏距离门控
  - Track 生命周期: TENTATIVE → CONFIRMED → LOST
  - TENTATIVE track 不发布到下游，过滤虚假检测
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs


# ---------------------------------------------------------------------------
# Track 状态枚举
# ---------------------------------------------------------------------------

class TrackState(Enum):
    TENTATIVE = auto()   # 刚创建，等待足够确认帧
    CONFIRMED = auto()   # 已确认，发布到下游
    LOST      = auto()   # 已丢失，等待删除


# ---------------------------------------------------------------------------
# KalmanTrack — 封装单个行人目标的卡尔曼滤波器
# ---------------------------------------------------------------------------

class KalmanTrack:
    """
    4 维 Kalman Filter: 状态 x = [px, py, vx, vy]^T
    运动模型: 恒速 (Constant Velocity)
    观测模型: 只观测位置 z = [px, py]^T
    """

    _I4 = np.eye(4)
    _H  = np.array([[1., 0., 0., 0.],
                    [0., 1., 0., 0.]])

    def __init__(
        self,
        track_id: int,
        z: np.ndarray,           # 初始观测 [px, py]
        stamp_sec: float,
        obs_yaw: float,          # 来自 YOLO 的初始朝向 (仅用于输出 orientation)
        process_noise_std: float,
        measurement_noise_std: float,
    ):
        self.track_id = track_id
        self.state    = TrackState.TENTATIVE
        self.hits     = 1          # 连续匹配帧数（含初始）
        self.misses   = 0          # 连续未匹配帧数
        self.last_update = stamp_sec

        # 朝向：来自 YOLO 方向（始终用最新观测更新）
        self.obs_yaw: float = obs_yaw

        # 噪声参数
        self._sigma_a = process_noise_std       # 过程噪声加速度标准差 (m/s²)
        self._sigma_z = measurement_noise_std   # 观测噪声标准差 (m)
        self._R = (measurement_noise_std ** 2) * np.eye(2)

        # 初始化状态：位置来自观测，速度初始化为零
        self.x = np.array([z[0], z[1], 0., 0.], dtype=float)

        # 初始协方差：位置置信度低（对应观测噪声），速度完全不确定
        self.P = np.diag([
            measurement_noise_std ** 2,
            measurement_noise_std ** 2,
            (process_noise_std * 2) ** 2,   # 速度初始不确定性大
            (process_noise_std * 2) ** 2,
        ])

    # ------------------------------------------------------------------
    # 过程噪声矩阵 Q(dt) — 连续白噪声加速度模型
    # ------------------------------------------------------------------
    def _Q(self, dt: float) -> np.ndarray:
        s = self._sigma_a ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        return s * np.array([
            [dt4 / 4., 0.,       dt3 / 2., 0.      ],
            [0.,       dt4 / 4., 0.,       dt3 / 2.],
            [dt3 / 2., 0.,       dt2,      0.      ],
            [0.,       dt3 / 2., 0.,       dt2     ],
        ])

    # ------------------------------------------------------------------
    # 状态转移矩阵 F(dt)
    # ------------------------------------------------------------------
    @staticmethod
    def _F(dt: float) -> np.ndarray:
        return np.array([
            [1., 0., dt, 0.],
            [0., 1., 0., dt],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.],
        ])

    # ------------------------------------------------------------------
    # KF Predict — 时间更新
    # ------------------------------------------------------------------
    def predict(self, dt: float) -> np.ndarray:
        """前推一步，返回预测的观测位置 [px, py]"""
        dt = max(dt, 1e-4)
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)
        return self._H @ self.x

    # ------------------------------------------------------------------
    # KF Update — 测量更新
    # ------------------------------------------------------------------
    def update(self, z: np.ndarray, obs_yaw: float, stamp_sec: float):
        """
        z: 观测位置 np.array([px, py])
        obs_yaw: YOLO 给出的朝向角
        stamp_sec: 时间戳
        """
        H = self._H
        S = H @ self.P @ H.T + self._R           # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)      # Kalman gain
        y = z - H @ self.x                        # Innovation
        self.x = self.x + K @ y
        self.P = (self._I4 - K @ H) @ self.P

        self.obs_yaw    = obs_yaw
        self.last_update = stamp_sec
        self.misses     = 0

    # ------------------------------------------------------------------
    # 马氏距离 — 用于数据关联门控
    # ------------------------------------------------------------------
    def mahalanobis(self, z: np.ndarray) -> float:
        """返回观测 z 与当前预测的马氏距离平方 d²"""
        H = self._H
        S = H @ self.P @ H.T + self._R
        y = z - H @ self.x
        try:
            return float(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return float('inf')

    # ------------------------------------------------------------------
    # 属性访问
    # ------------------------------------------------------------------

    @property
    def position(self) -> Tuple[float, float]:
        """KF 滤波后的当前位置 (px, py)"""
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        """KF 估计的当前速度 (vx, vy)"""
        return float(self.x[2]), float(self.x[3])

    @property
    def speed(self) -> float:
        vx, vy = self.velocity
        return math.hypot(vx, vy)

    @property
    def motion_yaw(self) -> float:
        """由速度方向推算的航向角（速度太小时返回 obs_yaw）"""
        vx, vy = self.velocity
        if math.hypot(vx, vy) > 0.1:
            return math.atan2(vy, vx)
        return self.obs_yaw


# ---------------------------------------------------------------------------
# PedestrianPredictionNode — 主节点
# ---------------------------------------------------------------------------

class PedestrianPredictionNode(Node):
    """
    Kalman Filter 行人跟踪与轨迹预测节点。

    输入:
      /pedestrian/detected_poses (PoseArray, base_link 坐标系)
        - pose.position: 行人 3D 位置
        - pose.orientation: 编码为四元数的 YOLO 朝向角

    输出:
      /pedestrian/current_poses   (PoseArray, odom 坐标系)
      /pedestrian/predicted_poses (PoseArray, odom 坐标系)
    """

    def __init__(self):
        super().__init__('pedestrian_prediction_node')

        # ── 参数声明 ──────────────────────────────────────────────────────
        self.declare_parameter('detected_poses_topic', '/pedestrian/detected_poses')
        self.declare_parameter('output_frame', 'odom')

        # KF 噪声参数
        self.declare_parameter('process_noise_std',    1.5)   # σ_a  (m/s²)
        self.declare_parameter('measurement_noise_std', 0.3)  # σ_z  (m)

        # 数据关联
        self.declare_parameter('gating_threshold', 9.21)      # 马氏距离门控 (χ²(2) 99%)

        # Track 生命周期
        self.declare_parameter('confirm_hits',  3)     # TENTATIVE→CONFIRMED 所需连续匹配帧数
        self.declare_parameter('max_misses',    5)     # 最大连续未匹配帧数
        self.declare_parameter('track_timeout', 2.0)  # 超时秒数

        # 轨迹预测
        self.declare_parameter('prediction_horizon',    3.0)  # 预测时间范围 (s)
        self.declare_parameter('prediction_dt',         0.5)  # 预测步长 (s)
        self.declare_parameter('max_prediction_speed',  2.0)  # 速度上限 (m/s)
        self.declare_parameter('velocity_damping',      0.3)  # 速度指数衰减系数

        # 输出频率
        self.declare_parameter('output_fps', 5.0)

        # ── 读取参数 ──────────────────────────────────────────────────────
        self._input_topic        = self.get_parameter('detected_poses_topic').value
        self._output_frame       = self.get_parameter('output_frame').value

        self._process_noise_std    = float(self.get_parameter('process_noise_std').value)
        self._measurement_noise_std = float(self.get_parameter('measurement_noise_std').value)

        self._gating_threshold   = float(self.get_parameter('gating_threshold').value)

        self._confirm_hits       = int(self.get_parameter('confirm_hits').value)
        self._max_misses         = int(self.get_parameter('max_misses').value)
        self._track_timeout      = float(self.get_parameter('track_timeout').value)

        self._pred_horizon       = float(self.get_parameter('prediction_horizon').value)
        self._pred_dt            = float(self.get_parameter('prediction_dt').value)
        self._max_pred_speed     = float(self.get_parameter('max_prediction_speed').value)
        self._vel_damping        = float(self.get_parameter('velocity_damping').value)

        self._output_fps         = float(self.get_parameter('output_fps').value)

        # ── 跟踪器状态 ────────────────────────────────────────────────────
        self._tracks: Dict[int, KalmanTrack] = {}
        self._next_id = 1
        self._last_stamp: float = 0.0   # 上一帧的时间戳，用于计算 dt

        # ── TF ───────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── 订阅与发布 ───────────────────────────────────────────────────
        qos = QoSProfile(depth=10)

        self.create_subscription(
            PoseArray,
            self._input_topic,
            self._on_detected_poses,
            qos,
        )

        self._pub_current   = self.create_publisher(PoseArray,    '/pedestrian/current_poses',   10)
        self._pub_predicted = self.create_publisher(PoseArray,    '/pedestrian/predicted_poses', 10)
        self._pub_markers   = self.create_publisher(MarkerArray,  '/pedestrian/tracker_markers', 10)

        # 独立发布定时器（与输入频率解耦，防止输入闪烁影响下游）
        self.create_timer(1.0 / self._output_fps, self._on_publish_timer)

        self.get_logger().info(
            f'PedestrianPredictionNode [KF] started: '
            f'{self._input_topic} → /pedestrian/* in {self._output_frame}'
        )

    # ------------------------------------------------------------------
    # 回调：接收上游 fusion 的检测结果
    # ------------------------------------------------------------------

    def _on_detected_poses(self, msg: PoseArray):
        # ── 获取时间戳 ────────────────────────────────────────────────
        now_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if now_sec == 0.0:
            now_sec = self.get_clock().now().nanoseconds * 1e-9

        # ── TF 变换到 odom ───────────────────────────────────────────
        src_frame = msg.header.frame_id or 'base_link'
        try:
            tf = self._tf_buffer.lookup_transform(
                self._output_frame,
                src_frame,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.05),
            )
        except tf2_ros.TransformException:
            # 离线 Bag 回放时偶尔时间戳对不上，静默跳过（不用最新 TF 避免坐标抖动）
            return

        # ── 将所有检测转换到 odom 坐标系 ─────────────────────────────
        measurements: List[Tuple[np.ndarray, float]] = []  # [(z, yaw), ...]
        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose   = pose
            ps_t = tf2_geometry_msgs.do_transform_pose_stamped(ps, tf)

            px   = ps_t.pose.position.x
            py   = ps_t.pose.position.y
            qz   = ps_t.pose.orientation.z
            qw   = ps_t.pose.orientation.w
            yaw  = 2.0 * math.atan2(qz, qw)

            measurements.append((np.array([px, py]), yaw))

        # ── 计算 dt ───────────────────────────────────────────────────
        if self._last_stamp > 0.0:
            dt = max(now_sec - self._last_stamp, 1e-4)
        else:
            dt = 0.1   # 第一帧：假设 10Hz

        self._last_stamp = now_sec

        # ── 执行 KF 跟踪管线 ─────────────────────────────────────────
        self._run_tracking(measurements, dt, now_sec)

    # ------------------------------------------------------------------
    # 核心跟踪管线
    # ------------------------------------------------------------------

    def _run_tracking(
        self,
        measurements: List[Tuple[np.ndarray, float]],
        dt: float,
        now_sec: float,
    ):
        track_ids = list(self._tracks.keys())

        # ── Step 1: 对所有现有 track 执行 KF predict ─────────────────
        for tid in track_ids:
            self._tracks[tid].predict(dt)

        # ── Step 2: 构建代价矩阵（马氏距离） ─────────────────────────
        n_tracks = len(track_ids)
        n_meas   = len(measurements)

        if n_tracks > 0 and n_meas > 0:
            cost_matrix = np.full((n_tracks, n_meas), fill_value=1e9, dtype=float)
            for i, tid in enumerate(track_ids):
                for j, (z, _) in enumerate(measurements):
                    d2 = self._tracks[tid].mahalanobis(z)
                    if d2 < self._gating_threshold:
                        cost_matrix[i, j] = d2

            # ── Step 3: 匈牙利算法最优匹配 ───────────────────────────
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            matched_tracks = set()
            matched_meas   = set()

            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] >= 1e9:
                    continue  # 门控拒绝，视为未匹配
                tid = track_ids[r]
                z, yaw = measurements[c]

                # ── Step 4: 已匹配 track 执行 KF update ──────────────
                track = self._tracks[tid]
                track.update(z, yaw, now_sec)
                track.hits  += 1
                track.misses = 0

                # TENTATIVE 达到 confirm_hits 后升级为 CONFIRMED
                if (track.state == TrackState.TENTATIVE
                        and track.hits >= self._confirm_hits):
                    track.state = TrackState.CONFIRMED

                matched_tracks.add(r)
                matched_meas.add(c)

        else:
            matched_tracks = set()
            matched_meas   = set()

        # ── Step 5: 未匹配 track → 递增 miss ────────────────────────
        for i, tid in enumerate(track_ids):
            if i not in matched_tracks:
                self._tracks[tid].misses  += 1
                self._tracks[tid].hits   = max(0, self._tracks[tid].hits - 1)

        # ── Step 6: 未匹配观测 → 创建新 TENTATIVE track ─────────────
        for j, (z, yaw) in enumerate(measurements):
            if j not in matched_meas:
                self._tracks[self._next_id] = KalmanTrack(
                    track_id=self._next_id,
                    z=z,
                    stamp_sec=now_sec,
                    obs_yaw=yaw,
                    process_noise_std=self._process_noise_std,
                    measurement_noise_std=self._measurement_noise_std,
                )
                self._next_id += 1

        # ── Step 7: 删除过期/丢失 track ──────────────────────────────
        stale = [
            tid for tid, tr in self._tracks.items()
            if (tr.misses > self._max_misses
                or now_sec - tr.last_update > self._track_timeout)
        ]
        for tid in stale:
            del self._tracks[tid]

    # ------------------------------------------------------------------
    # 轨迹预测 — 使用 KF 多步前推
    # ------------------------------------------------------------------

    def _predict_trajectory(
        self, track: KalmanTrack
    ) -> List[Tuple[float, float]]:
        """
        从当前 KF 状态出发，反复应用 F(dt) 进行多步预测。
        返回 [(px, py), ...] 的未来位置列表。
        """
        vx, vy = track.velocity
        speed  = math.hypot(vx, vy)

        # 速度过小，不预测（没意义且容易被噪声主导）
        if speed < 0.1:
            return []

        # 速度上限处理
        if speed > self._max_pred_speed:
            scale = self._max_pred_speed / speed
            vx *= scale
            vy *= scale

        # 当前滤波后的状态
        px, py = track.position
        x_sim  = np.array([px, py, vx, vy])

        positions: List[Tuple[float, float]] = []
        F = KalmanTrack._F(self._pred_dt)
        t  = self._pred_dt

        while t <= self._pred_horizon + 1e-6:
            x_sim = F @ x_sim

            # 速度衰减（越远预测越保守）
            damping = math.exp(-self._vel_damping * max(0.0, t - self._pred_dt))
            px_pred = x_sim[0]
            py_pred = x_sim[1]

            # 速度分量也随衰减缩减（仅影响模拟速度，位置已被积分）
            x_sim[2] *= damping
            x_sim[3] *= damping

            positions.append((px_pred, py_pred))
            t += self._pred_dt

        return positions

    # ------------------------------------------------------------------
    # RViz Marker 可视化构建
    # ------------------------------------------------------------------

    def _build_markers(self, stamp) -> MarkerArray:
        """
        构建完整的 MarkerArray，包含：
          - CONFIRMED track: 青色球体 + 速度箭头 + 预测折线 + ID 文字
          - TENTATIVE track: 灰色半透明球体（调试可见性）
        """
        markers = MarkerArray()

        # 每帧先发一个 DELETEALL，清除上一帧的所有旧 marker
        clear = Marker()
        clear.header.frame_id = self._output_frame
        clear.header.stamp    = stamp
        clear.action          = Marker.DELETEALL
        clear.ns              = 'pedestrian_tracker'
        clear.id              = 0
        markers.markers.append(clear)

        mid = 1  # marker ID 计数器

        for tid, track in sorted(self._tracks.items()):
            px, py = track.position
            vx, vy = track.velocity
            speed  = track.speed
            yaw    = track.motion_yaw

            is_confirmed = track.state == TrackState.CONFIRMED

            # ── 1. 球体：当前位置 ────────────────────────────────────
            sphere = Marker()
            sphere.header.frame_id = self._output_frame
            sphere.header.stamp    = stamp
            sphere.ns              = 'pedestrian_tracker'
            sphere.id              = mid; mid += 1
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position.x = px
            sphere.pose.position.y = py
            sphere.pose.position.z = 0.9   # 大约腰部高度
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.35
            sphere.scale.y = 0.35
            sphere.scale.z = 0.35
            if is_confirmed:
                # 青色 (CONFIRMED)
                sphere.color.r = 0.0
                sphere.color.g = 0.9
                sphere.color.b = 0.9
                sphere.color.a = 1.0
            else:
                # 灰色半透明 (TENTATIVE)
                sphere.color.r = 0.6
                sphere.color.g = 0.6
                sphere.color.b = 0.6
                sphere.color.a = 0.4
            markers.markers.append(sphere)

            # ── 2. Track ID 文字标签 ────────────────────────────────
            label = Marker()
            label.header.frame_id = self._output_frame
            label.header.stamp    = stamp
            label.ns              = 'pedestrian_tracker'
            label.id              = mid; mid += 1
            label.type            = Marker.TEXT_VIEW_FACING
            label.action          = Marker.ADD
            label.pose.position.x = px
            label.pose.position.y = py
            label.pose.position.z = 1.5   # 头顶上方
            label.pose.orientation.w = 1.0
            label.scale.z         = 0.25  # 字体高度
            state_str = 'C' if is_confirmed else 'T'
            label.text            = f'#{tid}[{state_str}] {speed:.1f}m/s'
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0 if is_confirmed else 0.5
            markers.markers.append(label)

            # 以下仅对 CONFIRMED track 绘制速度箭头和预测轨迹
            if not is_confirmed:
                continue

            # ── 3. 速度箭头 ──────────────────────────────────────────
            if speed > 0.1:
                arrow = Marker()
                arrow.header.frame_id = self._output_frame
                arrow.header.stamp    = stamp
                arrow.ns              = 'pedestrian_tracker'
                arrow.id              = mid; mid += 1
                arrow.type            = Marker.ARROW
                arrow.action          = Marker.ADD
                # 箭头起点 = 当前位置，终点 = 位置 + 速度向量（按比例缩放）
                arrow_scale = 1.0   # 1 m/s 对应箭头长 1m
                start = Point(x=px, y=py, z=0.9)
                end   = Point(
                    x=px + vx * arrow_scale,
                    y=py + vy * arrow_scale,
                    z=0.9,
                )
                arrow.points = [start, end]
                arrow.scale.x = 0.06   # 箭杆直径
                arrow.scale.y = 0.12   # 箭头直径
                arrow.scale.z = 0.0    # 使用 points 模式时忽略
                arrow.color.r = 0.0
                arrow.color.g = 1.0
                arrow.color.b = 0.4
                arrow.color.a = 1.0
                markers.markers.append(arrow)

            # ── 4. 预测轨迹折线 ──────────────────────────────────────
            future = self._predict_trajectory(track)
            if future:
                line = Marker()
                line.header.frame_id = self._output_frame
                line.header.stamp    = stamp
                line.ns              = 'pedestrian_tracker'
                line.id              = mid; mid += 1
                line.type            = Marker.LINE_STRIP
                line.action          = Marker.ADD
                line.scale.x         = 0.05   # 线宽
                line.color.r         = 1.0
                line.color.g         = 0.55
                line.color.b         = 0.0
                line.color.a         = 0.85
                line.pose.orientation.w = 1.0
                # 折线起点：当前位置
                line.points.append(Point(x=px, y=py, z=0.5))
                for (fpx, fpy) in future:
                    line.points.append(Point(x=fpx, y=fpy, z=0.5))
                markers.markers.append(line)

                # 在每个预测点处画一个小球，区分不同时间步
                for k, (fpx, fpy) in enumerate(future):
                    dot = Marker()
                    dot.header.frame_id = self._output_frame
                    dot.header.stamp    = stamp
                    dot.ns              = 'pedestrian_tracker'
                    dot.id              = mid; mid += 1
                    dot.type            = Marker.SPHERE
                    dot.action          = Marker.ADD
                    dot.pose.position.x = fpx
                    dot.pose.position.y = fpy
                    dot.pose.position.z = 0.5
                    dot.pose.orientation.w = 1.0
                    # 越远的预测点越小越透明
                    fade = 1.0 - k * 0.12
                    sz   = 0.10 + k * 0.01
                    dot.scale.x = sz
                    dot.scale.y = sz
                    dot.scale.z = sz
                    dot.color.r = 1.0
                    dot.color.g = 0.55
                    dot.color.b = 0.0
                    dot.color.a = max(0.2, fade)
                    markers.markers.append(dot)

        return markers

    # ------------------------------------------------------------------
    # 发布定时器回调
    # ------------------------------------------------------------------

    def _on_publish_timer(self):
        stamp = self.get_clock().now().to_msg()

        current_msg = PoseArray()
        current_msg.header.frame_id = self._output_frame
        current_msg.header.stamp    = stamp

        predicted_msg = PoseArray()
        predicted_msg.header.frame_id = self._output_frame
        predicted_msg.header.stamp    = stamp

        # 只发布 CONFIRMED 状态的 track
        for tid, track in sorted(self._tracks.items()):
            if track.state != TrackState.CONFIRMED:
                continue

            px, py   = track.position
            yaw      = track.motion_yaw

            # ── 当前位置 ────────────────────────────────────────────
            pose = Pose()
            pose.position.x     = px
            pose.position.y     = py
            pose.position.z     = 0.0
            pose.orientation.x  = 0.0
            pose.orientation.y  = 0.0
            pose.orientation.z  = math.sin(yaw / 2.0)
            pose.orientation.w  = math.cos(yaw / 2.0)
            current_msg.poses.append(pose)

            # ── 预测轨迹 ────────────────────────────────────────────
            future_positions = self._predict_trajectory(track)
            for (fpx, fpy) in future_positions:
                ppose = Pose()
                ppose.position.x     = fpx
                ppose.position.y     = fpy
                ppose.position.z     = 0.0
                ppose.orientation.x  = 0.0
                ppose.orientation.y  = 0.0
                ppose.orientation.z  = math.sin(yaw / 2.0)
                ppose.orientation.w  = math.cos(yaw / 2.0)
                predicted_msg.poses.append(ppose)

        self._pub_current.publish(current_msg)
        self._pub_predicted.publish(predicted_msg)
        self._pub_markers.publish(self._build_markers(stamp))


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

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