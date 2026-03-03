#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Path
from qcar2_interfaces.msg import MotorCommands
from qcar2_msgs.msg import LateralGuidance
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from qcar2_control.scalecar_vfg import LPVHinfController, PathBase, VectorFieldGuidance


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PolylinePath(PathBase):
    """PathBase adapter over a ROS Path polyline.

    This adapter is used only to feed scalecar's VFG/LPV-Hinf pipeline
    without copying their algorithm implementation.
    """

    def __init__(self, points_xy: np.ndarray):
        pts = np.asarray(points_xy, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2:
            raise ValueError("points_xy must have shape (N, 2)")
        if pts.shape[0] < 2:
            raise ValueError("at least 2 points are required")

        # Remove near-duplicate points to keep segment lengths positive.
        dedup = [pts[0]]
        for p in pts[1:]:
            if np.linalg.norm(p - dedup[-1]) > 1e-6:
                dedup.append(p)

        self._pts = np.asarray(dedup, dtype=float)
        if self._pts.shape[0] < 2:
            raise ValueError("all points are duplicated")

        self._seg_vec = self._pts[1:] - self._pts[:-1]
        self._seg_len = np.linalg.norm(self._seg_vec, axis=1)
        if not np.all(self._seg_len > 1e-9):
            raise ValueError("invalid zero-length segment in path")

        self._cum_s = np.concatenate(([0.0], np.cumsum(self._seg_len)))
        self._total_length = float(self._cum_s[-1])
        self._heading_seg = np.arctan2(self._seg_vec[:, 1], self._seg_vec[:, 0])
        self._median_seg_len = float(np.median(self._seg_len))

    @property
    def total_length(self) -> float:
        return self._total_length

    def _segment_index(self, s: float) -> int:
        if self._total_length <= 0.0:
            return 0
        s_clamped = float(np.clip(s, 0.0, self._total_length))
        idx = int(np.searchsorted(self._cum_s, s_clamped, side="right") - 1)
        return int(np.clip(idx, 0, len(self._seg_len) - 1))

    def _segment_param(self, s: float) -> Tuple[int, float]:
        idx = self._segment_index(s)
        s0 = self._cum_s[idx]
        seg_len = self._seg_len[idx]
        local = float(np.clip(s - s0, 0.0, seg_len))
        ratio = local / seg_len if seg_len > 1e-9 else 0.0
        return idx, ratio

    def position(self, s: float) -> np.ndarray:
        idx, ratio = self._segment_param(s)
        return self._pts[idx] + ratio * self._seg_vec[idx]

    def tangent(self, s: float) -> np.ndarray:
        idx = self._segment_index(s)
        return self._seg_vec[idx] / self._seg_len[idx]

    def normal(self, s: float) -> np.ndarray:
        t = self.tangent(s)
        return np.array([-t[1], t[0]])

    def heading(self, s: float) -> float:
        idx = self._segment_index(s)
        return float(self._heading_seg[idx])

    def curvature(self, s: float) -> float:
        if self._total_length <= 1e-9:
            return 0.0
        ds = max(1e-3, min(0.1, 0.5 * self._median_seg_len))
        s0 = max(0.0, s - ds)
        s1 = min(self._total_length, s + ds)
        if s1 - s0 <= 1e-9:
            return 0.0
        h0 = self.heading(s0)
        h1 = self.heading(s1)
        dh = _wrap_angle(h1 - h0)
        return float(dh / (s1 - s0))

    def closest_point(self, q, s_init=None, tol=1e-8, max_iter=20):  # noqa: D401
        del s_init, tol, max_iter
        qv = np.asarray(q, dtype=float)

        p0 = self._pts[:-1]
        v = self._seg_vec
        seg_len_sq = self._seg_len * self._seg_len

        w = qv - p0
        t = np.einsum("ij,ij->i", w, v) / seg_len_sq
        t_clamped = np.clip(t, 0.0, 1.0)

        proj = p0 + t_clamped[:, None] * v
        dist_sq = np.sum((proj - qv) ** 2, axis=1)

        idx = int(np.argmin(dist_sq))
        s_star = float(self._cum_s[idx] + t_clamped[idx] * self._seg_len[idx])
        return s_star, proj[idx]


class ScalecarHinfPathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("scalecar_hinf_path_follower")

        self.declare_parameter("path_topic", "/planning/local_path_path")
        self.declare_parameter("cmd_topic", "/qcar2_motor_speed_cmd")
        self.declare_parameter("guidance_topic", "/vfg/lateral_guidance")
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("dt_ctrl", 0.05)
        self.declare_parameter("v_const", 0.3)
        self.declare_parameter("k_e", 3.0)
        self.declare_parameter("steer_limit_rad", 0.6)
        self.declare_parameter("end_stop_margin_m", 0.12)
        self.declare_parameter("publish_guidance", True)

        self._path_topic = str(self.get_parameter("path_topic").value)
        self._cmd_topic = str(self.get_parameter("cmd_topic").value)
        self._guidance_topic = str(self.get_parameter("guidance_topic").value)
        self._path_frame_default = str(self.get_parameter("path_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._dt_ctrl = float(self.get_parameter("dt_ctrl").value)
        self._v_const = float(self.get_parameter("v_const").value)
        self._k_e = float(self.get_parameter("k_e").value)
        self._steer_limit_rad = float(self.get_parameter("steer_limit_rad").value)
        self._end_stop_margin_m = float(self.get_parameter("end_stop_margin_m").value)
        self._publish_guidance = bool(self.get_parameter("publish_guidance").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._path: Optional[PolylinePath] = None
        self._guidance: Optional[VectorFieldGuidance] = None
        self._path_frame = self._path_frame_default
        self._path_signature = None
        self._curve_id = 0

        self._delta_prev = 0.0
        self._controller = LPVHinfController.default(dt=self._dt_ctrl)

        self._path_sub = self.create_subscription(Path, self._path_topic, self._path_callback, 10)
        self._cmd_pub = self.create_publisher(MotorCommands, self._cmd_topic, 10)
        self._guidance_pub = self.create_publisher(LateralGuidance, self._guidance_topic, 10)

        self._timer = self.create_timer(max(1e-3, self._dt_ctrl), self._on_timer)

        self.get_logger().info(
            "scalecar_hinf_path_follower started: "
            f"path_topic={self._path_topic}, cmd_topic={self._cmd_topic}, dt={self._dt_ctrl:.3f}s"
        )

    def _path_signature_from_points(self, points: np.ndarray, frame_id: str):
        n = points.shape[0]
        sample_n = min(8, n)
        idx = np.linspace(0, n - 1, sample_n, dtype=int)
        sampled = np.round(points[idx], 4).flatten().tolist()
        return (frame_id, n, *sampled)

    def _path_callback(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            return

        frame_id = msg.header.frame_id if msg.header.frame_id else self._path_frame_default
        points = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses],
            dtype=float,
        )

        signature = self._path_signature_from_points(points, frame_id)
        if signature == self._path_signature:
            return

        try:
            path = PolylinePath(points)
        except ValueError as exc:
            self.get_logger().warn(f"Invalid path received: {exc}")
            return

        self._path = path
        self._guidance = VectorFieldGuidance(path, k_e=self._k_e)
        self._path_frame = frame_id
        self._path_signature = signature
        self._curve_id += 1

        self._controller.reset()
        self._delta_prev = 0.0

        self.get_logger().info(
            f"Path updated: points={points.shape[0]}, length={path.total_length:.3f} m, frame={frame_id}, curve_id={self._curve_id}"
        )

    def _publish_cmd(self, steer: float, speed: float) -> None:
        cmd = MotorCommands()
        cmd.motor_names = ["steering_angle", "motor_throttle"]
        cmd.values = [float(steer), float(speed)]
        self._cmd_pub.publish(cmd)

    def _publish_guidance_status(self, status: int, status_msg: str) -> None:
        if not self._publish_guidance:
            return

        msg = LateralGuidance()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._path_frame
        msg.curve_id = int(self._curve_id)
        msg.status = int(status)
        msg.status_msg = status_msg
        msg.curvature_1pm = float("nan")
        msg.lookahead_distance_m = 0.0
        msg.lookahead_max_curvature_1pm = float("nan")
        msg.ff_lookahead_curvature_1pm = float("nan")
        self._guidance_pub.publish(msg)

    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._path_frame,
                self._base_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        return tx, ty, yaw

    def _on_timer(self) -> None:
        if self._path is None or self._guidance is None:
            self._publish_cmd(0.0, 0.0)
            self._publish_guidance_status(LateralGuidance.STATUS_NO_PATH, "No path received")
            return

        pose = self._lookup_pose()
        if pose is None:
            self._publish_cmd(0.0, 0.0)
            self._publish_guidance_status(LateralGuidance.STATUS_TF_FAIL, "TF lookup failed")
            return

        x, y, psi_vehicle = pose
        q = np.array([x, y], dtype=float)

        try:
            g = self._guidance.compute(q, psi_vehicle)
        except Exception as exc:  # Defensive: keep safe stop on runtime failure.
            self._publish_cmd(0.0, 0.0)
            self._publish_guidance_status(LateralGuidance.STATUS_NUMERIC_FAIL, f"Guidance failed: {exc}")
            return

        psi_des = float(g["psi_des"])
        kappa = float(g["kappa"])
        e_d = float(g["e_d"])
        s_star = float(g["s_star"])
        k_conv = float(g["k_conv"])
        k_trav = float(g["k_trav"])

        heading_error = _wrap_angle(psi_des - psi_vehicle)

        speed_cmd = self._v_const
        if s_star >= max(0.0, self._path.total_length - self._end_stop_margin_m):
            speed_cmd = 0.0

        rho = abs(kappa) * abs(speed_cmd)
        delta_cmd = float(
            self._controller.compute(
                e_psi=heading_error,
                delta_meas=self._delta_prev,
                rho=rho,
                dt=self._dt_ctrl,
            )
        )
        if not math.isfinite(delta_cmd):
            delta_cmd = 0.0

        delta_cmd = float(np.clip(delta_cmd, -self._steer_limit_rad, self._steer_limit_rad))
        if speed_cmd <= 0.0:
            delta_cmd = 0.0

        self._publish_cmd(delta_cmd, speed_cmd)
        self._delta_prev = delta_cmd

        if not self._publish_guidance:
            return

        closest = self._path.position(s_star)
        psi_path = self._path.heading(s_star)

        out = LateralGuidance()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._path_frame
        out.curve_id = int(self._curve_id)

        out.heading_error_rad = float(heading_error)
        out.u_star = float(s_star / max(1e-9, self._path.total_length))
        out.closest_point.x = float(closest[0])
        out.closest_point.y = float(closest[1])
        out.closest_point.z = 0.0
        out.psi_vehicle_rad = float(psi_vehicle)
        out.psi_path_tangent_rad = float(psi_path)
        out.psi_vfg_rad = float(psi_des)
        out.cross_track_error_m = float(e_d)
        out.curvature_1pm = float(kappa)

        out.lookahead_distance_m = 0.0
        out.lookahead_max_curvature_1pm = float("nan")
        out.ff_lookahead_curvature_1pm = float("nan")

        out.w1_to_path = float(k_conv)
        out.w2_tangent = float(k_trav)
        out.v1_to_path_norm = float(abs(e_d))
        out.v_vfg_norm = 1.0

        out.status = LateralGuidance.STATUS_OK
        out.status_msg = "OK (scalecar vfg + lpv-hinf)"

        self._guidance_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScalecarHinfPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
