#!/usr/bin/env python3

import os
import math
from collections import deque
from typing import Deque, List, Tuple, Optional, Dict

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from qcar2_msgs.msg import LateralGuidance
from qcar2_interfaces.msg import MotorCommands

import matplotlib

def _setup_matplotlib():
    if not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")
    matplotlib.rcParams["figure.raise_window"] = False

_setup_matplotlib()
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator


# Dark theme colors
BG_COLOR = "#1E1E1E"
TEXT_COLOR = "#E0E0E0"
GRID_COLOR = "#3A3A3A"
DIVIDER_COLOR = "#555555"
PATH_COLOR = "#4CAF50"      # Green
TRAJ_COLOR = "#FF5722"      # Orange-red
TARGET_SPEED_COLOR = "#2196F3"  # Blue
ACTUAL_SPEED_COLOR = "#FF9800"  # Orange
STEER_COLOR = "#E91E63"     # Pink
CTE_COLOR = "#9C27B0"       # Purple
WAITING_COLOR = "#FFC107"   # Amber for waiting status


class Dashboard(Node):
    def __init__(self) -> None:
        super().__init__("dashboard")

        # Parameters
        self.declare_parameter("path_topic", "/planning/local_path_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("update_rate_hz", 60.0)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("max_points", 3000)
        self.declare_parameter("enable_plot", True)
        self.declare_parameter("plot_xaxis", "distance")  # time | distance
        self.declare_parameter("completion_distance_m", 0.2)
        self.declare_parameter("completion_hold_sec", 0.8)
        self.declare_parameter("completion_target", "penultimate")  # last | penultimate
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("gear_ratio", 0.0817)  # (13*19)/(70*30)
        self.declare_parameter("encoder_cpr", 2880.0)  # 720*4
        self.declare_parameter("auto_save_on_complete", True)
        self.declare_parameter("save_directory", "./acc_plots")
        self.declare_parameter("cte_ignore_initial_distance_m", 2.0)
        self.declare_parameter("display_rotation_deg", 0.0)

        self._path_topic = str(self.get_parameter("path_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._enable_plot = bool(self.get_parameter("enable_plot").value)
        self._plot_xaxis = str(self.get_parameter("plot_xaxis").value)
        self._completion_distance_m = float(self.get_parameter("completion_distance_m").value)
        self._completion_hold_sec = float(self.get_parameter("completion_hold_sec").value)
        self._completion_target = str(self.get_parameter("completion_target").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._gear_ratio = float(self.get_parameter("gear_ratio").value)
        self._encoder_cpr = float(self.get_parameter("encoder_cpr").value)
        self._auto_save_on_complete = bool(self.get_parameter("auto_save_on_complete").value)
        self._save_directory = str(self.get_parameter("save_directory").value)
        self._cte_ignore_initial_distance_m = float(self.get_parameter("cte_ignore_initial_distance_m").value)
        self._display_yaw = math.radians(float(self.get_parameter("display_rotation_deg").value))
        self._cos_yaw = math.cos(self._display_yaw)
        self._sin_yaw = math.sin(self._display_yaw)

        if self._enable_plot and not os.environ.get("DISPLAY"):
            self.get_logger().warn("DISPLAY not set; disabling live plot.")
            self._enable_plot = False

        # Topic readiness tracking
        self._topic_ready: Dict[str, bool] = {
            "path": False,
            "tf": False,
            "guidance": False,
            "motor_cmd": False,
        }
        self._all_ready = False
        self._ready_logged = False

        # Path data (set all-at-once from topic, not incremental)
        self._path_xy: List[Tuple[float, float]] = []
        self._path_x_arr: List[float] = []
        self._path_y_arr: List[float] = []
        self._limits_set = False

        # Time-series data: split x/y deques to avoid tuple-unpacking at render time
        N = self._max_points
        self._traj_x: Deque[float] = deque(maxlen=N)
        self._traj_y: Deque[float] = deque(maxlen=N)

        self._cte_x: Deque[float] = deque(maxlen=N)
        self._cte_y: Deque[float] = deque(maxlen=N)

        # Incremental CTE stats — O(1) per sample; avoids sum()/max() over full deque
        self._cte_count: int = 0
        self._cte_running_sum: float = 0.0

        self._speed_target_x: Deque[float] = deque(maxlen=N)
        self._speed_target_y: Deque[float] = deque(maxlen=N)
        self._speed_actual_x: Deque[float] = deque(maxlen=N)
        self._speed_actual_y: Deque[float] = deque(maxlen=N)

        self._steering_x: Deque[float] = deque(maxlen=N)
        self._steering_y: Deque[float] = deque(maxlen=N)

        # State
        self._start_time: Optional[rclpy.time.Time] = None
        self._prev_pos: Optional[Tuple[float, float]] = None
        self._distance_travelled = 0.0
        self._completion_start: Optional[rclpy.time.Time] = None
        self._completed = False
        self._plot_saved = False
        self._render_tick: int = 0
        self._render_interval: int = 6  # render every 6th data tick (10 Hz at 60 Hz)
        self._last_waiting_status: str = ""

        # X axis limit: expand when data reaches 85% of current limit.
        # Y axes: relim()+autoscale_view() called only when _needs_relim is True
        # (triggered by x-limit expansion or path change — not every frame).
        self._xlim_max: float = 1.0
        self._data_x_max: float = 0.0
        self._needs_relim: bool = False


        # Latest values for real-time display
        self._latest_target_speed = 0.0
        self._latest_actual_speed = 0.0
        self._latest_steering_rad = 0.0
        self._latest_cte_cm = 0.0

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Subscriptions
        self._sub_path = self.create_subscription(
            Path, self._path_topic, self._path_callback, 10
        )
        self._sub_motor_cmd = self.create_subscription(
            MotorCommands, "/qcar2_motor_speed_cmd", self._motor_cmd_callback, 10
        )
        self._sub_joint = self.create_subscription(
            JointState, "/qcar2_joint", self._joint_callback, 10
        )
        self._sub_guidance = self.create_subscription(
            LateralGuidance, "/vfg/lateral_guidance", self._guidance_callback, 10
        )

        # Setup plots with dark theme
        if self._enable_plot:
            plt.ion()
            plt.style.use('dark_background')

            self._fig = plt.figure(figsize=(13, 7), facecolor=BG_COLOR)
            self._fig.suptitle("ACC Control Dashboard", fontsize=14, color=TEXT_COLOR)

            # GridSpec layout (3 rows, 2 columns):
            # Col 0: Speed (row 0), Steering (row 1), CTE (row 2)
            # Col 1: Map (spans all rows)
            gs = GridSpec(3, 2, figure=self._fig,
                         width_ratios=[1, 1],
                         hspace=0.3, wspace=0.2)

            x_label = "distance [m]" if self._plot_xaxis == "distance" else "time [s]"

            # Speed (col 0, row 0)
            self._ax_speed = self._fig.add_subplot(gs[0, 0])
            self._setup_axis(self._ax_speed)
            self._speed_target_line, = self._ax_speed.plot(
                [], [], color=TARGET_SPEED_COLOR, linewidth=1.5, label="target")
            self._speed_actual_line, = self._ax_speed.plot(
                [], [], color=ACTUAL_SPEED_COLOR, linewidth=1.5, label="actual")
            self._ax_speed.legend(loc="upper right", fontsize=8, facecolor=BG_COLOR, edgecolor=GRID_COLOR)
            self._ax_speed.set_title("Speed", color=TEXT_COLOR, fontsize=10)
            self._ax_speed.set_ylabel("speed [m/s]", color=TEXT_COLOR, fontsize=9)
            self._ax_speed.yaxis.set_major_locator(MultipleLocator(0.2))
            self._ax_speed.set_xlim(0, self._xlim_max)

            # Steering (col 0, row 1)
            self._ax_steer = self._fig.add_subplot(gs[1, 0])
            self._setup_axis(self._ax_steer)
            self._steer_line, = self._ax_steer.plot([], [], color=STEER_COLOR, linewidth=1.5)
            self._ax_steer.axhline(0.0, color=GRID_COLOR, linewidth=0.8, alpha=0.5)
            self._ax_steer.set_title("Steering Angle", color=TEXT_COLOR, fontsize=10)
            self._ax_steer.set_ylabel("angle [deg]", color=TEXT_COLOR, fontsize=9)
            self._ax_steer.yaxis.set_major_locator(MultipleLocator(5.0))
            self._ax_steer.set_xlim(0, self._xlim_max)

            # CTE (col 0, row 2)
            self._ax_cte = self._fig.add_subplot(gs[2, 0])
            self._setup_axis(self._ax_cte)
            self._cte_line, = self._ax_cte.plot(
                [], [], color=CTE_COLOR, linewidth=1.5)
            self._ax_cte.axhline(0.0, color=GRID_COLOR, linewidth=0.8, alpha=0.5)
            self._ax_cte.set_title("Cross-Track Error", color=TEXT_COLOR, fontsize=10)
            self._ax_cte.set_xlabel(x_label, color=TEXT_COLOR, fontsize=9)
            self._ax_cte.set_ylabel("error [cm]", color=TEXT_COLOR, fontsize=9)
            self._ax_cte.yaxis.set_major_locator(MultipleLocator(5.0))
            self._ax_cte.set_xlim(0, self._xlim_max)
            self._cte_stats_text = self._ax_cte.text(
                0.02, 0.98, "", transform=self._ax_cte.transAxes,
                verticalalignment="top", fontsize=8, color=TEXT_COLOR
            )

            # Map (col 1, spans all rows)
            self._ax_map = self._fig.add_subplot(gs[:, 1])
            self._setup_axis(self._ax_map)
            self._path_line, = self._ax_map.plot([], [], color=PATH_COLOR, linewidth=2.5, label="path")
            self._traj_line, = self._ax_map.plot([], [], color=TRAJ_COLOR, linewidth=1.5, label="vehicle")
            self._ax_map.set_aspect("equal", adjustable="box")
            self._ax_map.legend(loc="upper right", fontsize=9, facecolor=BG_COLOR, edgecolor=GRID_COLOR)
            self._ax_map.set_title("Path vs Trajectory", color=TEXT_COLOR, fontsize=11)

            self._waiting_text = self._ax_map.text(
                0.5, 0.5, "Waiting for topics...",
                transform=self._ax_map.transAxes,
                horizontalalignment="center", verticalalignment="center",
                fontsize=16, color=WAITING_COLOR, fontweight="bold"
            )
            self._waiting_status_text = self._ax_map.text(
                0.5, 0.35, "",
                transform=self._ax_map.transAxes,
                horizontalalignment="center", verticalalignment="center",
                fontsize=10, color=TEXT_COLOR, family="monospace"
            )

            self._add_dividers()
            self._fig.tight_layout(pad=0.3, h_pad=0.8, w_pad=0.5, rect=[0, 0, 1, 0.95])
            self._fig.subplots_adjust(left=0.06, right=0.99)
            plt.pause(0.05)  # Force initial window display

        period = 1.0 / max(1e-3, float(self.get_parameter("update_rate_hz").value))
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info("Dashboard started - waiting for topics...")

    def _rotate_display(self, x: float, y: float) -> Tuple[float, float]:
        """Rotate (x, y) by _display_yaw around the map origin for display."""
        return self._cos_yaw * x - self._sin_yaw * y, self._sin_yaw * x + self._cos_yaw * y

    def _setup_axis(self, ax):
        """Setup common axis styling for dark theme."""
        ax.set_facecolor(BG_COLOR)
        ax.grid(True, linestyle="--", alpha=0.3, color=GRID_COLOR)
        ax.tick_params(colors=TEXT_COLOR)
        for spine in ax.spines.values():
            spine.set_color(DIVIDER_COLOR)
            spine.set_linewidth(1.5)

    def _add_dividers(self):
        """Add vertical divider between left (plots) and right (map) columns."""
        line_v = Line2D([0.50, 0.50], [0.02, 0.98], transform=self._fig.transFigure,
                       color=DIVIDER_COLOR, linewidth=2, linestyle='-')
        self._fig.add_artist(line_v)

    def _check_all_ready(self) -> bool:
        return all(self._topic_ready.values())

    def _get_waiting_status_string(self) -> str:
        lines = []
        for name, ready in self._topic_ready.items():
            status = "[OK]" if ready else "[  ]"
            lines.append(f"{status} {name}")
        return "\n".join(lines)

    def _path_callback(self, msg: Path) -> None:
        raw = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_xy = [self._rotate_display(x, y) for x, y in raw]
        self._path_x_arr = [p[0] for p in self._path_xy]
        self._path_y_arr = [p[1] for p in self._path_xy]
        self._limits_set = False
        self._needs_relim = True
        if not self._topic_ready["path"]:
            self._topic_ready["path"] = True
            self.get_logger().info("Path topic received")

    def _motor_cmd_callback(self, msg: MotorCommands) -> None:
        if len(msg.motor_names) >= 2 and len(msg.values) >= 2:
            self._latest_steering_rad = msg.values[0]
            self._latest_target_speed = msg.values[1]
        if not self._topic_ready["motor_cmd"]:
            self._topic_ready["motor_cmd"] = True
            self.get_logger().info("Motor command topic received")

    def _joint_callback(self, msg: JointState) -> None:
        if msg.velocity and len(msg.velocity) > 0:
            encoder_speed = msg.velocity[0]
            self._latest_actual_speed = (
                (encoder_speed / self._encoder_cpr) *
                self._gear_ratio *
                (2.0 * math.pi) *
                self._wheel_radius
            )

    def _guidance_callback(self, msg: LateralGuidance) -> None:
        self._latest_cte_cm = msg.cross_track_error_m * 100.0
        if not self._topic_ready["guidance"]:
            self._topic_ready["guidance"] = True
            self.get_logger().info("Guidance topic received")

    def _update_map_limits(self) -> None:
        if not self._path_xy or not self._enable_plot:
            return
        xs = [p[0] for p in self._path_xy]
        ys = [p[1] for p in self._path_xy]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        xc = (xmin + xmax) / 2.0
        yc = (ymin + ymax) / 2.0
        half = max(xmax - xmin, ymax - ymin) / 2.0 + 0.5
        self._ax_map.set_xlim(xc - half, xc + half)
        self._ax_map.set_ylim(yc - half, yc + half)
        self._limits_set = True

    def _on_timer(self) -> None:
        now = self.get_clock().now()

        # Check TF availability
        if not self._topic_ready["tf"]:
            try:
                self._tf_buffer.lookup_transform(
                    self._frame_id,
                    self._base_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=self._tf_timeout),
                )
                self._topic_ready["tf"] = True
                self.get_logger().info("TF transform available")
            except Exception:
                pass

        # Update waiting status display (only redraw when status actually changes)
        if self._enable_plot and not self._all_ready:
            new_status = self._get_waiting_status_string()
            if new_status != self._last_waiting_status:
                self._waiting_status_text.set_text(new_status)
                self._last_waiting_status = new_status
                self._fig.canvas.draw_idle()

        # Check if all topics are ready
        if not self._all_ready:
            if self._check_all_ready():
                self._all_ready = True
                if not self._ready_logged:
                    self.get_logger().info("All topics ready - starting data collection")
                    self._ready_logged = True
                if self._enable_plot:
                    self._waiting_text.set_visible(False)
                    self._waiting_status_text.set_visible(False)
            else:
                return  # Still waiting for topics

        # Get vehicle position from TF
        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._base_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=self._tf_timeout),
            )
            x_map = tf.transform.translation.x
            y_map = tf.transform.translation.y
            xd, yd = self._rotate_display(x_map, y_map)
            self._traj_x.append(xd)
            self._traj_y.append(yd)
            if self._start_time is None:
                self._start_time = now
            if self._prev_pos is not None:
                dx = x_map - self._prev_pos[0]
                dy = y_map - self._prev_pos[1]
                self._distance_travelled += math.hypot(dx, dy)
            self._prev_pos = (x_map, y_map)  # raw map coords for distance calc
        except Exception:
            return

        if not self._enable_plot:
            return

        if self._start_time is None:
            return

        elapsed = (now - self._start_time).nanoseconds * 1e-9
        x_axis = self._distance_travelled if self._plot_xaxis == "distance" else elapsed
        self._data_x_max = max(self._data_x_max, x_axis)

        # Collect time-series data at 60 Hz
        self._speed_target_x.append(x_axis)
        self._speed_target_y.append(self._latest_target_speed)
        self._speed_actual_x.append(x_axis)
        self._speed_actual_y.append(self._latest_actual_speed)
        self._steering_x.append(x_axis)
        self._steering_y.append(math.degrees(self._latest_steering_rad))

        # Cross-track error
        if not self._completed:
            signed_cte = self._latest_cte_cm
            if self._distance_travelled >= self._cte_ignore_initial_distance_m:
                self._cte_x.append(x_axis)
                self._cte_y.append(signed_cte)
                # Incremental stats — O(1), no full-deque scan
                self._cte_count += 1
                self._cte_running_sum += abs(signed_cte)

        # Render at 10 Hz (every 6th data tick)
        self._render_tick += 1
        if self._render_tick >= self._render_interval:
            self._render_tick = 0
            self._update_plots()

        # Check completion
        if not self._completed and self._distance_travelled > 1.0:
            if abs(self._latest_target_speed) < 0.01:
                if self._completion_start is None:
                    self._completion_start = now
                elif (now - self._completion_start).nanoseconds * 1e-9 >= self._completion_hold_sec:
                    self._completed = True
                    self._save_plot_on_complete()
            else:
                self._completion_start = None

    def _save_plot_on_complete(self) -> None:
        """Automatically save plots as PNG when the run completes."""
        if not self._auto_save_on_complete or self._plot_saved or not self._enable_plot:
            return
        try:
            os.makedirs(self._save_directory, exist_ok=True)
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"acc_plot_{timestamp}.png"
            filepath = os.path.join(self._save_directory, filename)
            self._fig.savefig(filepath, dpi=150, facecolor=BG_COLOR,
                            edgecolor='none', bbox_inches='tight')
            self._plot_saved = True
            self.get_logger().info(f"Plot saved: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save plot: {e}")

    def _update_plots(self) -> None:
        # --- Update all line data ---
        if self._path_xy:
            self._path_line.set_data(self._path_x_arr, self._path_y_arr)
        self._traj_line.set_data(self._traj_x, self._traj_y)
        self._cte_line.set_data(self._cte_x, self._cte_y)
        self._speed_target_line.set_data(self._speed_target_x, self._speed_target_y)
        self._speed_actual_line.set_data(self._speed_actual_x, self._speed_actual_y)
        self._steer_line.set_data(self._steering_x, self._steering_y)

        # Update CTE stats text (O(1))
        if self._cte_count > 0:
            avg_err = self._cte_running_sum / self._cte_count
            status = "completed" if self._completed else "running"
            self._cte_stats_text.set_text(
                f"avg: {avg_err:.2f} cm"
                f"\n(>{self._cte_ignore_initial_distance_m:.0f}m) {status}"
            )
        elif self._distance_travelled < self._cte_ignore_initial_distance_m:
            self._cte_stats_text.set_text(
                f"collecting...\n(<{self._cte_ignore_initial_distance_m:.0f}m)"
            )

        # --- X limit: expand when data reaches 85% of current limit ---
        # Trigger is infrequent (~once per lap segment); relim/autoscale piggybacked.
        if self._data_x_max > self._xlim_max * 0.85:
            self._xlim_max = self._data_x_max * 1.3 + 1.0
            self._ax_speed.set_xlim(0, self._xlim_max)
            self._ax_cte.set_xlim(0, self._xlim_max)
            self._ax_steer.set_xlim(0, self._xlim_max)
            self._needs_relim = True

        # --- Map limits (once after path received) ---
        if not self._limits_set and self._path_xy:
            self._update_map_limits()

        # --- Y axis autoscale: only when flagged (x-limit change or path update) ---
        # relim()+autoscale_view() is O(n) — kept out of the hot render path.
        if self._needs_relim:
            for ax in [self._ax_speed, self._ax_cte, self._ax_steer]:
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
            self._needs_relim = False

        # --- Render ---
        self._fig.canvas.draw_idle()


def main() -> None:
    rclpy.init()
    node = Dashboard()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            if node._enable_plot:
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
