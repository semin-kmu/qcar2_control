# qcar2_control

ROS 2 path-following control package for QCar2.
In the current codebase, the following four nodes operate together.

- `bezier_path_publisher` (Python): Publishes test Bezier path and visualization path
- `vfg_guidance_node` (C++): Computes lateral guidance based on VFG (Vector Field Guidance)
- `pid_lateral_controller_node` (C++): PID + feed-forward + speed-based gain scheduling control
- `dashboard` (Python): Real-time visualization of trajectory/speed/CTE

## 1. Package Structure

```text
qcar2_control/
├── include/
│   └── qcar2_control/
│       └── bezier.hpp          # Shared Bezier math (Vec2, evalBezier, etc.)
├── launch/
│   └── bezier_vfg_control.launch.py
├── qcar2_control/
│   ├── bezier_path_publisher.py
│   └── dashboard.py
├── scripts/
│   ├── bezier_path_publisher
│   └── dashboard
├── src/
│   ├── vfg_guidance.cpp
│   └── pid_controller.cpp
├── CMakeLists.txt
├── package.xml
└── setup.py
```

## 2. Overall Data Flow

```text
/planning/local_path (BezierCurve)
        |
        v
vfg_guidance_node
        |
        v
/vfg/lateral_guidance (LateralGuidance)
        |
        v
pid_lateral_controller_node
        |
        v
/qcar2_motor_speed_cmd (MotorCommands)
```

Auxiliary flows:

- `vfg_guidance_node` subscribes to `/qcar2_motor_speed_cmd` and computes speed-based lookahead distance
- `dashboard` subscribes to the following data for real-time visualization
  - `/planning/local_path_path`
  - `/vfg/lateral_guidance`
  - `/qcar2_motor_speed_cmd`
  - `/qcar2_joint`
  - TF (`map -> base_link`)

## 3. Node-by-Node Feature Summary

### 3.1 `bezier_path_publisher`

- Generates high-order Bezier control points from path definitions (`WAYPOINTS`, `SEGMENTS`)
- QLabs frame -> map frame transform (`scale`, rotation, translation)
- Periodically publishes:
  - `/planning/local_path` (`acc_msgs/msg/BezierCurve`)
  - `/planning/local_path_marker` (`visualization_msgs/msg/Marker`)
  - `/planning/local_path_path` (`nav_msgs/msg/Path`)

Commonly tuned parameters:

- `publish_rate_hz` (default `30.0`)
- `scale`, `qlabs_to_map_theta`, `qlabs_to_map_tx`, `qlabs_to_map_ty`, `translate_z`
- `line_density`, `arc_density`
- `publish_markers`, `publish_path`

### 3.2 `vfg_guidance_node`

Input:

- `/planning/local_path` (`acc_msgs/msg/BezierCurve`)
- `/qcar2_motor_speed_cmd` (`qcar2_interfaces/msg/MotorCommands`)
- TF (`path frame -> base_frame`, default `map -> base_link`)

Output:

- `/vfg/lateral_guidance` (`acc_msgs/msg/LateralGuidance`)
- `/vfg/debug_markers` (`visualization_msgs/msg/MarkerArray`, optional)

Core behavior:

- Nearest-point search based on De Casteljau subdivision
- Balances tangent following and path convergence with an `atan`-weighted VFG vector
- Computes current-position curvature, maximum lookahead curvature, and FF lookahead curvature
- Caches control polygon length (`cached_approx_length_`) once when a path is received

Commonly tuned parameters:

- `update_rate_hz` (default `100.0`)
- `a0` (default `0.2`)
- `lookahead_preview_time`, `lookahead_min_distance`, `lookahead_max_distance`
- `ff_lookahead_distance`
- `subdivide_max_depth`, `subdivide_flatness_m`, `u_search_window`

### 3.3 `pid_lateral_controller_node`

Input:

- `/vfg/lateral_guidance` (`acc_msgs/msg/LateralGuidance`)
- `/planning/local_path` (`acc_msgs/msg/BezierCurve`)

Output:

- `/qcar2_motor_speed_cmd` (`qcar2_interfaces/msg/MotorCommands`)

Core behavior:

- Steering: `P + I + D + FF` (`ff = kff * wheelbase * curvature`)
- Speed: smoothstep mapping based on maximum lookahead curvature (`speed_max ~ speed_min`)
- Gain scheduling: PID gain scale decreases as speed increases
- Command post-processing: acceleration/deceleration limits + EMA smoothing
- Stop logic: stop latch when near the path's penultimate control point

Commonly tuned parameters:

- `control_rate_hz` (default `200.0`)
- `kp`, `ki`, `kd`, `kff`, `wheelbase`
- `gain_schedule_enabled`, `gain_ref_speed`, `gain_min_speed`
- `speed_control_enabled`, `speed_max`, `speed_min`
- `curvature_low`, `curvature_high`
- `max_acceleration`, `max_deceleration`
- `speed_smooth_alpha`, `steer_smooth_alpha`
- `stop_on_penultimate`, `stop_distance_m`

### 3.4 `dashboard`

- Real-time visualization (dark theme, 13x7 window):
  - Path vs Trajectory (map, right panel)
  - Speed (target/actual)
  - Steering Angle
  - Cross-track error (CTE) + segment average
- Automatically saves PNG when completion condition is met (`auto_save_on_complete=true`)
- If `DISPLAY` is not available, disables live plotting and runs the node only
- Adjusts map display rotation with `display_rotation_deg` (default `0.0`)

Commonly tuned parameters:

- `update_rate_hz`, `max_points`, `plot_xaxis`
- `cte_ignore_initial_distance_m`
- `display_rotation_deg`
- `auto_save_on_complete`, `save_directory`

## 4. Build

From the workspace root:

```bash
colcon build --packages-select qcar2_control
source install/setup.bash
```

Required dependencies should be installed according to `package.xml`.

- `rclcpp`, `rclpy`, `tf2_ros`, `tf2_geometry_msgs`
- `geometry_msgs`, `nav_msgs`, `visualization_msgs`
- `acc_msgs`, `qcar2_interfaces`
- `python3-matplotlib`

## 5. Run

### 5.1 Run all at once (recommended)

```bash
ros2 launch qcar2_control bezier_vfg_control.launch.py
```

Launched nodes:

- `dashboard`
- `bezier_path_publisher`
- `vfg_guidance_node`
- `pid_lateral_controller_node`

### 5.2 Run individually

```bash
ros2 run qcar2_control bezier_path_publisher
ros2 run qcar2_control vfg_guidance_node
ros2 run qcar2_control pid_lateral_controller_node
ros2 run qcar2_control dashboard
```

### 5.3 Parameter override examples

```bash
ros2 run qcar2_control vfg_guidance_node --ros-args \
  -p a0:=0.15 \
  -p publish_markers:=true \
  -p ff_lookahead_distance:=0.16
```

```bash
ros2 run qcar2_control pid_lateral_controller_node --ros-args \
  -p kp:=0.7 -p kd:=0.10 -p kff:=0.9 \
  -p speed_max:=0.5 -p speed_min:=0.2
```

## 6. Checklist for Real Vehicle/Simulator Integration

- Verify `/planning/local_path` is being published correctly
- Verify TF `map -> base_link` lookups are stable
- Verify control command topic `/qcar2_motor_speed_cmd` is being consumed
- `/qcar2_joint` (`sensor_msgs/msg/JointState`) is required to show actual speed in `dashboard`

## 7. Recommended Tuning Order

1. Set `speed_control_enabled=false` and fix low speed with `throttle_test_value_mps`
2. Stabilize laterally using mainly `kp` and `kd` (add minimal `ki` at the end)
3. Compensate corner entry steering with `kff` and `ff_lookahead_distance`
4. Tune VFG convergence/oscillation balance with `a0` and lookahead parameters
5. Set `speed_control_enabled=true`, then tune curvature-speed map (`curvature_*`, `speed_*`)
6. Refine ride comfort/stability with accel/decel limits and EMA (`max_acceleration`, `speed_smooth_alpha`, etc.)

## 8. Troubleshooting

- `No path received`: Check the node publishing `/planning/local_path`
- `STATUS_TF_FAIL`: Check TF frame name (`base_frame`) and TF publisher
- Excessive vehicle oscillation:
  - Lower `kp` and `kd`, or
  - Reduce high-speed gain via `gain_ref_speed` / `gain_min_speed`
  - Lower `speed_max`
- Late corner entry:
  - Increase `kff`
  - Increase `ff_lookahead_distance`
  - Retune `curvature_low/high`
- Dashboard window not shown:
  - Check `DISPLAY` environment (auto-disabled in headless environments)

## 9. Notes for the Current Codebase

- `completion_distance_m` and `completion_target` are declared but not currently used directly for completion checks.
- The license field in `package.xml` is still `TODO`.
