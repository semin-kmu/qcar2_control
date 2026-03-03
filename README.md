# qcar2_control

ROS 2 path-following package for QCar2.  
The current codebase includes Bezier/PH path publishers, VFG guidance, PID or H-infinity steering control, and a runtime dashboard.

## 1. Available Launch Profiles

There are 4 launch files:

1. `bezier_vfg_control.launch.py`
- `bezier_path_publisher` + `vfg_guidance_node` + `pid_lateral_controller_node` + `dashboard`

2. `ph_vfg_control.launch.py`
- `ph_path_publisher` + `vfg_guidance_ph_node` + `pid_lateral_controller_ph_node` + `dashboard`

3. `ph_vfg_h_infinite.launch.py`
- `ph_path_publisher` + `vfg_guidance_ph_node` + `h_inf_controller_ph_node` + `dashboard`

4. `ph_scalecar_hinf_control.launch.py`
- `ph_path_publisher` + `scalecar_hinf_path_follower` + `dashboard`
- This profile does not use the C++ VFG/H-inf nodes. Guidance and control are handled inside Python (`scalecar_vfg`).

## 2. Package Layout (Core Files)

```text
qcar2_control/
├── launch/
│   ├── bezier_vfg_control.launch.py
│   ├── ph_vfg_control.launch.py
│   ├── ph_vfg_h_infinite.launch.py
│   └── ph_scalecar_hinf_control.launch.py
├── src/
│   ├── vfg_guidance.cpp
│   ├── pid_controller.cpp
│   ├── vfg_guidance_ph.cpp
│   ├── pid_controller_ph.cpp
│   ├── h_inf_controller_ph.cpp
│   ├── ph_runtime.cpp
│   ├── lpv_hinf_runtime.cpp
│   └── lpv_hinf_v3_data.cpp
├── include/qcar2_control/
│   ├── bezier.hpp
│   ├── ph_runtime.hpp
│   ├── lpv_hinf_runtime.hpp
│   └── lpv_hinf_v3_data.hpp
├── qcar2_control/
│   ├── bezier_path_publisher.py
│   ├── ph_path_publisher.py
│   ├── scalecar_hinf_path_follower.py
│   ├── dashboard.py
│   └── scalecar_vfg/...
├── scripts/
│   ├── bezier_path_publisher
│   ├── ph_path_publisher
│   ├── scalecar_hinf_path_follower
│   └── dashboard
├── CMakeLists.txt
├── package.xml
└── setup.py
```

## 3. Topic and Message Interfaces

### 3.1 Common Topics

- Control command: `/qcar2_motor_speed_cmd` (`qcar2_interfaces/msg/MotorCommands`)
- Guidance output: `/vfg/lateral_guidance` (`qcar2_msgs/msg/LateralGuidance`)
- Visualization path: `/planning/local_path_path` (`nav_msgs/msg/Path`)
- RViz line marker: `/planning/local_path_marker` (`visualization_msgs/msg/Marker`)

### 3.2 Path Publishers

`bezier_path_publisher`:
- Publishes `/planning/local_path` (`qcar2_msgs/msg/BezierCurve`)
- Publishes `/planning/local_path_path`, `/planning/local_path_marker`

`ph_path_publisher`:
- Publishes `/planning/local_path_ph` (`qcar2_msgs/msg/PhQuinticPath`)
- Publishes `/planning/local_path_path`, `/planning/local_path_marker`
- Publishes `stop_topic` (default: `stop`) (`std_msgs/msg/String`)
- In `sequential_mode`, it switches partial paths by phase/stop breaks and emits stop events

### 3.3 Data Flow by Stack

Bezier + C++ VFG + PID:

```text
/planning/local_path (BezierCurve)
  -> vfg_guidance_node
  -> /vfg/lateral_guidance
  -> pid_lateral_controller_node
  -> /qcar2_motor_speed_cmd
```

PH + C++ VFG + PID/H-inf:

```text
/planning/local_path_ph (PhQuinticPath)
  -> vfg_guidance_ph_node
  -> /vfg/lateral_guidance
  -> pid_lateral_controller_ph_node or h_inf_controller_ph_node
  -> /qcar2_motor_speed_cmd
```

PH + Python scalecar follower:

```text
/planning/local_path_path (Path)
  -> scalecar_hinf_path_follower
  -> /qcar2_motor_speed_cmd
  -> (optional) /vfg/lateral_guidance
```

## 4. Key Parameters by Node

`bezier_path_publisher`:
- Path transform: `scale`, `qlabs_to_map_theta`, `qlabs_to_map_tx`, `qlabs_to_map_ty`, `translate_z`
- Path generation: `line_density`, `arc_density`
- Output control: `publish_rate_hz`, `publish_markers`, `publish_path`

`ph_path_publisher`:
- Path transform: `scale`, `qlabs_to_map_theta`, `qlabs_to_map_tx`, `qlabs_to_map_ty`, `translate_z`
- Sequential driving: `sequential_mode`, `phase_break_indices`, `phase_reach_distance_m`, `phase_reach_hold_sec`, `loop_phases`
- Stop events: `stop_topic`, `stop_break_indices`, `stop_messages`, `stop_end_message`, `stop_hold_sec`

`vfg_guidance_node` / `vfg_guidance_ph_node`:
- Loop/TF: `update_rate_hz`, `tf_timeout_sec`, `base_frame`
- VFG weight: `a0`
- Speed-coupled lookahead: `lookahead_preview_time`, `lookahead_min_distance`, `lookahead_max_distance`, `lookahead_sample_count`
- Feed-forward curvature: `ff_lookahead_distance`
- Debug: `publish_markers`

Bezier-only (`vfg_guidance_node`) extras:
- Closest-point search: `subdivide_max_depth`, `subdivide_flatness_m`, `u_search_window`

PH-only (`vfg_guidance_ph_node`) extras:
- Closest-point search: `local_search_half_window_m`, `local_search_seed_count`, `global_seeds_per_segment`, `newton_max_iterations`, `newton_tolerance`, `closest_global_fallback_dist_m`

`pid_lateral_controller_node` / `pid_lateral_controller_ph_node`:
- PID/FF: `kp`, `ki`, `kd`, `kff`, `wheelbase`, `d_filter_alpha`
- Gain scheduling: `gain_schedule_enabled`, `gain_ref_speed`, `gain_min_speed`
- Speed control: `speed_control_enabled`, `speed_max`, `speed_min`, `curvature_low`, `curvature_high`
- Command post-processing: `max_acceleration`, `max_deceleration`, `speed_smooth_alpha`, `steer_smooth_alpha`
- End stop logic: `stop_on_penultimate`, `stop_distance_m`, `stop_sample_count`
- Fixed-speed test input: `throttle_test_value_mps`

`h_inf_controller_ph_node`:
- Same speed/stop/smoothing parameter groups as PID-PH node
- H-inf scheduling option: `rho_use_lookahead_curvature`

`scalecar_hinf_path_follower`:
- Topics/frames: `path_topic`, `cmd_topic`, `guidance_topic`, `path_frame`, `base_frame`, `tf_timeout_sec`
- Control loop/constants: `dt_ctrl`, `v_const`, `k_e`, `steer_limit_rad`, `end_stop_margin_m`
- Guidance message output: `publish_guidance`

`dashboard`:
- Input/display: `path_topic`, `frame_id`, `base_frame`, `update_rate_hz`, `plot_xaxis`, `display_rotation_deg`
- History length: `max_points`
- CTE statistics: `cte_ignore_initial_distance_m`
- Plot saving: `auto_save_on_complete`, `save_directory`
- Actual-speed conversion: `wheel_radius`, `gear_ratio`, `encoder_cpr`

## 5. Build and Run Examples

```bash
colcon build --packages-select qcar2_control
source install/setup.bash
```

```bash
ros2 launch qcar2_control bezier_vfg_control.launch.py
ros2 launch qcar2_control ph_vfg_control.launch.py
ros2 launch qcar2_control ph_vfg_h_infinite.launch.py
ros2 launch qcar2_control ph_scalecar_hinf_control.launch.py
```

Individual run examples:

```bash
ros2 run qcar2_control bezier_path_publisher
ros2 run qcar2_control ph_path_publisher
ros2 run qcar2_control vfg_guidance_node
ros2 run qcar2_control vfg_guidance_ph_node
ros2 run qcar2_control pid_lateral_controller_node
ros2 run qcar2_control pid_lateral_controller_ph_node
ros2 run qcar2_control h_inf_controller_ph_node
ros2 run qcar2_control scalecar_hinf_path_follower
ros2 run qcar2_control dashboard
```

## 6. Dependencies

Based on `package.xml`:

- ROS: `rclcpp`, `rclpy`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`, `std_msgs`
- Message packages: `qcar2_msgs`, `qcar2_interfaces`
- Numeric library: `eigen` (`Eigen3`)
- Python: `python3-matplotlib`, `python3-numpy`, `python3-scipy`

## 7. Dashboard Input Checklist

`dashboard` starts data collection only when the following are ready:

- Path topic (`/planning/local_path_path`)
- Guidance topic (`/vfg/lateral_guidance`)
- Motor command topic (`/qcar2_motor_speed_cmd`)
- TF (`frame_id -> base_frame`, default `map -> base_link`)

For meaningful actual-speed plotting, `/qcar2_joint` (`sensor_msgs/msg/JointState`) should also be available.

## 8. QLabs Helper Scripts

The following root-level scripts are QLabs scenario utilities (not ROS nodes):

- `qlabs_stage.py`: spawns QLabs environment elements (traffic lights/signs/pedestrians) and starts RT model
- `pedestrian.py`: plane-world pedestrian movement scenario

## 9. Notes (Current Codebase)

- Message types are `qcar2_msgs` (not `acc_msgs`).
- In `dashboard`, `completion_distance_m` and `completion_target` are declared but not directly used in current completion logic.
- `package.xml` and `setup.py` still use `TODO` for license fields.
