from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch PH VFG + H-infinite controller with dashboard."""

    return LaunchDescription(
        [
            Node(
                package="qcar2_control",
                executable="dashboard",
                name="dashboard",
                output="screen",
                emulate_tty=True,
                parameters=[{"display_rotation_deg": 0.0}],
            ),
            Node(
                package="qcar2_control",
                executable="ph_path_publisher",
                name="ph_path_publisher",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"phase_break_indices": [7, 12]},
                    {"phase_reach_hold_sec": 0.0},
                    {"stop_topic": "stop"},
                    {"stop_break_indices": [6, 10]},
                    {"stop_messages": ["stop1", "stop2"]},
                    {"stop_end_message": "stop3"},
                    {"stop_hold_sec": 3.0},
                ],
            ),
            Node(
                package="qcar2_control",
                executable="vfg_guidance_ph_node",
                name="vfg_guidance_ph_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="qcar2_control",
                executable="h_inf_controller_ph_node",
                name="h_inf_controller_ph_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"control_rate_hz": 200.0},
                    {"speed_control_enabled": False},
                    {"throttle_test_value_mps": 0.3},
                ],
            ),
        ]
    )
