from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch file for PH-path VFG + PID control.

    Nodes:
    - dashboard: Visualization dashboard
    - ph_path_publisher: Publishes PH path from segment definitions
    - vfg_guidance_ph_node: PH path-based VFG guidance controller
    - pid_lateral_controller_ph_node: PID lateral controller
    """
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
                    # For current SEGMENTS:
                    # - phase breaks [7, 12] split path into 3 parts (### boundaries)
                    # - stop breaks [6, 10] hold at waypoint "18" and "19"
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
                executable="pid_lateral_controller_ph_node",
                name="pid_lateral_controller_ph_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
