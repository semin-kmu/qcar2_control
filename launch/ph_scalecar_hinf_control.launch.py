from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch PH path publisher + scalecar VFG/LPV-Hinf controller + dashboard."""

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
                executable="scalecar_hinf_path_follower",
                name="scalecar_hinf_path_follower",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"path_topic": "/planning/local_path_path"},
                    {"cmd_topic": "/qcar2_motor_speed_cmd"},
                    {"guidance_topic": "/vfg/lateral_guidance"},
                    {"path_frame": "map"},
                    {"base_frame": "base_link"},
                    {"tf_timeout_sec": 0.1},
                    {"dt_ctrl": 0.05},
                    {"v_const": 0.30},
                    {"k_e": 3.0},
                    {"steer_limit_rad": 0.6},
                    {"end_stop_margin_m": 0.12},
                    {"publish_guidance": True},
                ],
            ),
        ]
    )
