from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch file for basic VFG control with Bezier path.

    Nodes:
    - dashboard: Visualization dashboard
    - bezier_path_publisher: Publishes Bezier curve path
    - vfg_guidance_node: Vector Field Guidance controller
    - pid_lateral_controller_node: PID lateral controller

    Usage:
    ros2 launch qcar2_control bezier_vfg_control.launch.py

    Stop all nodes with Ctrl+C
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
                executable="bezier_path_publisher",
                name="bezier_path_publisher",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="qcar2_control",
                executable="vfg_guidance_node",
                name="vfg_guidance_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="qcar2_control",
                executable="pid_lateral_controller_node",
                name="pid_lateral_controller_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
