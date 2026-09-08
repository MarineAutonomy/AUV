from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("auv_joy_teleop")
    default_config = os.path.join(pkg, "config", "joy_teleop.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="YAML params for joy_thruster_teleop",
            ),
            DeclareLaunchArgument(
                "launch_joy_node",
                default_value="true",
                description="Also start ros2 joy_node on this machine (USB pad here)",
            ),
            DeclareLaunchArgument(
                "dev",
                default_value="/dev/input/js0",
                description="Legacy js path (informational; SDL joy uses device_id)",
            ),
            DeclareLaunchArgument(
                "device_id",
                default_value="0",
                description="SDL joystick index for joy_node (0 = first pad)",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {
                        # Humble joy is SDL-based (not linuxjs "device" path).
                        "device_id": ParameterValue(LaunchConfiguration("device_id"), value_type=int),
                        # Keep small — only kill stick drift; teleop applies its own light deadzone too.
                        "deadzone": 0.02,
                        "autorepeat_rate": 100.0,
                        # 0 = publish every SDL event ASAP (lower input→/joy latency).
                        "coalesce_interval_ms": 0,
                        "sticky_buttons": False,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("launch_joy_node")),
            ),
            Node(
                package="auv_joy_teleop",
                executable="joy_thruster_teleop",
                name="joy_thruster_teleop",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
