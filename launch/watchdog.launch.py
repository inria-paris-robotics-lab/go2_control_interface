import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    limits = LaunchConfiguration("limits").perform(context)
    n_fails = LaunchConfiguration("n_fails").perform(context)
    freq = LaunchConfiguration("freq").perform(context)

    config_dir = os.path.join(get_package_share_directory("unitree_control_interface"), "config")

    if limits == "custom":
        custom_path = os.path.join(config_dir, f"{robot_type}_custom_limits.yaml")
        default_path = os.path.join(config_dir, f"{robot_type}_default_limits.yaml")
        if os.path.exists(custom_path):
            config_path = custom_path
            print(f"[watchdog] Using custom limits: {custom_path}")
        else:
            config_path = default_path
            print(f"[watchdog] Custom limits not found, falling back to: {default_path}")
    else:
        config_path = os.path.join(config_dir, f"{robot_type}_{limits}_limits.yaml")
        print(f"[watchdog] Using limits: {config_path}")

    return [
        Node(
            package="unitree_control_interface",
            executable="watchdog_node.py",
            name="watchdog",
            output="screen",
            parameters=[
                config_path,
                {
                    "n_fails": int(n_fails),
                    "freq": int(freq),
                    "robot_type": robot_type,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", description="'go2' or 'g1' robot"),
            DeclareLaunchArgument(
                "n_fails",
                default_value="2",
                description="How many consecutive check without receiving any joint command is allowed before killing the robot.",
            ),
            DeclareLaunchArgument(
                "freq",
                default_value="100",
                description="How many checks per seconds to perform",
            ),
            DeclareLaunchArgument(
                "limits",
                default_value="custom",
                description="'custom' (falls back to default if file absent) or 'default', or any other suffix matching {robot_type}_{limits}_limits.yaml",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
