import os

import yaml
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
    # 27-DOF: was implicit. dof (27 or 29) is threaded from deploy.py --g1-dof and
    # must match the bridge's --dof. The watchdog slices the 29-DOF limit arrays
    # down to N_DOF, so the limits file is the same for both variants.
    dof = LaunchConfiguration("dof").perform(context)
    enable_clamp = LaunchConfiguration("enable_clamp").perform(context).lower() in ("true", "1", "yes")

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

    nodes = [
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
                    "dof": int(dof),
                },
            ],
        )
    ]

    # Joint clamping relay (soft position limits). Sits between controller and
    # robot: it reads `lowcmd_raw` + `lowstate` and republishes the clamped command
    # on `/lowcmd`. To feed it, the controller must publish to `lowcmd_raw` instead
    # of `/lowcmd` -- launch the controller with the remapping `lowcmd:=lowcmd_raw`.
    # Soft limits are the q_min/q_max of the same limits file used by the watchdog.
    if enable_clamp:
        print("[watchdog] Joint clamping enabled (lowcmd_raw -> clamp -> /lowcmd).")
        # The limits file keys its parameters under the `watchdog:` node name, so a
        # node named `joint_clamp` wouldn't pick up q_min/q_max from it. Read the
        # soft limits here and pass them directly to the clamp node (keeps the YAML
        # format unchanged for the watchdog and the empirical recorder).
        with open(config_path) as f:
            limits = yaml.safe_load(f)["watchdog"]["ros__parameters"]
        clamp_params = {
            "robot_type": robot_type,
            "dof": int(dof),
            "cmd_in": "lowcmd_raw",
            "cmd_out": "/lowcmd",
            "q_max": [float(x) for x in limits["q_max"]],
            "q_min": [float(x) for x in limits["q_min"]],
        }
        # Optional per-joint soft-limit margin (soft = hard -/+ margin). If the limits
        # file doesn't define it, the clamp node defaults to 0.005 rad on every joint.
        if "q_soft_margin" in limits:
            clamp_params["q_soft_margin"] = [float(x) for x in limits["q_soft_margin"]]
        nodes.append(
            Node(
                package="unitree_control_interface",
                executable="joint_clamp_node.py",
                name="joint_clamp",
                output="screen",
                parameters=[clamp_params],
            )
        )

    return nodes


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
            # 27-DOF: new arg. 27 (mode 6, waist roll/pitch locked) or 29 (mode 5).
            # Ignored for go2.
            DeclareLaunchArgument(
                "dof",
                default_value="27",
                description="Actuated G1 DOF: 27 (mode 6) or 29 (mode 5). Ignored for go2.",
            ),
            DeclareLaunchArgument(
                "enable_clamp",
                default_value="false",
                description=(
                    "Start the joint_clamp relay node (soft position limits). When true, the "
                    "controller MUST publish to 'lowcmd_raw' (launch it with remapping "
                    "lowcmd:=lowcmd_raw), otherwise the robot receives no command and is "
                    "timeout-killed. Default false preserves the previous direct /lowcmd path."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
