#!/usr/bin/env python3
"""
Joint clamping relay node.

Sits in the command pipeline between the controller and the robot:

    controller --(cmd_in: lowcmd_raw)--> [joint_clamp_node] --(cmd_out: /lowcmd)--> robot/sim
                                              ^
                                       lowstate (q_state)

It enforces per-joint position limits without killing the robot. Two limits per
joint: the hard limit q_lim (q_min/q_max from the limits YAML) and the soft limit
q_soft = q_lim shrunk inward by q_soft_margin. For each joint, two latches
("interrupteurs"), one per direction:

  * Upper: while the raw command stays <= the hard limit q_lim_max it passes
    through, so the joint is free to move anywhere up to the hard limit, including
    the band [soft_max, q_lim_max]. The latch engages only when the raw command
    tries to exceed the HARD limit (q_cmd > q_lim_max) AND the joint is already past
    the soft limit (q_state > soft_max); while engaged the output is held at the
    measured position q_state, so the joint stops being driven further. It releases
    as soon as the raw command comes back within the hard limit (q_cmd <= q_lim_max).
  * Lower: symmetric, against q_lim_min / soft_min.

The soft limits are derived from the hard limits q_min/q_max (the limits YAML the
watchdog used to consume) backed off by a per-joint margin:
    soft_max = q_max - q_soft_margin   ;   soft_min = q_min + q_soft_margin
q_soft_margin defaults to 0.005 rad on every joint and can be set per joint (in the
limits YAML) for a manual setup. The arrays are authored for the full 29-DOF G1 and
sliced down to N_DOF here, exactly like the watchdog used to do.

To enable this node in the pipeline, the controller must publish to `cmd_in`
instead of `/lowcmd`. As the controller publishes to the *relative* topic `lowcmd`,
this is done with a launch remapping `lowcmd:=lowcmd_raw` (no controller code
change). See watchdog.launch.py `enable_clamp:=true`.
"""

import rclpy
from rclpy.node import Node
from unitree_control_interface_py import Go2ControlInterface, G1ControlInterface


class JointClampNode(Node):
    def __init__(self):
        Node.__init__(self, "joint_clamp")

        # Robot interface (reused for: message types, urdf<->unitree index map,
        # CRC, the lowstate subscription via register_callback, and the is_safe
        # tracking). Its own lowcmd publisher / watchdog/arm publisher are left
        # unused -- this node publishes the clamped command through its own
        # publisher below. Filtering is disabled so q_state is the raw measured
        # position (most reactive / accurate reference for the clamp).
        robot_type = self.declare_parameter("robot_type", rclpy.Parameter.Type.STRING).value
        dof = self.declare_parameter("dof", 27).value
        if robot_type.lower() == "go2":
            self.robot_if = Go2ControlInterface(self, joints_filter_fq_default=-1.0)
        elif robot_type.lower() == "g1":
            self.robot_if = G1ControlInterface(self, dof=dof, joints_filter_fq_default=-1.0)
        else:
            assert False, f"Invalid robot_type: '{robot_type}', expected 'g1' or 'go2'"

        # Pipeline topics
        cmd_in = self.declare_parameter("cmd_in", "lowcmd_raw").value
        cmd_out = self.declare_parameter("cmd_out", "/lowcmd").value

        # Hard position limits (URDF / recorded limits), authored for 29-DOF (unitree
        # index order). The soft limits are these shrunk inward by a per-joint margin:
        #     soft_max = q_max - q_soft_margin ;  soft_min = q_min + q_soft_margin
        # q_soft_margin defaults to 0.005 rad on every joint; set it per joint (in the
        # limits YAML) for a manual setup. All three arrays are sliced to N_DOF the
        # same way the watchdog does (G1 27-DOF drops indices 13/14); after slicing
        # they are in URDF order, length N_DOF.
        q_max = self.declare_parameter("q_max", rclpy.Parameter.Type.DOUBLE_ARRAY).value
        q_min = self.declare_parameter("q_min", rclpy.Parameter.Type.DOUBLE_ARRAY).value
        q_soft_margin = self.declare_parameter("q_soft_margin", [0.005] * len(q_max)).value

        if robot_type.lower() == "g1" and len(q_max) == 29:
            keep = self.robot_if._urdf_to_unitree_index_array
            q_max = [q_max[i] for i in keep]
            q_min = [q_min[i] for i in keep]
            q_soft_margin = [q_soft_margin[i] for i in keep]

        n_dof = self.robot_if.N_DOF
        assert len(q_max) == n_dof, f"Parameter q_max should be length {n_dof}, got {len(q_max)}"
        assert len(q_min) == n_dof, f"Parameter q_min should be length {n_dof}, got {len(q_min)}"
        assert len(q_soft_margin) == n_dof, (
            f"Parameter q_soft_margin should be length {n_dof}, got {len(q_soft_margin)}"
        )
        assert all(m >= 0.0 for m in q_soft_margin), "q_soft_margin must be non-negative"

        # Keep both the hard limits (q_lim) and the soft limits (q_lim -/+ margin):
        # the clamp logic uses the hard limit as the command ceiling and the soft
        # limit as the "joint is in the danger zone" threshold.
        self.q_lim_max = q_max
        self.q_lim_min = q_min
        self.soft_max = [hi - m for hi, m in zip(q_max, q_soft_margin)]
        self.soft_min = [lo + m for lo, m in zip(q_min, q_soft_margin)]
        assert all(lo <= hi for lo, hi in zip(self.soft_min, self.soft_max)), (
            "Soft limits inverted: a q_soft_margin exceeds half the joint range (soft_min > soft_max)."
        )

        self.get_logger().info(f"Joint clamp hard q_max: {self.q_lim_max}")
        self.get_logger().info(f"Joint clamp hard q_min: {self.q_lim_min}")
        self.get_logger().info(f"Joint clamp soft q_max: {self.soft_max}")
        self.get_logger().info(f"Joint clamp soft q_min: {self.soft_min}")

        # Per-joint latch state (the two "interrupteurs"), URDF order.
        self.clamp_high = [False] * n_dof
        self.clamp_low = [False] * n_dof

        # Latest measured position, URDF order, set by the state callback.
        self.q_state = None
        self.robot_if.register_callback(self.__state_cb)

        # I/O: subscribe to the raw command, publish the clamped command.
        _state_type, cmd_type = self.robot_if.get_msgs_type()
        self.cmd_publisher = self.create_publisher(cmd_type, cmd_out, 10)
        self.cmd_subscription = self.create_subscription(cmd_type, cmd_in, self.__cmd_cb, 10)

        self.get_logger().info(f"Joint clamp running: '{cmd_in}' -> clamp -> '{cmd_out}'")

    def __state_cb(self, t, q, dq, ddq):
        # q is in URDF order, length N_DOF, unfiltered (filter disabled above).
        self.q_state = q

    def __cmd_cb(self, msg):
        # Soft e-stop active: drop. The watchdog already spams the kill on /lowcmd,
        # and a well-behaved controller stops sending when not safe.
        if not self.robot_if._is_safe:
            return

        # No state reference yet: cannot clamp, forward unchanged.
        if self.q_state is None:
            self.cmd_publisher.publish(msg)
            return

        for j, u in enumerate(self.robot_if._urdf_to_unitree_index_array):
            q_cmd = msg.motor_cmd[u].q
            q_st = self.q_state[j]

            # Upper latch. While the command stays <= the hard limit it passes
            # through, so the joint is free to move anywhere up to q_lim_max,
            # including the band [soft_max, q_lim_max]. Freeze only when the command
            # tries to exceed the HARD limit AND the joint is already past the soft
            # limit; release once the command is back within the hard limit.
            if not self.clamp_high[j]:
                if q_cmd > self.q_lim_max[j] and q_st > self.soft_max[j]:
                    self.clamp_high[j] = True
            elif q_cmd <= self.q_lim_max[j]:
                self.clamp_high[j] = False

            # Lower latch (symmetric).
            if not self.clamp_low[j]:
                if q_cmd < self.q_lim_min[j] and q_st < self.soft_min[j]:
                    self.clamp_low[j] = True
            elif q_cmd >= self.q_lim_min[j]:
                self.clamp_low[j] = False

            # While clamped, hold the joint at its measured position (q_state) so it
            # stops being driven past the hard limit; otherwise pass the command
            # through.
            if self.clamp_high[j] or self.clamp_low[j]:
                msg.motor_cmd[u].q = q_st

        msg.crc = self.robot_if.compute_cmd_crc(msg)
        self.cmd_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointClampNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
