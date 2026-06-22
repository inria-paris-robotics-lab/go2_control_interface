#!/usr/bin/env python3
"""
Joint clamping relay node.

Sits in the command pipeline between the controller and the robot:

    controller --(cmd_in: lowcmd_raw)--> [joint_clamp_node] --(cmd_out: /lowcmd)--> robot/sim
                                              ^
                                       lowstate (q_state)

It enforces *soft* position limits per joint without killing the robot. For each
joint it keeps two latches ("interrupteurs"), one per direction:

  * Upper: when the raw command exceeds the soft upper limit AND the measured
    position has reached that limit, the latch engages and the output command is
    held at the measured position (q_state) -- so the position error is ~0 and the
    joint stops being driven past the limit. The latch releases (normal command
    resumes) only once the raw command drops back below the soft upper limit.
  * Lower: symmetric, against the soft lower limit.

The soft limits are the q_min/q_max arrays from the limits YAML (the same file
used to be consumed by the watchdog), authored for the full 29-DOF G1 and sliced
down to N_DOF here, exactly like the watchdog used to do.

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
        # unused this node publishes the clamped command through its own
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

        # Soft position limits, authored for 29-DOF (unitree index order). Slice to
        # N_DOF the same way the watchdog does (G1 27-DOF drops indices 13/14). After
        # slicing the arrays are in URDF order, length N_DOF.
        self.q_max = self.declare_parameter("q_max", rclpy.Parameter.Type.DOUBLE_ARRAY).value
        self.q_min = self.declare_parameter("q_min", rclpy.Parameter.Type.DOUBLE_ARRAY).value
        if robot_type.lower() == "g1" and len(self.q_max) == 29:
            keep = self.robot_if._urdf_to_unitree_index_array
            self.q_max = [self.q_max[i] for i in keep]
            self.q_min = [self.q_min[i] for i in keep]

        n_dof = self.robot_if.N_DOF
        assert len(self.q_max) == n_dof, f"Parameter q_max should be length {n_dof}, got {len(self.q_max)}"
        assert len(self.q_min) == n_dof, f"Parameter q_min should be length {n_dof}, got {len(self.q_min)}"
        assert all(lo <= hi for lo, hi in zip(self.q_min, self.q_max)), "q_min must be <= q_max for every joint"

        self.get_logger().info(f"Joint clamp soft q_max: {list(self.q_max)}")
        self.get_logger().info(f"Joint clamp soft q_min: {list(self.q_min)}")

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
            soft_max = self.q_max[j]
            soft_min = self.q_min[j]

            # Upper latch
            if not self.clamp_high[j]:
                if q_cmd > soft_max and q_st >= soft_max:
                    self.clamp_high[j] = True
            elif q_cmd <= soft_max:
                self.clamp_high[j] = False

            # Lower latch
            if not self.clamp_low[j]:
                if q_cmd < soft_min and q_st <= soft_min:
                    self.clamp_low[j] = True
            elif q_cmd >= soft_min:
                self.clamp_low[j] = False

            # While clamped, hold the command at the measured position so the joint
            # is not driven past the soft limit; otherwise pass the command through.
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
