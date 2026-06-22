#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_control_interface_py import Go2ControlInterface, G1ControlInterface
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class WatchDogNode(Node):
    """
    The watchdog has 3 states :
     state | is_stopped | is_waiting | description
    -------|------------|------------|------------
       A   |     0      |     1      | The watchdog is armed, ready to start, but not actually checking
       B   |     0      |     0      | The watchdog is running, check joint velocity and timeout
       C   |     1      |     -      | The watchdog spam stops commands

    The transitions are as follow:
    A -> B : if a msg is received on /lowcmd
    B -> C : if the velocity limit (|dq| > DQ_MAX) or the timeout is exceeded
    any -> C : if a False is received on /watchdog/arm
    any -> A : if a True is received on /watchdog/arm

    The topics are published as follow :
    A or B -> is_safe set to True, no command sent to the robot
    C -> is_safe set to False, damping commands spammed to the robot
    """

    def __init__(self):
        Node.__init__(self, "watchdog")

        # Get robot type and create interface
        self.robot_if = None
        robot_type = self.declare_parameter("robot_type", rclpy.Parameter.Type.STRING).value
        # 27-DOF: was implicit. dof (27 or 29) is threaded from deploy.py --g1-dof
        # via the launch `dof:=` arg; it must match the bridge's --dof.
        dof = self.declare_parameter("dof", 27).value
        if robot_type.lower() == "go2":
            self.robot_if = Go2ControlInterface(self, joints_filter_fq_default=200)
        elif robot_type.lower() == "g1":
            self.robot_if = G1ControlInterface(self, dof=dof, joints_filter_fq_default=200)
        else:
            assert False, f"Invalid robot_type: '{robot_type}', expected 'g1' or 'go2'"

        # Watchdog timer parameters
        self.freq = self.declare_parameter("freq", 100).value
        self.n_fail = self.declare_parameter("n_fail", 2).value

        # Velocity safety limits |dq| (rad/s), in URDF order, length N_DOF. These
        # come from the robot URDF via the interface (G1 only for now)
        # DQ_MAX = None, in which case the velocity check is disabled. Position
        # bounds are no longer enforced here: out-of-range positions are prevented
        # upstream by the joint_clamp_node (soft limits), not by killing the robot.
        self.dq_max = self.robot_if.DQ_MAX
        if self.dq_max is None:
            self.get_logger().warning("Watchdog velocity check disabled (no DQ_MAX defined for this robot).")
        else:
            assert len(self.dq_max) == self.robot_if.N_DOF, (
                f"DQ_MAX should be length {self.robot_if.N_DOF}, got {len(self.dq_max)}"
            )
            self.get_logger().info(f"Watchdog dq_max (|dq| limit) is {list(self.dq_max)}")

        # Watchdog timer logic
        self.cnt = 0
        self.is_stopped = False
        self.is_waiting = False

        self.lowcmd_subscription = self.create_subscription(
            self.robot_if.get_msgs_type()[1], "/lowcmd", self.__cmd_cb, 10
        )
        self.start_subscription = self.create_subscription(Bool, "/watchdog/arm", self.__arm_disarm_cb, 10)
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)

        self.robot_if.register_callback(self.__state_cb)

        self._is_safe_publisher = self.create_publisher(
            Bool, "/watchdog/is_safe", QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )

    def __arm_disarm_cb(self, msg):
        # Acting as an e-stop
        if not msg.data:
            self._stop_robot("E-stop pressed.")
        else:
            self._arm_watchdog()

    def __cmd_cb(self, msg):
        if self.is_waiting:
            self.get_logger().warning("First command received on /lowcmd, watchdog running")

        self.is_waiting = False
        self.cnt = 0  # Reset timeout

    def __state_cb(self, t, q, dq, ddq):
        # Velocity bounds: |dq| must stay within the URDF velocity limits.
        if self.dq_max is None:
            return

        dq_bound = [abs(dq_i) > dq_max_i for dq_i, dq_max_i in zip(dq, self.dq_max)]
        if any(dq_bound):
            self._stop_robot(
                f"Watch-dog detect joint {[(i, dq[i], self.dq_max[i]) for i, b in enumerate(dq_bound) if b]} "
                "(joint number, current dq, max |dq|) over speed limit."
            )
        # TODO: Add check on tau (look at cmd ??)

    def timer_callback(self):
        # If stopped, spam damping command
        if self.is_stopped:
            self._send_kill_cmd()
            return

        if self.is_waiting:
            # No check needs to be done
            return

        # Timeout
        self.cnt += 1
        if self.cnt >= self.n_fail:
            self._stop_robot("Watch-dog timer reached.")

    def _arm_watchdog(self):
        # Arming the watchdog
        self.cnt = 0
        self.is_waiting = True
        self.is_stopped = False
        self.get_logger().warning("Watch-dog armed, waiting for /lowcmd")

        # Send info to other nodes
        is_safe_msg = Bool()
        is_safe_msg.data = True
        self._is_safe_publisher.publish(is_safe_msg)

    def _stop_robot(self, msg_str):
        self._send_kill_cmd()  # ASAP
        if not self.is_stopped:
            self.get_logger().error(msg_str + " Stopping robot.")
        self.is_stopped = True
        self.is_waiting = False

    def _send_kill_cmd(self):
        self.robot_if._send_command(
            [0.0] * self.robot_if.N_DOF,
            [0.0] * self.robot_if.N_DOF,
            [0.0] * self.robot_if.N_DOF,
            [0.0] * self.robot_if.N_DOF,
            [1.0] * self.robot_if.N_DOF,
            scaling=False,
            skip_safety=True,
        )
        # Send info to other nodes
        is_safe_msg = Bool()
        is_safe_msg.data = False
        self._is_safe_publisher.publish(is_safe_msg)


def main(args=None):
    rclpy.init(args=args)
    watch_dog_node = WatchDogNode()

    rclpy.spin(watch_dog_node)

    watch_dog_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
