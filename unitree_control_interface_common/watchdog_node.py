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
       B   |     0      |     0      | The watchdog is running, check joints bounds and timeout
       C   |     1      |     -      | The watchdog spam stops commands

    The transitions are as follow:
    A -> B : if a msg is received on /lowcmd
    B -> C : if the joint bounds or the timeout is exceeded
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

        # Safety values
        self.q_max = self.declare_parameter("q_max", rclpy.Parameter.Type.DOUBLE_ARRAY).value
        self.q_min = self.declare_parameter("q_min", rclpy.Parameter.Type.DOUBLE_ARRAY).value
        self.get_logger().info(f"Watchdog q_max is {self.q_max}")
        self.get_logger().info(f"Watchdog q_min is {self.q_min}")

        self.margin_duration = self.declare_parameter("margin_duration", rclpy.Parameter.Type.DOUBLE_ARRAY).value

        # 27-DOF: G1 limit files are authored for the full 29-DOF set, i.e. array
        # index == unitree joint index 0..28. The 27-DOF variant (mode 6) does not
        # actuate waist_roll(13)/waist_pitch(14), so keep only the actuated indices
        # to line up with N_DOF. _urdf_to_unitree_index_array is range(29) in 29-DOF
        # (no-op) and (0..12, 15..28) in 27-DOF (drops 13/14). Go2 files already
        # match N_DOF, and a legacy 27-entry G1 file is left untouched (the asserts
        # below still validate the length).
        if robot_type.lower() == "g1" and len(self.q_max) == 29:
            keep = self.robot_if._urdf_to_unitree_index_array
            self.q_max = [self.q_max[i] for i in keep]
            self.q_min = [self.q_min[i] for i in keep]
            self.margin_duration = [self.margin_duration[i] for i in keep]

        assert len(self.q_max) == self.robot_if.N_DOF, f"Parameter q_max should be length {self.robot_if.N_DOF}"
        assert len(self.q_min) == self.robot_if.N_DOF, f"Parameter q_min should be length {self.robot_if.N_DOF}"
        assert len(self.margin_duration) == self.robot_if.N_DOF, (
            f"Parameter margin_duration should be length {self.robot_if.N_DOF}"
        )
        assert all(d >= 0.0 for d in self.margin_duration), "Parameter margin_duration should be non negative"

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
        # Joint bounds
        q_max_bound = [
            q_i + dt_i * dq_i > q_max_i for q_i, dq_i, dt_i, q_max_i in zip(q, dq, self.margin_duration, self.q_max)
        ]
        q_min_bound = [
            q_i + dt_i * dq_i < q_min_i for q_i, dq_i, dt_i, q_min_i in zip(q, dq, self.margin_duration, self.q_min)
        ]

        if any(q_max_bound): 
            self._stop_robot(
                f"Watch-dog detect joint {[(i, q[i], self.q_max[i]) for i, b in enumerate(q_max_bound) if b]}(joint number, current q, max q) out of bounds. (max q, dq)"
            )
        if any(q_min_bound):
            self._stop_robot(
                f"Watch-dog detect joint {[(i, q[i], self.q_min[i]) for i, b in enumerate(q_min_bound) if b]}(joint number, current q, max q) out of bounds. (min q, dq)"
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
