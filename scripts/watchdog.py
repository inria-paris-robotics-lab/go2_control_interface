#!/bin/env python3

import rclpy
from rclpy.node import Node
from go2_control_interface.robot_interface import Go2RobotInterface
from unitree_go.msg import LowCmd
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class WatchDogNode(Node, Go2RobotInterface):
    """
    The watchdog has 3 states :
     state | is_stopped | is_waiting | description
    -------|------------|------------|------------
       A   |     0      |     1      | The watchdog is armed, check for joints bounds, but not for timeouts
       B   |     0      |     0      | The watchdog is running, check for joints bounds and timeout
       C   |     1      |     -      | The watchdog spam stops commands

    The transitions are as follow:
    A -> B : if a msg is received on /lowcmd
    A -> C : if joint bounds are exceeded
    B -> C : if the joint bounds or the timeout is exceeded
    any -> C : if a False is received on /watchdog/arm
    any -> A : if a True is received on /watchdog/arm

    The topics are published as follow :
    A or B -> is_safe set to True, no command sent to the robot
    C -> is_safe set to False, damping commands spammed to the robot
    """
    def __init__(self):
        Node.__init__(self, "watchdog")
        Go2RobotInterface.__init__(self, self)

        # Watchdog timer parameters
        self.freq = self.declare_parameter("freq", 100).value
        self.n_fail = self.declare_parameter("n_fail", 2).value

        # Safety values
        self.q_max = self.declare_parameter("q_max", [0.]).value
        self.q_min = self.declare_parameter("q_min", [0.]).value
        self.v_max = self.declare_parameter("v_max", -1.).value
        assert len(self.q_max) == 12, "Parameter q_max should be length 12"
        assert len(self.q_min) == 12, "Parameter q_min should be length 12"
        assert self.v_max >= 0., "Parameter v_max should be non negative"

        # Watchdog timer logic
        self.cnt = 0
        self.is_stopped = False
        self.is_waiting = False

        self.lowcmd_subscription =  self.create_subscription(LowCmd, "/lowcmd", self.__cmd_cb, 10)
        self.start_subscription =  self.create_subscription(Bool, "/watchdog/arm", self.__arm_disarm_cb, 10)
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

        self._is_safe_publisher =  self.create_publisher(Bool, "/watchdog/is_safe", QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

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
        self.cnt = 0 # Reset timeout

    def timer_callback(self):
        # Joint bounds
        tqva = self.get_joint_state()
        if tqva is not None:
            _, q, v, _ = tqva
            out_of_bounds = any([q[i] < self.q_min[i] or
                                q[i] > self.q_max[i] or
                                abs(v[i]) > self.v_max for i in range(12)])
            # TODO: Add check on tau (look at cmd ??)
            if out_of_bounds:
                self._stop_robot("Watch-dog detect joint out of bounds.")

        # Timeout
        if not self.is_waiting:
            self.cnt += 1
            if self.cnt >= self.n_fail:
                self._stop_robot("Watch-dog timer reached.")

        # If stopped, spam damping command
        if self.is_stopped:
            self._send_kill_cmd()

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
        self._send_kill_cmd() # ASAP
        if not self.is_stopped:
            self.get_logger().error(msg_str + " Stopping robot.")
        self.is_stopped = True
        self.is_waiting = False

    def _send_kill_cmd(self):
        self._send_command([0.]*12, [0.]*12, [0.]*12, [0.]*12, [1.]*12, 1.0)
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

if __name__ == '__main__':
    main()
