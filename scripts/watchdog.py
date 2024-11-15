#!/bin/env python3

import rclpy
from rclpy.node import Node
from go2_control_interface.robot_interface import Go2RobotInterface
from unitree_go.msg import LowCmd
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class WatchDogNode(Node, Go2RobotInterface):

    def __init__(self):
        Node.__init__(self, "damp_joint_node")
        Go2RobotInterface.__init__(self, self)

        self.freq = self.declare_parameter("freq", 100).value
        self.n_fail = self.declare_parameter("n_fail", 2).value

        self.cnt = 0
        self.is_stopped = False
        self.is_waiting = False

        self.lowcmd_subscription =  self.create_subscription(LowCmd, "/lowcmd", self.__cmd_cb, 10)
        self.start_subscription =  self.create_subscription(Bool, "/watchdog/start", self.__start_cb, 10)
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

        self.issafe_publisher =  self.create_publisher(Bool, "/watchdog/is_safe", QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def __start_cb(self, msg):
        # Acting as an e-stop
        if not msg.data:
            self.get_logger().error("E-stop pressed.")
            self.is_waiting = False
            self.is_stopped = True
            return

        # Arming the watchdog
        self.is_waiting = True
        self.is_stopped = False
        self.get_logger().warning("Watch-dog ready, waiting for /lowcmd")

        # Send info to other nodes
        issafe_msg = Bool()
        issafe_msg.data = True
        self.issafe_publisher.publish(issafe_msg)


    def __cmd_cb(self, msg):
        if self.is_waiting:
            self.get_logger().warning("First command received on /lowcmd, watchdog armed")

        self.is_waiting = False
        self.cnt = 0


    def timer_callback(self):
        # Does not count is the watchdog is armed and wait for the first command
        if not self.is_waiting:
            self.cnt += 1

        # If the counter exceed, trigger stop
        if self.cnt >= self.n_fail:
            self._kill_robot() # ASAP
            if not self.is_stopped:
                self.get_logger().error("Watch-dog reach sending damping to all joints, spamming")
            self.is_stopped = True

        # If stopped send damping command
        if self.is_stopped:
            self._kill_robot()

    def _kill_robot(self):
        self._send_command([0.]*12, [0.]*12, [0.]*12, [0.]*12, [1.]*12, 1.0)
        # Send info to other nodes
        issafe_msg = Bool()
        issafe_msg.data = False
        self.issafe_publisher.publish(issafe_msg)

def main(args=None):
    rclpy.init(args=args)
    watch_dog_node = WatchDogNode()

    rclpy.spin(watch_dog_node)

    watch_dog_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
