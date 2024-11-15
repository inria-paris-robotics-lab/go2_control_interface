#!/bin/env python3

import rclpy
from rclpy.node import Node
from go2_control_interface.robot_interface import Go2RobotInterface
import threading
from unitree_go.msg import LowCmd


class WatchDogNode(Node, Go2RobotInterface):

    def __init__(self):
        Node.__init__(self, "damp_joint_node")
        Go2RobotInterface.__init__(self, self)

        self.freq = self.declare_parameter("freq", 100).value
        self.n_fail = self.declare_parameter("n_fail", 2).value

        self.lowcmd_subscription =  self.create_subscription(LowCmd, "lowcmd", self.__cmd_cb, 10)
        self.timer = self.create_timer(1./self.freq, self.timer_callback)
        self.cnt = 0
        self.is_stopped = False

    def __cmd_cb(self, msg):
        self.cnt = 0

    def timer_callback(self):
        self.cnt += 1
        if self.is_stopped or self.cnt >= self.n_fail:
            if not self.is_stopped:
                self.get_logger().error("Watch-dog reach sending damping to all joints, spamming")
            self.is_stopped = True
            self._send_command([0.]*12, [0.]*12, [0.]*12, [0.]*12, [1.]*12, 1.0)


def main(args=None):
    rclpy.init(args=args)
    watch_dog_node = WatchDogNode()

    rclpy.spin(watch_dog_node)

    watch_dog_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
