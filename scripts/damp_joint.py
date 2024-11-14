#!/bin/env python3

import rclpy
from rclpy.node import Node
from go2_control_interface.robot_interface import Go2RobotInterface
import threading

class DampJointNode(Node):

    def __init__(self):
        Node.__init__(self, "damp_joint_node")
        self.robotIf = Go2RobotInterface(self)

    def start(self):
        self.robotIf.init([0.]*12)
        input("Ready to start...")
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        self.robotIf.send_command([0.]*12, [0.]*12, [0.]*12, [0.]*12, [0.1]*12)

def main(args=None):
    rclpy.init(args=args)
    damp_joint_node = DampJointNode()


    thread = threading.Thread(target=rclpy.spin, args=(damp_joint_node, ), daemon=True)
    thread.start()

    damp_joint_node.start()

    thread.join()
    damp_joint_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
