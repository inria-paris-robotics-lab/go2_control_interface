#!/bin/env python3

import rclpy
from rclpy.node import Node
from go2_control_interface.robot_interface import RobotInterface


class DampJointNode(Node, RobotInterface):

    def __init__(self):
        Node.__init__(self, "damp_joint_node")
        RobotInterface.__init__(self, self)

        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        self.send_command([0]*12, [0]*12, [0]*12, [0]*12, [0.1]*12)

def main(args=None):
    rclpy.init(args=args)
    damp_joint_node = DampJointNode()
    rclpy.spin(damp_joint_node)
    damp_joint_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()