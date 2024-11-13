import rclpy
from rclpy.node import Node

from typing import List
from unitree_go.msg import LowCmd

class RobotInterface():

    def __init__(self, node: Node):
        self.is_init = False
        self.scaling = 1.0

        self.node = node
        self.publisher =  self.node.create_publisher(LowCmd, "lowcmd", 10)

    def init(self, q_start: List[float]):
        # TODO: Disable sportsmode controller
        # TODO: go to q_start

        self.is_init = True
        pass

    def set_scaling(self, scaling: float):
        self.scaling = scaling

    def send_command(self, q: List[float], v: List[float], tau: List[float], kp: List[float], kd: List[float]):
        assert self.is_init, "RobotInterface not init-ed, call init(q_start) first"
        assert len(q) == 12, "Wrong configuration size"
        assert len(v) == 12, "Wrong configuration size"
        assert len(tau) == 12, "Wrong configuration size"
        assert len(kp) == 12, "Wrong configuration size"
        assert len(kd) == 12, "Wrong configuration size"

        msg = LowCmd()

        # Init header
        msg.head = 0xFE, 0xEF

        # Unused fields
        msg.level_flag = 0
        msg.frame_reserve = 0
        msg.sn = 0, 0
        msg.bandwidth = 0
        msg.fan = 0, 0
        msg.reserve = 0
        msg.led = [0]*12

        # battery
        msg.bms_cmd.off = 0
        msg.bms_cmd.reserve= 0, 0, 0

        # Version
        msg.sn = 0, 0

        # Gpio
        msg.gpio = 0

        for i in range(12):
            msg.motor_cmd[i].mode = 0x01 #Set toque mode
            msg.motor_cmd[i].q = q[i]
            msg.motor_cmd[i].dq = v[i]
            msg.motor_cmd[i].tau = self.scaling * tau[i]
            msg.motor_cmd[i].kp = self.scaling * kp[i]
            msg.motor_cmd[i].kd = self.scaling * kd[i]

        for i in range(12, 20): # Unused motors
            msg.motor_cmd[i].mode = 0x00 #Set passive mode
            msg.motor_cmd[i].q = 0
            msg.motor_cmd[i].dq = 0
            msg.motor_cmd[i].tau = 0
            msg.motor_cmd[i].kp = 0
            msg.motor_cmd[i].kd = 0

        # Compute CRC here
        msg.crc = 0x123
        self.node.publisher.publish(msg)