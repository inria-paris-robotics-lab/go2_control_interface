import rclpy
from rclpy.node import Node

from typing import List
from unitree_go.msg import LowCmd
from unitree_sdk2py.utils.crc import CRC

class Go2RobotInterface():
    # TODO: Populate this array programmatically
    __ros_to_urdf_index = [
            3,  4,  5,
            0,  1,  2,
            9, 10, 11,
            6,  7,  8,
        ] # re-ordering joints

    def __init__(self, node: Node):
        self.is_init = False
        self.scaling = 1.0

        self.node = node
        self.publisher =  self.node.create_publisher(LowCmd, "lowcmd", 10)

        self.crc = CRC()
        # TODO: Add a callback to joint_states and verify that robots is within safety bounds

    def init(self, q_start: List[float]):
        # TODO: Disable sportsmode controller
        self.__go_to_configuration__(q_start, 2.0)
        self.is_init = True
        pass

    def set_scaling(self, scaling: float):
        self.scaling = scaling

    def send_command(self, q: List[float], v: List[float], tau: List[float], kp: List[float], kd: List[float]):
        assert self.is_init, "Go2RobotInterface not init-ed, call init(q_start) first"
        assert len(q) == 12, "Wrong configuration size"
        assert len(v) == 12, "Wrong configuration size"
        assert len(tau) == 12, "Wrong configuration size"
        assert len(kp) == 12, "Wrong configuration size"
        assert len(kd) == 12, "Wrong configuration size"

        # TODO: Add a sanity check on robot pos/vel/acc

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
            i_urdf = self.__ros_to_urdf_index[i] # Re-order joints

            msg.motor_cmd[i].mode = 0x01 #Set toque mode
            msg.motor_cmd[i].q = q[i_urdf]
            msg.motor_cmd[i].dq = v[i_urdf]
            msg.motor_cmd[i].tau = self.scaling * tau[i_urdf]
            msg.motor_cmd[i].kp = self.scaling * kp[i_urdf]
            msg.motor_cmd[i].kd = self.scaling * kd[i_urdf]

        for i in range(12, 20): # Unused motors
            msg.motor_cmd[i].mode = 0x00 #Set passive mode
            msg.motor_cmd[i].q = 0.
            msg.motor_cmd[i].dq = 0.
            msg.motor_cmd[i].tau = 0.
            msg.motor_cmd[i].kp = 0.
            msg.motor_cmd[i].kd = 0.

        # Compute CRC here
        # TODO: Cleaner CRC computation
        msg.crc = self.crc._CRC__Crc32(self.crc._CRC__PackLowCmd(msg))
        self.node.publisher.publish(msg)

    def __go_to_configuration__(self, q: List[float], duration: float):
        # TODO: implement
        pass