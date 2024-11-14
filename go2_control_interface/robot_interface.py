import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from typing import List
from unitree_go.msg import LowCmd, LowState
from unitree_sdk2py.utils.crc import CRC

from copy import deepcopy

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
        self.subscription =  self.node.create_subscription(LowState, "lowstate", self.__state_cb, 10)
        self.last_state = None

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
        self.__send_command(q,v,tau,kp,kd)

    def get_joint_state(self):
        state = deepcopy(self.last_state)
        return state

    def __send_command(self, q: List[float], v: List[float], tau: List[float], kp: List[float], kd: List[float]):
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
        self.publisher.publish(msg)

    def __state_cb(self, msg: LowState):
        t = self.node.get_clock().now().nanoseconds / 1.e9
        q_urdf = [msg.motor_state[i].q for i in self.__ros_to_urdf_index]
        v_urdf = [msg.motor_state[i].dq for i in self.__ros_to_urdf_index]
        tau_urdf = [msg.motor_state[i].tau for i in self.__ros_to_urdf_index]

        self.last_state = t, q_urdf, v_urdf, tau_urdf
        # TODO: add some checks for safety

    def __go_to_configuration__(self, q: List[float], duration: float):
        for i in range(5):
            if self.get_joint_state() is not None:
                break
            self.node.get_clock().sleep_for(Duration(seconds=.5)) # Wait for first configuration to be received
        else:
            assert False, "Robot state not received in time for initialization of interface."

        q_goal = q
        q_start = self.get_joint_state()[1]

        t_start = self.node.get_clock().now()
        while(True):
            t = self.node.get_clock().now()
            ratio = (t - t_start).nanoseconds / (duration * 1e9)
            ratio = min(ratio, 1)

            q_des = [q_start[i] + (q_goal[i] - q_start[i]) * ratio for i in range(12)]
            self.__send_command(q_des, [0.]*12, [0.]*12, [10.]*12, [.1]*12)

            if(ratio == 1):
                break