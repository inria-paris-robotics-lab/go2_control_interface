from .robot_interface import UnitreeControlInterface
from unitree_hg.msg import LowCmd, LowState
from typing import List
from unitree_sdk2py.utils.crc import CRC


class G1ControlInterface(UnitreeControlInterface):
    # Full set of unitree motor indices for a 29-DOF G1, in URDF order.
    _UNITREE_INDICES_29 = tuple(range(29))
    # The 27-DOF variant (mode_machine=6) mechanically locks waist_roll (13) and
    # waist_pitch (14); they are excluded from the actuated set.
    _LOCKED_WAIST = (13, 14)

    # Absolute joint velocity limits |dq| (rad/s), indexed by unitree joint index
    # 0..28 (== URDF order for the G1). Read straight from g1_29dof.urdf <limit
    # velocity="...">. Order:
    #   left leg  : hip_pitch,hip_roll,hip_yaw,knee,ankle_pitch,ankle_roll
    #   right leg : (idem)
    #   waist     : yaw,roll,pitch              (roll/pitch locked in 27-DOF)
    #   left arm  : sho_pitch,sho_roll,sho_yaw,elbow,wrist_roll,wrist_pitch,wrist_yaw
    #   right arm : (idem)
    _G1_DQ_MAX_29 = (
        32.0, 32.0, 32.0, 20.0, 37.0, 37.0,   # left leg
        32.0, 32.0, 32.0, 20.0, 37.0, 37.0,   # right leg
        32.0, 37.0, 37.0,                      # waist (yaw, roll, pitch)
        37.0, 37.0, 37.0, 37.0, 37.0, 22.0, 22.0,  # left arm
        37.0, 37.0, 37.0, 37.0, 37.0, 22.0, 22.0,  # right arm
    )

    @property
    def _urdf_to_unitree_index_array(self) -> List[int]:
        if self._dof == 29:
            return self._UNITREE_INDICES_29
        # 27-DOF: was the hardcoded tuple (0..12, 15..28), i.e. skipping 13 and 14.
        return tuple(i for i in self._UNITREE_INDICES_29 if i not in self._LOCKED_WAIST)

    @property
    def DQ_MAX(self) -> List[float]:
        """
        Per-joint |dq| limits in URDF order, length N_DOF. Same slicing as q_max:
        identity in 29-DOF, drops the locked waist joints (13/14) in 27-DOF.
        """
        return tuple(self._G1_DQ_MAX_29[i] for i in self._urdf_to_unitree_index_array)

    @property
    def N_DOF(self) -> int:
        """
        Number of actuated degrees of freedom (thus free-flyer should be excluded)
        """
        return self._dof  # 27-DOF: was a hardcoded 27

    @property
    def ROBOT_FQ(self) -> float:
        """
        Control frequency of the robot (e.g 500.0Hz for the Go2, 1kHz for the G1, ...)
        """
        return 1000.0

    @property
    def Kp_static(self) -> List[int]:
        """
        Default kp gains to control the robot in position (for going to start configuration)
        """
        return [75.0] * self.N_DOF

    @property
    def Kd_static(self) -> List[int]:
        """
        Default kd gains to control the robot in position (for going to start configuration)
        """
        return [1.0] * self.N_DOF

    def get_msgs_type(self):
        """
        Returns the state and command message types to control the robot
        """
        return LowState, LowCmd

    def make_cmd_msg(self):
        """
        Create an empty command message, with all the fields pre-filled
        """
        msg = LowCmd()

        msg.mode_pr = 0  # Parallel mechanism (ankle and waist) control mode (default 0) 0:PR, 1:AB
        # G1 Type：4：23-Dof; 5:29-Dof; 6:27-Dof (29Dof fitted at the waist).
        # 27-DOF: was a hardcoded 6.
        msg.mode_machine = 5 if self._dof == 29 else 6

        return msg

    def compute_cmd_crc(self, msg) -> int:
        """
        Compute the crc of a filled command message
        """
        return self.crc._CRC__Crc32(self.crc._CRC__PackHGLowCmd(msg))

    def __init__(self, node, *, dof=27, joints_filter_fq_default=-1):
        # 27-DOF: was a hardcoded 27. Now a constructor arg threaded from the CLI
        # (deploy.py --g1-dof → watchdog `dof:=` param and bridge `--dof`).
        # Default 27 preserves the previous behaviour.
        if dof not in (27, 29):
            raise ValueError(f"G1 dof must be 27 or 29, got {dof}")
        self._dof = dof
        super().__init__(node, joints_filter_fq_default=joints_filter_fq_default)
        self.crc = CRC()
