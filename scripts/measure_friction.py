#!/bin/env python3

#!/bin/env python3

import rclpy
import rclpy.clock
from rclpy.duration import Duration
from rclpy.node import Node
from go2_control_interface.robot_interface import Go2RobotInterface
import threading
import numpy as np

class JointFrictionEstimatorNode(Node, ):
    def __init__(self):
        Node.__init__(self, "joint_friction_estimator")
        self.robot_if = Go2RobotInterface(self)

        # Procedure parameters
        self.joint_i = self.declare_parameter("joint").value
        self.torque_rate = self.declare_parameter("torque_rate", 0.5).value # Nm/s

        self.measure_sign = 1 if self.torque_rate > 0 else -1

        # Node state
        self.measure_friction = False
        self.measure_noise = False

        # Joint states
        self.tqva_i = None

        # Start listening
        self.robot_if.register_callback(self.__state_cb)
        self.robot_if.start_async([], goto_config=False)

        # Start procedure
        self.thread = threading.Thread(target=self.calibration_run, daemon=True)
        self.thread.start()



    def __state_cb(self, t, q, dq, ddq):
        q_i, v_i, a_i = q[self.joint_i], dq[self.joint_i], ddq[self.joint_i]
        zeros = [0.] * 12
        if(not self.robot_if.is_ready):
            return

        # To measure in the same direction as the torque is applied
        v_i_signed = self.measure_sign * v_i

        if(self.measure_noise):
            self.max_noise = max(self.max_noise, v_i_signed)

        if(not self.measure_friction):
            self.robot_if.send_command(zeros, zeros, zeros, zeros, zeros)
            self.t_start = t
            return

        if(v_i_signed > self.max_noise * 3.):
            self.measure_friction = False
            print(f"Static friction on joint {self.joint_i} estimated at {self.tau_i} Nm.")

        tau = [0.] * 12
        self.tau_i = (t - self.t_start) * self.torque_rate
        tau[self.joint_i] = self.tau_i
        self.robot_if.send_command(zeros, zeros, tau, zeros, zeros)

    def calibration_run(self):
        # Wait for robot to be ready
        while rclpy.ok():
            if(self.robot_if.is_ready):
                break

        # Wait for user to be ready
        print(f"""
                Put joint {self.joint_i} orthogonal to gravity, and set it to an appropriate angle.
              """)
        input("Press enter to start procedure...")

        # Measure noise level on joint only the first time
        print(f"Measuring noise level on joint {self.joint_i}...")
        self.max_noise = 0.0
        self.measure_noise = True
        self.get_clock().sleep_for(Duration(seconds=3.))
        self.measure_noise = False
        print(f"Max noise of {self.max_noise} rad/s on joint {self.joint_i}.")

        while(rclpy.ok()):
            print(f"Gradually increasing torque on joint {self.joint_i} with a rate of {self.torque_rate} Nm/s...")
            self.measure_friction = True

            # wait for measure to be finished
            while(self.measure_friction):
                self.get_clock().sleep_for(Duration(seconds=.1))
            input("Press enter to retry...")

def main(args=None):
    rclpy.init(args=args)
    node = JointFrictionEstimatorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
