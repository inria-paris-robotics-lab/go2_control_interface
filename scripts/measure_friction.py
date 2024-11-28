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
        self.v_thresh = self.declare_parameter("v_thresh", 0.1).value
        self.torque_rate = self.declare_parameter("torque_rate", 0.5).value # N/s
        self.start_measure = False
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

        if(not self.start_measure):
            self.robot_if.send_command(zeros, zeros, zeros, zeros, zeros)
            self.t_start = t
            return

        if(abs(v_i) > self.v_thresh):
            self.start_measure = False
            print(f"Static friction on joint {self.joint_i} estiamted at {self.tau_i} N")

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

        while(rclpy.ok()):
            print(f"Gradually increasing torque on joint {self.joint_i} with a rate of {self.torque_rate} N/s...")
            self.start_measure = True
            while(self.start_measure):
                self.get_clock().sleep_for(Duration(seconds=.1))
                # wait for measure to be finished
            input("Press enter to retry...")

def main(args=None):
    rclpy.init(args=args)
    node = JointFrictionEstimatorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
