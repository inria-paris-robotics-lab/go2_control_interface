#!/usr//bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_srvs.srv import SetBool
from go2_control_interface.sportsmode_shutdown import Go2Shutdown


class SportsModeShutdownService(Node):
    def __init__(self):
        super().__init__("sportsmode_shutdown_server")

        # Declare the parameter and read it to force it to be defined
        self.declare_parameter('if_name', descriptor = ParameterDescriptor(description='Network interface to connect to the robot.', type=ParameterType.PARAMETER_STRING))
        if_name = self.get_parameter('if_name').get_parameter_value().string_value

        # Declare service
        self.server_ = self.create_service(SetBool, "sportsmode_shutdown_service", self.service_callback)

        # Log
        self.get_logger().info("sportsmode_shutdown_service created")

    def service_callback(self, request, response):
        # Read if_name from parameter (in case of change)
        if_name = self.get_parameter('if_name').get_parameter_value().string_value

        # Connecting SDK
        self.get_logger().info(f"Connecting unitree sdk to interface '{if_name}'")
        client = Go2Shutdown(if_name)

        # Turning off sports mode
        self.get_logger().info(f"Switching off sportsmode.")
        client.SwitchOff()

        # Success
        response.res = 0
        return response

def main(args = None):
    rclpy.init(args=args)
    node = SportsModeShutdownService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()