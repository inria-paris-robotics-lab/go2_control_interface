#!/urs/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from go2_control_interface.sportsmode_shutdown import Go2Shutdown


class SportsModeShutdownService(Node):
    def __init__(self):
        super().__init__("sportsmode_shutdown_server")
        self.server_ = self.create_service(SetBool, "sportsmode_shutdown_service", self.service_callback)
        self.get_logger().info("Service server Python node has been created")
    
    def service_callback(self, request, response): 
        client = Go2Shutdown("enx00143d1488c7")
        client.SwitchOff()
        response.res = 0
        self.get_logger().info(f'Processed request shutdown')
        return response 

def main(args = None):
    rclpy.init(args=args)
    node = SportsModeShutdownService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()