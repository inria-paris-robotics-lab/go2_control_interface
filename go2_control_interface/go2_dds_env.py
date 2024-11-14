import socket
import fcntl
import struct
import os
from typing import Optional

from go2_netwif import Go2NetworkInfo

import importlib


class Go2EnvSetup():

    ROOT_UNITREE = "/root/devel/unitree_ws/"
    ROUT_GO2_MCP = "/root/devel/go2_mpc_ws/"


    def __init__(self):
        super().__init__()

    def displayCurrentEnv(self):

        dds_impl = os.environ['RMW_IMPLEMENTATION']
        dds_uri = os.environ['CYCLONEDDS_URI']
        ros_version = os.environ['ROS_DISTRO']
        print ("\nCureent env:")
        print (f'  - RMW_IMPLEMENTATION {dds_impl}')
        print (f'  - CYCLONEDDS_URI {dds_uri}')
        print (f'  - ROS_DISTRO {ros_version}')

        print ('\nPython packages')
        python_packages = ["unitree_go", "unitree_sdk2py", "pinocchio", "eigenpy", "proxsuite_nlp", "aligator", "simple_mpc"]
        for pypkg in python_packages:
            loader = importlib.util.find_spec(pypkg)
            found = loader is not None
            print (f'  - {pypkg} is {found}')

        print ('\nPython paths')
        python_paths = os.environ['PYTHONPATH'].split(":")
        for pypath in python_paths:
            print (f'  - {pypath}')

    def generateEnv(self):
        go2_net_info = Go2NetworkInfo()
        ifname = go2_net_info.getGo2InterfaceName()

        print (f'\nSetup commands\n')
        print (f'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
        print (f'export CYCLONEDDS_URI=\'<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="{ifname}" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>\'')
        print (f'source {self.ROOT_UNITREE}')
        print (f'source {self.ROUT_GO2_MCP}')

def main(args = None):
    go2_env = Go2EnvSetup()
    go2_env.displayCurrentEnv()
    go2_env.generateEnv()

if __name__=='__main__':
    main()
