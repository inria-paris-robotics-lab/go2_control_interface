import socket
import fcntl
import struct
import os, sys
from typing import Optional

from go2_netwif import Go2NetworkInfo

import importlib, pathlib
import importlib.util

class Go2EnvSetup():


    def __init__(self):
        super().__init__()

    def displayCurrentEnv(self):
        script_path = pathlib.Path(__file__).parent.resolve()
        process_path = pathlib.Path().resolve()
        print (f'Script in location {script_path}', file=sys.stderr)
        print (f'Process in location {process_path}', file=sys.stderr)

        dds_impl = os.environ.get('RMW_IMPLEMENTATION', None)
        dds_uri = os.environ.get('CYCLONEDDS_URI', None)
        ros_version = os.environ.get('ROS_DISTRO', None)
        print ("\nCurrent env:", file=sys.stderr)
        print (f'  - RMW_IMPLEMENTATION {dds_impl}', file=sys.stderr)
        print (f'  - CYCLONEDDS_URI {dds_uri}', file=sys.stderr)
        print (f'  - ROS_DISTRO {ros_version}', file=sys.stderr)

        print ('\nPython packages', file=sys.stderr)
        python_packages = ["unitree_go", "unitree_sdk2py", "pinocchio", "eigenpy", "proxsuite_nlp", "aligator", "simple_mpc"]
        for pypkg in python_packages:
            loader = importlib.util.find_spec(pypkg)
            found = loader is not None
            print (f'  - {pypkg} is {found}', file=sys.stderr)

        print ('\nPython paths', file=sys.stderr)
        python_paths = os.environ.get('PYTHONPATH', "").split(":")
        for pypath in python_paths:
            print (f'  - {pypath}', file=sys.stderr)

    def generateEnv(self):
        script_path = pathlib.Path(__file__).parent.resolve()
        process_path = pathlib.Path().resolve()

        go2_net_info = Go2NetworkInfo()
        ifname = go2_net_info.getGo2InterfaceName()

        print (f'\nSetup commands\n', file=sys.stderr)
        print (f'source {script_path}/../../../install.bash;')
        print (f'export CYCLONEDDS_HOME={script_path}/../../../cyclonedds/;')
        print (f'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;')
        print (f'export CYCLONEDDS_URI=\'<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="{ifname}" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>\';')

def main(args = None):
    go2_env = Go2EnvSetup()
    go2_env.displayCurrentEnv()
    go2_env.generateEnv()

if __name__=='__main__':
    main()
