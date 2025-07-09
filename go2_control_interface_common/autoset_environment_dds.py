#!/usr/bin/env python3

import sys
from go2_control_interface_common.autodetect_network_if import Go2NetworkInfo


class Go2EnvSetup():
    def generateEnv(self, sim: bool) -> None:
        go2_net_info = Go2NetworkInfo()
        ifname, ifip = go2_net_info.getGo2InterfaceNameIp()

        self.output (f'\033[1;4mExecuting:\033[0m', display_only=True)

        if(sim):
            self.output (f'unset RMW_IMPLEMENTATION')
            self.output (f'unset CYCLONEDDS_URI')
            self.output (f'export ROS_LOCALHOST_ONLY=1')

        else:
            self.output (f'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
            self.output (f'''export CYCLONEDDS_URI=\'<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="{ifname}" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>\'
                        ''')
            self.output (f'export ROS_LOCALHOST_ONLY=0')

    def output(self, str : str, *, display_only: bool = False) -> None:
        print(str, file=sys.stderr)
        if(not display_only):
            print(str)


def main(args=None):
    is_sim = None
    # Try to parse argument
    try:
        if(len(sys.argv) != 2):
            raise ValueError
        if(sys.argv[1] == "SIMULATION"):
            is_sim = True
        elif(sys.argv[1] == "REAL"):
            is_sim = False
        else:
            raise ValueError
    except ValueError:
        print("Error expecting exactly one argument being SIMULATION or REAL")
        exit()
    go2_env = Go2EnvSetup()
    go2_env.generateEnv(sim=is_sim)

if __name__=='__main__':
    main()
