#!/usr/bin/env python3

import sys

from go2_utils.autodetect_network_if import Go2NetworkInfo


class Go2EnvSetup():
    def generateEnv(self):
        go2_net_info = Go2NetworkInfo()
        ifname, ifip = go2_net_info.getGo2InterfaceNameIp()

        print (f'\nSetup commands\n', file=sys.stderr)
        print (f'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
        print (f'''export CYCLONEDDS_URI=\'<CycloneDDS><Domain><General><Interfaces>
                         <NetworkInterface name="{ifname}" priority="default" multicast="default" />
                       </Interfaces></General></Domain></CycloneDDS>\'
               ''')

def main(args = None):
    go2_env = Go2EnvSetup()
    go2_env.generateEnv()

if __name__=='__main__':
    main()
