import time
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

class Go2Shutdown():

    def __init__(self, netwif):
        self.completed = False
        ChannelFactoryInitialize(0, netwif)

    # Public methods
    def SwitchOff(self):
        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()

        while result['name']:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        self.completed = True
        return 0


def main(args = None):
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    ifname = ""

    if len(sys.argv)==1:
        print("Script requires the network interface used to connect to the Go2")
        ifname = "enx00143d1488c7"
        # sys.exit(-1)
    else:
        ifname = sys.argv[1]

    client = Go2Shutdown (ifname)
    client.SwitchOff()

    while True:        
        if client.completed == 1.0: 
           time.sleep(1)
           print("Done!")
           sys.exit(-1)     
        time.sleep(1)
        
if __name__=='__main__':
    main()