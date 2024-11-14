import socket
import fcntl
import struct
import os
from typing import Optional

class Go2NetworkInfo():
    def __init__(self):
        super().__init__()

    def getIPInfo(self, ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            ip_address = fcntl.ioctl(
                s.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack('256s', ifname[:15].encode('utf-8'))
            )[20:24]
            return socket.inet_ntoa(ip_address)
        except IOError:
            return None

    def getGo2InterfaceName(self):
        # Return a list of network interface information
        nameindex_array = socket.if_nameindex()

        for index, ifname in nameindex_array:
            ipaddr = self.getIPInfo(ifname)
            if ipaddr!=None:
                if (ipaddr.startswith("192.168.123")):
                    return ifname
        return None 
    
    def getGo2InterfaceIP(self):
        # Return a list of network interface information
        nameindex_array = socket.if_nameindex()

        for index, ifname in nameindex_array:
            ipaddr = self.getIPInfo(ifname)
            if ipaddr!=None:
                if (ipaddr.startswith("192.168.123")):
                    return ipaddr
        return None 
    

    def isNettworkUp(self):
        if self.getGo2InterfaceName() is None:
            return False
        
        return True

    # hardcoded IP address
    def isRobotUp(self):
        ret = os.system("ping -q -c 2 -W 1 -t 100 192.168.123.18")
        if ret != 0:
            # not up
            return False
        return True

    def isJetsonUp(self):
        ret = os.system("ping -q -c 2 -W 1 -t 50 192.168.123.161")
        if ret != 0:
            # not up
            return False
        return True


def main(args = None):
    go2_net = Go2NetworkInfo()
    go2_ifname = go2_net.getGo2InterfaceName()
    go2_ifip = go2_net.getGo2InterfaceIP()
    go2_ifnet_up = go2_net.isNettworkUp()
    go2_robot_up = go2_net.isRobotUp()
    go2_jetson_up = go2_net.isJetsonUp()

    print(f'DDS Interface name {go2_ifname} IP address {go2_ifip} id up {go2_ifnet_up} with Go2 active {go2_robot_up} and Jetson active {go2_jetson_up}')

if __name__=='__main__':
    main()
