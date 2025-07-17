import socket
import fcntl
import struct
import subprocess


class Go2NetworkInfo:
    ROBOT_SUBNET = "192.168.123"
    GO2_HOST = ".18"
    JETSON_HOST = ".161"

    def getIPInfo(self, ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            ip_address = fcntl.ioctl(
                s.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack("256s", ifname[:15].encode("utf-8")),
            )[20:24]
            return socket.inet_ntoa(ip_address)
        except IOError:
            return None

    def getGo2InterfaceNameIp(self):
        # Return a list of network interface information
        nameindex_array = socket.if_nameindex()

        for index, ifname in nameindex_array:
            ipaddr = self.getIPInfo(ifname)
            if ipaddr is not None:
                if ipaddr.startswith(self.ROBOT_SUBNET):
                    return ifname, ipaddr
        return None, None

    # hardcoded IP address
    def ping(self, host):
        res = subprocess.run(
            ["ping", "-q", "-c", "2", "-W", "1", "-t", "100", self.ROBOT_SUBNET + host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if res.returncode != 0:
            # not up
            return False
        return True


def main(args=None):
    go2_net = Go2NetworkInfo()
    ifname, ifip = go2_net.getGo2InterfaceNameIp()
    print(f"DDS Interface name: {ifname}")
    print(f"IP address:         {ifip}")

    print("\nPinging devices:")
    robot_up = go2_net.ping(go2_net.GO2_HOST)
    jetson_up = go2_net.ping(go2_net.JETSON_HOST)
    print(f"Robot  is {'  ' if robot_up else 'NOT'} reachable ({go2_net.ROBOT_SUBNET + go2_net.GO2_HOST})")
    print(f"Jetson is {'  ' if jetson_up else 'NOT'} reachable ({go2_net.ROBOT_SUBNET + go2_net.GO2_HOST})")


if __name__ == "__main__":
    main()
