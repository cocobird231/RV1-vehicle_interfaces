import os
import json
import time
import threading

import rclpy
from rclpy.node import Node

from vehicle_interfaces.msg import DevInfo
from vehicle_interfaces.srv import DevInfoReg

from vehicle_interfaces.node_adaptor import NodeAdaptor
from vehicle_interfaces.utils import ConnToService


class DevInfoNode(NodeAdaptor):
    def __init__(self, nodeName : str, devInfoServiceName : str, devInterface : str, devMultiNode : bool):
        NodeAdaptor.__init__(self, nodeName)
        self.__nodeEnableF = False
        if (devInfoServiceName == ''):
            return

        self.__nodeName = nodeName
        self.__ifName = devInterface
        self.__hostname = ''
        self.__ipv4Addr = ''
        self.__ipv6Addr = ''
        self.__macAddr = ''
        self.__multiNodeF = devMultiNode
        self.__regClientStopF = False
        self.__reqEnableF = False
        
        self.__regClientNode = Node(nodeName + '_qosreg_client')
        self.__regClient = self.__regClientNode.create_client(DevInfoReg, devInfoServiceName + "_Reg")

        self.__regClientTh = threading.Thread(target=self.__waitService)
        self.__regClientTh.start()
        
        self.__nodeEnableF = True
    
    def __del__(self):
        self.__reqEnableF = False
        self.__regClientStopF = True
        self.__regClientTh.join()
    
    def __waitService(self):
        try:
            while (not self.__getHostname() and not self.__regClientStopF):
                time.sleep(1)
            while (not self.__getIPv4Addr() and not self.__regClientStopF):
                time.sleep(1)
            while (not self.__getMACAddr() and not self.__regClientStopF):
                time.sleep(1)
            ConnToService(self.__regClient, 10, -1)
            self.__reqEnableF = True
            
            while (not self.regDevInfo() and not self.__regClientStopF):
                time.sleep(1)
        except:
            self.get_logger().error('[DevInfoNode.__waitService] Caught unexpected errors.')

    def __getHostname(self):
        fp = os.popen("hostname | awk '{print \"^\"$0\"!\"}'")
        if (fp != None):
            rd = fp.read()
            fp.close()

        subStr = rd[rd.find('^') + 1 : rd.rfind('!')]
        if (len(subStr) > 0):
            self.__hostname = subStr
            return True
        return False
    
    def __getIPv4Addr(self):
        fp = os.popen("ip addr show dev %s | grep -Po \"(?<=inet )((\\d{1,3}\\.){3}\\d{1,3})\" | awk '{print \"^\"$0\"!\"}'" %(self.__ifName))
        if (fp != None):
            rd = fp.read()
            fp.close()

        subStr = rd[rd.find('^') + 1 : rd.rfind('!')]
        if (len(subStr) > 0):
            self.__ipv4Addr = subStr
            return True
        return False
    
    def __getMACAddr(self):
        fp = os.popen("ip addr show dev %s | grep -Po \"(?<=link/ether )(([A-Za-z0-9]{2}:){5}[A-Za-z0-9]{2})(?= brd)\" | awk '{print \"^\"$0\"!\"}'" %(self.__ifName))
        if (fp != None):
            rd = fp.read()
            fp.close()

        subStr = rd[rd.find('^') + 1 : rd.rfind('!')]
        if (len(subStr) > 0):
            self.__macAddr = subStr
            return True
        return False
    
    def regDevInfo(self):
        if (not self.__nodeEnableF and not __reqEnableF):
            return False

        if ((len(self.__ipv4Addr) <= 0 and len(self.__ipv6Addr) <= 0) or len(self.__macAddr) <= 0 or len(self.__hostname) <= 0):
            return False
        
        nodeName = self.__nodeName
        if (len(self.get_namespace()) > 0):
            if (nodeName.find(self.get_namespace()) < 0):# nodeName not include namespace
                if (nodeName[0] == '/'):
                    nodeName = self.get_namespace() + nodeName
                else:
                    nodeName = self.get_namespace() + '/' + nodeName
        
        msg = DevInfo()
        msg.node_name = nodeName
        msg.hostname = self.__hostname
        msg.mac_addr = self.__macAddr
        msg.ipv4_addr = self.__ipv4Addr
        msg.ipv6_addr = self.__ipv6Addr
        msg.multi_node = self.__multiNodeF

        request = DevInfoReg.Request()
        request.dev_info = msg
        future = self.__regClient.call_async(request)
        rclpy.spin_until_future_complete(self.__regClientNode, future, timeout_sec=0.5)
        if (future.done()):
            response = future.result()
            return response.response
        return False
