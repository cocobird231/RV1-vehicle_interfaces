import rclpy
from rclpy.node import Node

from vehicle_interfaces.srv import SafetyReg
from vehicle_interfaces.srv import SafetyReq

from vehicle_interfaces.node_adaptor import NodeAdaptor

class SafetyNode(NodeAdaptor):
    def __init__(self, nodeName : str, safetyServiceName : str):
        NodeAdaptor.__init__(self, nodeName)
        self.__nodeEnableF = False
        if (safetyServiceName == ''):
            return
        self.__regClientNode = Node(nodeName + '_safetyreg_client')
        self.__regClient = self.__regClientNode.create_client(SafetyReg, safetyServiceName + "_Reg")
        print("[SafetyNode] Connecting to safetyreg server: %s\n" %(safetyServiceName + "_Reg"))
        self.__connToService(self.__regClient)

        self.__reqClientNode = Node(nodeName + '_safetyreq_client')
        self.__reqClient = self.__reqClientNode.create_client(SafetyReq, safetyServiceName + "_Req")
        print("[SafetyNode] Connecting to safetyreq server: %s\n" %(safetyServiceName + "_Req"))
        self.__connToService(self.__reqClient)

        self.__nodeEnableF = True
    
    def __connToService(self, client):
        errCnt = 5
        while (not client.wait_for_service(timeout_sec=0.5) and errCnt > 0):
            errCnt -= 1
            self.get_logger().info('[SafetyNode.__connToService] service not available, waiting again...')
        if (errCnt <= 0):
            self.get_logger().info('[SafetyNode.__connToService] Connect to service failed.')
        else:
            self.get_logger().info('[SafetyNode.__connToService] Service connected.')
    
    def setEmergency(self, devID, emP):
        if (not self.__nodeEnableF):
            return False
        request = SafetyReg.Request()
        request.device_id = devID
        request.emergency_percentage = emP
        future = self.__regClient.call_async(request)
        rclpy.spin_until_future_complete(self.__regClientNode, future, timeout_sec=0.01)
        if (not future.done()):
            self.get_logger().info('[SafetyNode.setEmergency] Failed to call service')
            return False
        
        response = future.result()
        if (response.response):
            return True
        return False
    
    # return float >= 0 if request succeed
    def getEmergency(self, devID):
        if (not self.__nodeEnableF):
            return -1
        request = SafetyReq.Request()
        request.device_id = devID
        future = self.__reqClient.call_async(request)
        rclpy.spin_until_future_complete(self.__reqClientNode, future, timeout_sec=0.01)
        if (not future.done()):
            self.get_logger().info('[SafetyNode.getEmergency] Failed to call service')
            return -1
        
        response = future.result()
        if (response.response and len(response.emergency_percentages) > 0):
            return response.emergency_percentages[0]
        return -1
    
    # return dict({id : emP})
    def getEmergencies(self):
        if (not self.__nodeEnableF):
            return dict()
        request = SafetyReq.Request()
        request.device_id = "all"
        future = self.__reqClient.call_async(request)
        rclpy.spin_until_future_complete(self.__reqClientNode, future, timeout_sec=0.01)
        if (not future.done()):
            self.get_logger().info('[SafetyNode.getEmergencies] Failed to call service')
            return dict()
        
        response = future.result()
        if (response.response):
            ret = dict()
            for devID, emP in zip(response.device_ids, response.emergency_percentages):
                ret[devID] = emP
            return ret
        return dict()
