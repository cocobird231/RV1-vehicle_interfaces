import rclpy
from rclpy.node import Node

from vehicle_interfaces.srv import SafetyReg
from vehicle_interfaces.srv import SafetyReq

class SafetyNode(Node):
    def __init__(self, nodeName : str, safetyServiceName : str):
        super().__init__(nodeName)
        self.regClientNode_ = Node(nodeName + '_safetyreg_client')
        self.regClient_ = self.regClientNode_.create_client(SafetyReg, safetyServiceName + "_Reg")
        print("[SafetyNode] Connecting to safetyreg server: %s\n" %(safetyServiceName + "_Reg"))
        self.connToService(self.regClient_)

        self.reqClientNode_ = Node(nodeName + '_safetyreq_client')
        self.reqClient_ = self.reqClientNode_.create_client(SafetyReq, safetyServiceName + "_Req")
        print("[SafetyNode] Connecting to safetyreq server: %s\n" %(safetyServiceName + "_Req"))
        self.connToService(self.reqClient_)
    
    def connToService(self, client):
        while (not client.wait_for_service(timeout_sec=0.5)):
            self.get_logger().info('[SafetyNode.connToService] service not available, waiting again...')
        self.get_logger().info('[SafetyNode.connToService] service connected.')
    
    def setEmergency(self, devID, emP):
        request = SafetyReg.Request()
        request.device_id = devID
        request.emergency_percentage = emP
        future = self.regClient_.call_async(request)
        rclpy.spin_until_future_complete(self.regClientNode_, future, timeout_sec=0.01)
        if (not future.done()):
            self.get_logger().info('[SafetyNode.setEmergency] Failed to call service')
            return False
        
        response = future.result()
        if (response.response):
            return True
        return False
    
    # return float >= 0 if request succeed
    def getEmergency(self, devID):
        request = SafetyReq.Request()
        request.device_id = devID
        future = self.reqClient_.call_async(request)
        rclpy.spin_until_future_complete(self.reqClientNode_, future, timeout_sec=0.01)
        if (not future.done()):
            self.get_logger().info('[SafetyNode.getEmergency] Failed to call service')
            return -1
        
        response = future.result()
        if (response.response and len(response.emergency_percentages) > 0):
            return response.emergency_percentages[0]
        return -1
    
    # return dict({id : emP})
    def getEmergencies(self):
        request = SafetyReq.Request()
        request.device_id = "all"
        future = self.reqClient_.call_async(request)
        rclpy.spin_until_future_complete(self.reqClientNode_, future, timeout_sec=0.01)
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
