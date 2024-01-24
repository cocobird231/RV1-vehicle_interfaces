import rclpy
from rclpy.node import Node

from vehicle_interfaces.srv import SafetyReg
from vehicle_interfaces.srv import SafetyReq
from vehicle_interfaces.msg import SurroundEmergency
from vehicle_interfaces.node_adaptor import NodeAdaptor

class SafetyNode(NodeAdaptor):
    def __init__(self, nodeName : str, safetyServiceName : str):
        NodeAdaptor.__init__(self, nodeName)
        self.__nodeEnableF = False
        if (safetyServiceName == ''):
            self.get_logger().warning('[SafetyNode] Ignored.')
            return

        self.__regClientNode = Node(nodeName + '_safetyreg_client')
        self.__regClient = self.__regClientNode.create_client(SafetyReg, safetyServiceName + "_Reg")

        self.__reqClientNode = Node(nodeName + '_safetyreq_client')
        self.__reqClient = self.__reqClientNode.create_client(SafetyReq, safetyServiceName + "_Req")

        self.__nodeEnableF = True
        self.get_logger().info('[SafetyNode] Constructed.')

    def setEmergency(self, devID : str, emP : float, direction : int, lifetime_ms : float = 500.0):
        """Register emergency score to SafetyServer with specific direction.

        Parameters
        ----------
        devID : `str`
        The name to register to SafetyServer, e.g. the node name.

        emP : `float`
        The emergency score.

        direction : `int`
        The direction where emergency occurs.

        lifetime_ms : `float`
        The lifetime of `emP` score.

        Returns
        -------
        ret : `bool`
        `True` if request success. Otherwise, `False`.
        """
        if (not self.__nodeEnableF):
            return False
        score = SurroundEmergency()
        score.emergency_percentages[direction] = emP
        score.lifetime_ms = lifetime_ms
        request = SafetyReg.Request()
        request.device_id_vec.append(devID)
        request.emergency_scores.append(score)

        future = self.__regClient.call_async(request)
        rclpy.spin_until_future_complete(self.__regClientNode, future, timeout_sec=0.2)
        if (not future.done()):
            self.get_logger().error('[SafetyNode.setEmergency] Failed to call service')
            return False

        response = future.result()
        if (response.response):
            return True
        return False

    def getEmergency(self, devID : str, direction : int):
        """Request emergency score from SafetyServer with specific device ID and direction.

        Parameters
        ----------
        devID : `str`
        Either node name or "nearest". Do not pass "all" to prevent unexpected result.

        direction : `int`
        Specify the direction according to EmergencyScoreDirection.

        Returns
        -------
        outEmP : `float`
        Emergency value of specific direction. Return -1 if request failed.
        """
        if (not self.__nodeEnableF):
            return -1
        request = SafetyReq.Request()
        request.device_id = devID
        future = self.__reqClient.call_async(request)
        rclpy.spin_until_future_complete(self.__reqClientNode, future, timeout_sec=0.2)
        if (not future.done()):
            self.get_logger().error('[SafetyNode.getEmergency] Failed to call service')
            return -1

        response = future.result()
        if (response.response and len(response.device_id_vec) > 0):
            return response.emergency_scores[-1].emergency_percentages[direction]
        return -1

    def getEmergency(self, devID : str):
        """Request emergency scores from SafetyServer with specific device ID.

        Parameters
        ----------
        devID : `str`
        Either node name or "nearest". Do not pass "all" to prevent unexpected result.

        Returns
        -------
        outEmP : `float`
        8-direction emergencies. Return `list()` if request failed.
        """
        if (not self.__nodeEnableF):
            return list()
        request = SafetyReq.Request()
        request.device_id = devID
        future = self.__reqClient.call_async(request)
        rclpy.spin_until_future_complete(self.__reqClientNode, future, timeout_sec=0.2)
        if (not future.done()):
            self.get_logger().error('[SafetyNode.getEmergency] Failed to call service')
            return list()

        response = future.result()
        if (response.response and len(response.device_id_vec) > 0):
            return response.emergency_scores[-1].emergency_percentages
        return list()

    def getEmergencies(self):
        """Request all emergency scores from SafetyServer.

        Returns
        -------
        emergencies : `dict()`
        Output full emergencies information formed by map which node name and SurroundEmergency were related. 
        Return empty dict() if request failed.
        """
        if (not self.__nodeEnableF):
            return dict()
        request = SafetyReq.Request()
        request.device_id = "all"
        future = self.__reqClient.call_async(request)
        rclpy.spin_until_future_complete(self.__reqClientNode, future, timeout_sec=0.2)
        if (not future.done()):
            self.get_logger().error('[SafetyNode.getEmergencies] Failed to call service')
            return dict()

        response = future.result()
        if (response.response):
            ret = dict()
            for devID, emP in zip(response.device_id_vec, response.emergency_scores):
                ret[devID] = emP
            return ret
        return dict()
