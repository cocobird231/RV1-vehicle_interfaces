import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from vehicle_interfaces.msg import QosUpdate
from vehicle_interfaces.srv import QosReq

from vehicle_interfaces.node_adaptor import NodeAdaptor

class QoSUpdateNode(NodeAdaptor):
    def __init__(self, nodeName : str, qosServiceName : str):
        NodeAdaptor.__init__(self, nodeName)
        self.__nodeEnableF = False
        if (qosServiceName == ''):
            return
        self.__reqClientNode = Node(nodeName + '_qosreq_client')
        self.__reqClient = self.__reqClientNode.create_client(QosReq, qosServiceName + "_Req")
        print("[QoSUpdateNode] Connecting to qosreq server: %s\n" %(qosServiceName + "_Req"))
        self.__connToService(self.__reqClient)

        self.__subscription = self.create_subscription(QosUpdate, qosServiceName, self.__topic_callback, 10)

        self.__qosTopicNameVec = []
        self.__qosID = 0
        self.__qosCallbackFunc = None
        self.__callbackF = False
        
        self.__nodeEnableF = True

    def __topic_callback(self, msg):
        if (not self.__callbackF):# No callback function assigned
            return
        
        if (msg.qid == self.__qosID):# Ignore update in same qos ID
            return
        
        errF = False
        qmap = dict()

        for myTopic in self.__qosTopicNameVec:
            for newTopic in msg.topic_table:
                if (myTopic == newTopic):
                    try:
                        qmap[myTopic] = self.requestQoS(myTopic)
                    except:
                        errF = True
                    break
        
        if (errF):
            return
        
        if (len(qmap) > 0):
            self.__qosCallbackFunc(this, qmap)
        
        self.__qosID = msg.qid

    def __connToService(self, client):
        errCnt = 5
        while (not client.wait_for_service(timeout_sec=0.5) and errCnt > 0):
            errCnt -= 1
            self.get_logger().info('[QoSUpdateNode.__connToService] service not available, waiting again...')
        if (errCnt <= 0):
            self.get_logger().info('[QoSUpdateNode.__connToService] Connect to service failed.')
        else:
            self.get_logger().info('[QoSUpdateNode.__connToService] Service connected.')
    
    def __splitTime(time_ms : float):
        return Duration(seconds=time_ms / 1000, nanoseconds=(time_ms - int(time_ms)) * 1000000)

    def addQoSTracking(self, topicName : str):
        if (not self.__nodeEnableF):
            return
        for i in self.__qosTopicNameVec:
            if (i == topicName):
                return
        self.__qosTopicNameVec.append(topicName)
        self.__qosID = 0

    def addQoSCallbackFunc(self, func):
        if (not self.__nodeEnableF):
            return
        self.__qosCallbackFunc = func
        self.__callbackF = True

    # Return QoSProfile
    def requestQoS(self, topicName : str):
        if (self.__nodeEnableF):
            raise "Request QoS Failed"# Request QoS failed
        request = QosReq.Request()
        request.topic_name = topicName
        future = self.__reqClient.call_async(request)
        rclpy.spin_until_future_complete(self.__reqClientNode, future, timeout_sec=0.1)
        if (not future.done()):
            self.get_logger().info('[SafetyNode.getEmergency] Failed to call service')
            raise "Request QoS Failed"# Request QoS failed
        
        response = future.result()
        if (response.response):
            prof = QoSProfile(history=response.history)
            prof.history = response.history
            prof.depth = response.depth
            prof.reliability = response.reliability
            prof.durability = response.durability
            prof.deadline = self.__splitTime(response.deadline_ms)
            prof.lifespan = self.__splitTime(response.lifespan_ms)
            prof.liveliness = response.liveliness
            prof.liveliness_lease_duration = self.__splitTime(response.liveliness_lease_duration_ms)

            # self.__qosID = response.qid;
            return prof
        raise "Request QoS Failed";# Request QoS failed
