import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from vehicle_interfaces.msg import QosUpdate
from vehicle_interfaces.srv import QosReq

from vehicle_interfaces.node_adaptor import NodeAdaptor

# The qmap in python version defined as { str : rclpy.QoSProfile }, 
# the rclpy.QoSProfile can directly pass into create_publisher() and create_subscription().

# QoSUpdateNode will raised the callback function, which added by addQoSCallbackFunc, while qos profile updated.
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
        
        self.get_logger().info('[QoSUpdateNode.__topic_callback] qid: %d' %msg.qid)

        topicVec = []
        for myTopic in self.__qosTopicNameVec:
            for newTopic in msg.topic_table:
                if (myTopic == newTopic):
                    topicVec.append(myTopic)
                    break

        qmap = dict()
        for topic in topicVec:
            try:
                qmap[topic] = self.requestQoS(topic)
            except:
                self.get_logger().info('[QoSUpdateNode.requestQoS] Request error: %s' %topic)
        
        if (len(qmap) > 0):
            self.__qosCallbackFunc(qmap)
        
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
    
    def __splitTime(self, time_ms : float):
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
        if (not self.__nodeEnableF):
            raise "Request QoS Failed"# Request QoS failed
        request = QosReq.Request()
        request.topic_name = topicName
        future = self.__reqClient.call_async(request)
        rclpy.spin_until_future_complete(self.__reqClientNode, future, timeout_sec=0.1)
        if (not future.done()):
            self.get_logger().info('[QoSUpdateNode.requestQoS] Failed to call service')
            raise "Request QoS Failed"# Request QoS failed
        
        response = future.result()
        self.get_logger().info('[QoSUpdateNode.requestQoS] Response: %d' %response.response)
        if (response.response):
            msg = response.qos_profile
            prof = QoSProfile(history=msg.history)
            prof.history = msg.history
            prof.depth = msg.depth
            prof.reliability = msg.reliability
            prof.durability = msg.durability
            prof.deadline = self.__splitTime(msg.deadline_ms)
            prof.lifespan = self.__splitTime(msg.lifespan_ms)
            prof.liveliness = msg.liveliness
            prof.liveliness_lease_duration = self.__splitTime(msg.liveliness_lease_duration_ms)

            # self.__qosID = response.qid;
            return prof
        raise "Request QoS Failed";# Request QoS failed
