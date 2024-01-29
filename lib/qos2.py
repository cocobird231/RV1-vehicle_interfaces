import os
import json
import threading

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from vehicle_interfaces.msg import QosUpdate
from vehicle_interfaces.srv import QosReq

from vehicle_interfaces.node_adaptor import NodeAdaptor
from vehicle_interfaces.utils import TopicNames

# double ms to rmw_time_s
def CvtMsgToRMWTime(time_ms : float) -> Duration:
    return Duration(seconds=time_ms / 1000, nanoseconds=(time_ms - int(time_ms)) * 1000000)

# rmw_time_s to double ms
def CvtRMWTimeToMsg(rmwT : Duration) -> float:
    return rmwT.nanoseconds / 1000000.0

# Return true if succeed, otherwise, false
def DumpRMWQoSToJSON(qosFilePath : str , profile : QoSProfile) -> bool:
    try:
        d = {}
        d["history"] = int(profile.history)
        d["depth"] = profile.depth
        d["reliability"] = int(profile.reliability)
        d["durability"] = int(profile.durability)
        d["deadline_ms"] = CvtRMWTimeToMsg(profile.deadline)
        d["lifespan_ms"] = CvtRMWTimeToMsg(profile.lifespan)
        d["liveliness"] = int(profile.liveliness)
        d["liveliness_lease_duration_ms"] = CvtRMWTimeToMsg(profile.liveliness_lease_duration)
        jFile = json.dumps(d)
        with open(qosFilePath, 'w') as f:
            f.write(jFile)
        return True
    except:
        return False

# Return QoSProfile if succeed, otherwise, None
def LoadRMWQoSFromJSON(qosFilePath : str) -> QoSProfile:
    try:
        with open(qosFilePath, 'r') as f:
            d = json.load(f)
        profile = QoSProfile(history=0)
        profile.history = d["history"]
        profile.depth = d["depth"]
        profile.reliability = d["reliability"]
        profile.durability = d["durability"]
        profile.deadline = CvtMsgToRMWTime(d["deadline_ms"])
        profile.lifespan = CvtMsgToRMWTime(d["lifespan_ms"])
        profile.liveliness = d["liveliness"]
        profile.liveliness_lease_duration = CvtMsgToRMWTime(d["liveliness_lease_duration_ms"])
        return profile
    except:
        return None

# The qmap in python version defined as { str : rclpy.QoSProfile }, 
# the rclpy.QoSProfile can be directly passed into create_publisher() and create_subscription().

class TopicProp(Node):
    def __init__(self, msgType, topicName):
        self.msgType = msgType
        self.topicName = topicName
        self.locker = threading.Lock()

# QoSUpdateNode will raised the callback function, which added by addQoSCallbackFunc, while qos profile updated.
class QoSUpdateNode(NodeAdaptor):
    def __init__(self, nodeName : str, qosServiceName : str, qosDirPath : str):
        NodeAdaptor.__init__(self, nodeName)
        self.__nodeEnableF = False
        if (qosServiceName == ''):
            return

        # Create directory
        try:
            os.makedirs(qosDirPath)
        except FileExistsError:
            self.get_logger().info('[QoSUpdateNode] qosDirPath exists')

        self.__reqClientNode = Node(nodeName + '_qosreq_client')
        self.__reqClient = self.__reqClientNode.create_client(QosReq, qosServiceName + "_Req")

        self.__subscription = self.create_subscription(QosUpdate, qosServiceName, self.__topic_callback, 10)

        self.__qosTopicNameVec = set()
        self.__qosID = 0

        self.__qosDirPath = qosDirPath

        self.__pubDict = dict()# { topicName : [pub, TopicProp] }
        self.__subDict = dict()# { topicName : [sub, TopicProp] }

        self.__nodeEnableF = True

    def __topic_callback(self, msg):       
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
            self.__qosCallback(qmap)

            # Dump QoS profile
            for k in qmap:
                tn = k.replace('/', '_')
                DumpRMWQoSToJSON(os.path.join(self.__qosDirPath, tn + '.json'), qmap[k])
                self.get_logger().info('[QoSUpdateNode.requestQoS] Dump QoS profile: %s' \
                    %os.path.join(self.__qosDirPath, tn + '.json'))

        self.__qosID = msg.qid
    
    def __qosCallback(self, qmap):
        for topic in qmap:# Consider topic as fullName
            self.get_logger().info('[QoSUpdateNode.__qosCallback] Get qmap[%s]' %topic)
            if (self.__pubDict.get(topic) is not None):
                tp = self.__pubDict[topic][1]# TODO: [pub, TopicProp, lock]
                self.__pubDict[topic][1].locker.acquire()
                try:
                    # self.__pubDict[topic][0].destroy()
                    del self.__pubDict[topic][0]
                    self.get_logger().info('[QoSUpdateNode.__qosCallback] Publisher destroyed')
                except:
                    self.get_logger().info('[QoSUpdateNode.__qosCallback] Caught exception while destroying publisher')
                self.__pubDict[topic][1].locker.release()

                self.__pubDict[topic][1].locker.acquire()
                self.__publisher = self.create_publisher(self.__pubDict[topic][1].msgType, self.__pubDict[topic][1].topicName, qmap[topic])
                self.__pubDict[topic][1].locker.release()
                self.get_logger().info('[QoSUpdateNode.__qosCallback] New publisher')
    
    def __splitTime(self, time_ms : float):
        return Duration(seconds=time_ms / 1000, nanoseconds=(time_ms - int(time_ms)) * 1000000)
    
    def createTrackingPublisher(self, msgType, topicName):
        if (not self.__nodeEnableF):
            raise "Create Tracking Publisher Failed"# Request QoS failed
        
        topicNames = TopicNames(topicName, self.get_namespace())
        topicName = topicNames.topicName
        fullName = topicNames.fullName

        self.__qosTopicNameVec.add(fullName)

        if (self.__pubDict.get(fullName) is not None):
            self.get_logger().warning('[QoSUpdateNode.createTrackingPublisher] Topic name exists in track list: %s' %fullName)
            return False

        # Load QoS profile
        tn = fullName.replace('/', '_')# Topic name trans
        prof = LoadRMWQoSFromJSON(os.path.join(self.__qosDirPath, tn + '.json'))
        if (prof != None):
            self.get_logger().info('[QoSUpdateNode.createTrackingPublisher] Found QoS profile: depth: %d reliability: %d durability: %d' \
                %(prof.depth, prof.reliability, prof.durability))
        else:
            self.get_logger().info('[QoSUpdateNode::createTrackingPublisher] QoS profile not found. Set to default.')
            prof = QoSProfile(depth=10)
        
        # Create publisher
        self.__pubDict[fullName] = [self.create_publisher(msgType, topicName, prof), TopicProp(msgType, topicName)]
        return True
    
    def publish(self, topicName, msg):
        nameDict = TopicNames.getFullNames(topicName, self.get_namespace())
        fullName = nameDict["fullName"]
        if (self.__pubDict.get(fullName) is not None):
            self.__pubDict[fullName][1].locker.acquire()
            self.__pubDict[fullName][0].publish(msg)
            self.__pubDict[fullName][1].locker.release()

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
            prof = QoSProfile(depth=msg.depth)
            prof.history = msg.history
            prof.reliability = msg.reliability
            prof.durability = msg.durability
            # prof.deadline = self.__splitTime(msg.deadline_ms)
            # prof.lifespan = self.__splitTime(msg.lifespan_ms)
            # prof.liveliness = msg.liveliness
            # prof.liveliness_lease_duration = self.__splitTime(msg.liveliness_lease_duration_ms)

            # self.__qosID = response.qid;
            return prof
        raise "Request QoS Failed";# Request QoS failed
