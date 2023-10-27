from enum import Enum
import time
import numpy as np
import threading
import multiprocessing

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from vehicle_interfaces.msg import Header
from vehicle_interfaces.srv import TimeSync

from vehicle_interfaces.node_adaptor import NodeAdaptor
from vehicle_interfaces.utils import ConnToService

# TODO: Need to be improved
class Timer():
    def __init__(self, interval_ms : float, cb) -> None:
        self.__interval_ms = interval_ms
        self.__func = cb
        self.__timerTH = threading.Thread(target=self._startTimerTH)

        self.__activateF = False
        self.__exitF = False

        self.__funcCallableF = True
        self.__funcTH = threading.Thread(target=self.__func)

        self.__timerTH.start()
    
    def __del__(self):
        self.destroy()
    
    def _callback(self):
        self.__funcCallableF = False
        self.__func()
        self.__funcCallableF = True
    
    def _timer_tick(self):
        if (not self.__exitF and self.__activateF and self.__funcCallableF):
            if (self.__funcTH.is_alive()):
                self.__funcTH.join()
            self.__funcTH = threading.Thread(target=self._callback)
            self.__funcTH.start()

    def _startTimerTH(self):
        while (not self.__exitF):
            if (self.__activateF):
                timer_ = threading.Timer(self.__interval_ms / 1000.0, self._timer_tick)
                timer_.start()
                timer_.join()

    def start(self) -> None:
        self.__activateF = True
    
    def stop(self) -> None:
        self.__activateF = False
    
    def destroy(self):
        self.__activateF = False
        self.__exitF = True
        self.__timerTH.join()

'''
TimeSyncNode error occurs while using Node.create_timer(). Use Timer() instead.
'''
class TimeSyncNode(NodeAdaptor):
    def __init__(self, nodeName : str, timeSyncServiceName : str, timeSyncPeriod_ms : float, timeSyncAccuracy_ms : float, timeSyncWaitService : bool):
        NodeAdaptor.__init__(self, nodeName)
        self.__nodeEnableF = False
        self.__enableFuncF = False
        if (timeSyncServiceName == ''):
            self.get_logger().warning('[TimeSyncNode] Ignored.')
            return

        self.__clientNode = Node(nodeName + '_timesync_client')
        self.__client = self.__clientNode.create_client(TimeSync, timeSyncServiceName)

        self.__isSyncF = False
        self.__correctDuration = Duration(nanoseconds=0)
        self.__timeStampType = Header.STAMPTYPE_NO_SYNC
        self.__timeSyncPeriod_ms = timeSyncPeriod_ms
        self.__timeSyncAccuracy_ms = timeSyncAccuracy_ms

        self.__waitServiceF = timeSyncWaitService
        self.__retryDur_ms = self.__timeSyncPeriod_ms * 0.1 if (self.__timeSyncPeriod_ms * 0.1 < 10000.0) else 10000.0

        self.__nodeEnableF = True

        if (self.__waitServiceF):
            self.__waitService()
        else:
            self.__waitTh = threading.Thread(target=self.__waitService)
        
        self.get_logger().info('[TimeSyncNode] Constructed.')
    
    def __del__(self):
        self.__enableFuncF = False
        if (self.__waitServiceF):
            self.__waitTh.join()
        
    def __waitService(self):
        try:
            ConnToService(self.__client, 5, -1)
            self.__enableFuncF = True

            while (not self.syncTime()):
                time.sleep(1)
            
            if (self.__isSyncF):
                self.get_logger().warning("[TimeSyncNode::__timeSyncTimer_callback] Time synchronized.")
            else:
                self.get_logger().warning("[TimeSyncNode::__timeSyncTimer_callback] Time sync failed.")
            
            if (self.__timeSyncPeriod_ms > 0):
                # self.__timeSyncTimer = Timer(self.__timeSyncPeriod_ms, self.__timeSyncTimer_callback)
                # self.__timeSyncTimer.start()
                self.__timeSyncTimer = self.create_timer(self.__timeSyncPeriod_ms / 1000.0, self.__timeSyncTimer_callback)# TODO: freezed while wait_set.is_ready Error
        except Exception as e:
            self.get_logger().error('[TimeSyncNode::__waitService] Caught unexpected errors.')

    def __timeSyncTimer_callback(self):
        st = self.get_clock().now()
        try:
            while (not self.syncTime() and (self.get_clock().now() - st).nanoseconds < self.__retryDur_ms * 1000000.0):
                time.sleep(0.5)
            if (self.__isSyncF):
                self.get_logger().warning("[TimeSyncNode::__timeSyncTimer_callback] Time synchronized.")
            else:
                self.get_logger().warning("[TimeSyncNode::__timeSyncTimer_callback] Time sync failed.")
        except Exception as e:
            self.get_logger().error("[TimeSyncNode::__timeSyncTimer_callback] Caught unexpected errors.")

    def syncTime(self):
        if (not self.__nodeEnableF or not self.__enableFuncF):
            return False
        try:
            self.__isSyncF = False
            request = TimeSync.Request()
            request.request_code = self.__timeStampType
            request.request_time = self.get_clock().now().to_msg()
            future = self.__client.call_async(request)
            rclpy.spin_until_future_complete(self.__clientNode, future, timeout_sec=0.01)
            if (not future.done()):
                # self.get_logger().error('[TimeSyncNode::syncTime] Failed to call service.')
                return False

            nowTime = self.get_clock().now()
            response = future.result()
            sendTime = Time.from_msg(response.request_time)
            if ((nowTime - sendTime).nanoseconds > self.__timeSyncAccuracy_ms * 1000000.0):# If travel time > accuracy, re-sync
                return False

            refTime = Time.from_msg(response.response_time) - Duration(nanoseconds=(nowTime - sendTime).nanoseconds * 0.5)
            self.__timeStampType = response.response_code
            if (self.__timeStampType == Header.STAMPTYPE_NO_SYNC):
                raise Header.STAMPTYPE_NO_SYNC

            self.__correctDuration = refTime - sendTime

            # self.get_logger().info('Response: %d' %self.__timeStampType)
            # self.get_logger().info('Local time: %f s' %(sendTime.nanoseconds / 1000000000.0))
            # self.get_logger().info('Reference time: %f s' %(refTime.nanoseconds / 1000000000.0))
            # self.get_logger().info('Transport time: %f ms' %((nowTime - sendTime).nanoseconds / 1000000.0))
            # self.get_logger().info('Correct duration: %f us' %(self.__correctDuration.nanoseconds / 1000.0))
            self.__isSyncF = True
            return True

        except Exception as e:
            self.get_logger().error("[TimeSyncNode::syncTime] Unexpected Error.")
            self.__isSyncF = False
            return False
    
    def getTimestamp(self):
        if (not self.__nodeEnableF or not self.__enableFuncF):
            return self.get_clock().now()
        return self.get_clock().now() + self.__correctDuration
    
    def getCorrectDuration(self):
        if (not self.__nodeEnableF or not self.__enableFuncF):
            return Duration(nanoseconds=0)
        return self.__correctDuration
    
    def getTimestampType(self):
        if (not self.__nodeEnableF or not self.__enableFuncF):
            return Header.STAMPTYPE_NO_SYNC
        return self.__timeStampType
