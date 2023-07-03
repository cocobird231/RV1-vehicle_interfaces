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
    def __init__(self, nodeName : str, timeServiceName : str, syncInterval_ms : float, syncAccuracy_ms : float):
        NodeAdaptor.__init__(self, nodeName)
        self.__nodeEnableF = False
        if (timeServiceName == ''):
            return
        self.__clientNode = Node(nodeName + '_timesync_client')
        self.__client = self.__clientNode.create_client(TimeSync, timeServiceName)

        self.__isSyncF = False
        self.__initTime = Time()
        self.__refTime = Time()
        self.__correctDuration = Duration(nanoseconds=0)
        self.__timeStampType = Header.STAMPTYPE_NO_SYNC
        self.__timeSyncIntervals_ms = syncInterval_ms
        self.__timeSyncAccuracy_ms = syncAccuracy_ms
        # self.__connToService()

        self.__nodeEnableF = True

        self.__timeSyncTimer_callback()
        if (syncInterval_ms > 0):
            # self.__timeSyncTimer = Timer(self.__timeSyncIntervals_ms, self.__timeSyncTimer_callback)
            # self.__timeSyncTimer.start()
            self.__timeSyncTimer = self.create_timer(syncInterval_ms / 1000.0, self.__timeSyncTimer_callback)# TODO: freezed while wait_set.is_ready Error
        
    
    def __timeSyncTimer_callback(self):
        st = self.get_clock().now()
        try:
            retryDur = 5000.0 if self.__timeSyncIntervals_ms * 0.5 > 5000.0 else self.__timeSyncIntervals_ms * 0.5
            while ((not self.syncTime()) and ((self.get_clock().now() - st).nanoseconds / 1000000.0 < retryDur)):
                time.sleep(0.5)
            if (not self.__isSyncF):
                print("[TimeSyncNode::__timeSyncTimer_callback] Time sync failed.")
            else:
                print("[TimeSyncNode::__timeSyncTimer_callback] Time synced.")
            print("[TimeSyncNode::__timeSyncTimer_callback] Correct duration: %f us" %(self.__correctDuration.nanoseconds / 1000.0))
        except Exception as e:
            print("[TimeSyncNode::__timeSyncTimer_callback] Unexpected Error", e)

    def __connToService(self, client):
        errCnt = 5
        while (not client.wait_for_service(timeout_sec=0.5) and errCnt > 0):
            errCnt -= 1
            self.get_logger().info('[TimeSyncNode.__connToService] service not available, waiting again...')
        if (errCnt <= 0):
            self.get_logger().info('[TimeSyncNode.__connToService] Connect to service failed.')
        else:
            self.get_logger().info('[TimeSyncNode.__connToService] Service connected.')

    def syncTime(self):
        if (not self.__nodeEnableF):
            return False
        try:
            self.__isSyncF = False
            request = TimeSync.Request()
            request.request_code = self.__timeStampType
            request.request_time = self.get_clock().now().to_msg()
            future = self.__client.call_async(request)
            rclpy.spin_until_future_complete(self.__clientNode, future, timeout_sec=0.01)
            if (not future.done()):
                self.__clientNode.get_logger().info('Failed to call service')
                return False

            nowTime = self.get_clock().now()
            response = future.result()
            self.__initTime = Time.from_msg(response.request_time)
            if ((nowTime - self.__initTime).nanoseconds > self.__timeSyncAccuracy_ms * 1000000.0):# If travel time > accuracy, re-sync
                return False

            self.__refTime = Time.from_msg(response.response_time) - Duration(nanoseconds=(nowTime - self.__initTime).nanoseconds * 0.5)
            self.__timeStampType = response.response_code
            if (self.__timeStampType == Header.STAMPTYPE_NO_SYNC):
                raise Header.STAMPTYPE_NO_SYNC
            
            self.__correctDuration = self.__refTime - self.__initTime

            self.__clientNode.get_logger().info('Response: %d' %self.__timeStampType)
            self.__clientNode.get_logger().info('Local time: %f s' %(self.__initTime.nanoseconds / 1000000000.0))
            self.__clientNode.get_logger().info('Reference time: %f s' %(self.__refTime.nanoseconds / 1000000000.0))
            self.__clientNode.get_logger().info('Transport time: %f ms' %((nowTime - self.__initTime).nanoseconds / 1000000.0))
            self.__clientNode.get_logger().info('Correct duration: %f us' %(self.__correctDuration.nanoseconds / 1000.0))
            self.__isSyncF = True
            return True

        except Exception as e:
            print("[TimeSyncNode::syncTime] Unexpected Error")
            self.__isSyncF = False
            raise e
    
    def getTimestamp(self):
        if (not self.__nodeEnableF):
            return self.get_clock().now()
        return self.get_clock().now() + self.__correctDuration
    
    def getCorrectDuration(self):
        if (not self.__nodeEnableF):
            return Duration(nanoseconds=0)
        return self.__correctDuration
    
    def getTimestampType(self):
        if (not self.__nodeEnableF):
            return Header.STAMPTYPE_NO_SYNC
        return self.__timeStampType
