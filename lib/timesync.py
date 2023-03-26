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

class Timer():
    def __init__(self, interval_ms : float, cb) -> None:
        self.interval_ms_ = interval_ms
        self.func_ = cb
        self.timerTH_ = threading.Thread(target=self._startTimerTH)

        self.activateF_ = False
        self.exitF_ = False

        self.funcCallableF_ = True
        self.funcTH_ = threading.Thread(target=self.func_)

        self.timerTH_.start()
    
    def __del__(self):
        self.destroy()
    
    def _callback(self):
        self.funcCallableF_ = False
        self.func_()
        self.funcCallableF_ = True
    
    def _timer_tick(self):
        if (not self.exitF_ and self.activateF_ and self.funcCallableF_):
            if (self.funcTH_.is_alive()):
                self.funcTH_.join()
            self.funcTH_ = threading.Thread(target=self._callback)
            self.funcTH_.start()

    def _startTimerTH(self):
        while (not self.exitF_):
            if (self.activateF_):
                timer_ = threading.Timer(self.interval_ms_ / 1000.0, self._timer_tick)
                timer_.start()
                timer_.join()

    def start(self) -> None:
        self.activateF_ = True
    
    def stop(self) -> None:
        self.activateF_ = False
    
    def destroy(self):
        self.activateF_ = False
        self.exitF_ = True
        self.timerTH_.join()

class TimeSyncNode(Node):
    def __init__(self, nodeName : str, timeServiceName : str, syncInterval_ms : int = 10000):
        super().__init__(nodeName)
        self.clientNode_ = Node(nodeName + '_timesync_client')
        self.client_ = self.clientNode_.create_client(TimeSync, timeServiceName)

        self.isSyncF_ = False
        self.initTime_ = Time()
        self.refTime_ = Time()
        self.correctDuration_ = Duration(nanoseconds=0)
        self.timeStampType_ = Header.STAMPTYPE_NO_SYNC

        self.correctDurationLock_ = threading.Lock()

        print("[TimeSyncNode] Sync time from %s..." %timeServiceName)
        self.connToService()
        time.sleep(0.2)
        try:
            while (not self.syncTime()):
                time.sleep(0.5)
        except Exception as e:
            print("[TimeSyncNode] Unexpected Error", e)
        print("[TimeSyncNode] Time synced: %d\n" %self.isSyncF_)

        if (syncInterval_ms > 0):
            self.timeSyncIntervals_ms = syncInterval_ms
            self.timeSyncTimer_ = Timer(self.timeSyncIntervals_ms, self.timeSyncTimer_callback)
            self.timeSyncTimer_.start()
    
    def timeSyncTimer_callback(self):
        st = self.get_clock().now()
        try:
            while ((not self.syncTime()) and ((self.get_clock().now() - st).nanoseconds / 1000000.0 < self.timeSyncIntervals_ms)):
                time.sleep(0.5)
            if (not self.isSyncF_):
                print("[TimeSyncNode::timeSyncTimer_callback] Time sync failed.")
            else:
                print("[TimeSyncNode::timeSyncTimer_callback] Time synced.")
            print("[TimeSyncNode::timeSyncTimer_callback] Correct duration: %f us" %(self.correctDuration_.nanoseconds / 1000.0))
        except Exception as e:
            print("[TimeSyncNode::timeSyncTimer_callback] Unexpected Error", e)
    
    def connToService(self):
        while (not self.client_.wait_for_service(timeout_sec=0.5)):
            self.clientNode_.get_logger().info('service not available, waiting again...')
        self.clientNode_.get_logger().info('Time service connected.')
    
    def syncTime(self):
        try:
            self.isSyncF_ = False
            request = TimeSync.Request()
            request.request_code = self.timeStampType_
            request.request_time = self.get_clock().now().to_msg()
            future = self.client_.call_async(request)
            rclpy.spin_until_future_complete(self.clientNode_, future, timeout_sec=0.01)
            if (not future.done()):
                self.clientNode_.get_logger().info('Failed to call service')
                return False

            nowTime = self.get_clock().now()
            response = future.result()
            self.initTime_ = Time.from_msg(response.request_time)
            if ((nowTime - self.initTime_).nanoseconds > 10000000.0):# If request and response time > 10ms, re-sync
                return False

            self.refTime_ = Time.from_msg(response.response_time) - Duration(nanoseconds=(nowTime - self.initTime_).nanoseconds * 0.5)
            self.timeStampType_ = response.response_code
            if (self.timeStampType_ == Header.STAMPTYPE_NO_SYNC):
                raise Header.STAMPTYPE_NO_SYNC
            
            self.correctDuration_ = self.refTime_ - self.initTime_

            self.clientNode_.get_logger().info('Response: %d' %self.timeStampType_)
            self.clientNode_.get_logger().info('Local time: %f s' %(self.initTime_.nanoseconds / 1000000000.0))
            self.clientNode_.get_logger().info('Reference time: %f s' %(self.refTime_.nanoseconds / 1000000000.0))
            self.clientNode_.get_logger().info('Transport time: %f ms' %((nowTime - self.initTime_).nanoseconds / 1000000.0))
            self.clientNode_.get_logger().info('Correct duration: %f us' %(self.correctDuration_.nanoseconds / 1000.0))
            self.isSyncF_ = True
            return True

        except Exception as e:
            print("[TimeSyncNode::syncTime] Unexpected Error")
            self.isSyncF_ = False
            raise e
    
    def getTimestamp(self):
        return self.get_clock().now() + self.correctDuration_
    