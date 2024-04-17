#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/header.hpp"
#include "vehicle_interfaces/srv/time_sync.hpp"

#ifndef ROS_DISTRO// 0: eloquent, 1: foxy, 2: humble
#define ROS_DISTRO 1
#endif

using namespace std::chrono_literals;

namespace vehicle_interfaces
{

/**
 * The TimeSyncNode class implements the time sync mechanisms, which calling time sync server in a given fixed rate, and update the offset value 
 * between local timestamp and time sync server.
 * The nodes can easily inherit the TimeSyncNode to get time sync supported. The getTimestamp() function returns the corrected timestamp, the 
 * getCorrectDuration() function returns the correction offset, and calling getTimestampType() function can get the timestamp type defined by 
 * Header under vehicle_interfaces. Manually process time sync is possible by calling syncTime().
 */
class TimeSyncNode : virtual public rclcpp::Node
{
private:
    // Time sync value
    std::shared_ptr<rclcpp::Duration> correctDuration_;
    std::atomic<uint8_t> timeStampType_;
    
    // Node and service
    std::shared_ptr<rclcpp::Node> clientNode_;
    rclcpp::Client<vehicle_interfaces::srv::TimeSync>::SharedPtr client_;

    // Time sync parameters
    std::shared_ptr<Timer> timeSyncTimer_;
    std::chrono::duration<double, std::milli> timeSyncPeriod_;
    std::chrono::duration<double, std::nano> timeSyncAccuracy_;
    std::chrono::duration<double, std::milli> retryDur_;

    std::atomic<bool> isSyncF_;
    std::mutex correctDurationLock_;

    // Callback
    std::function<void()> cbFunc_;
    std::atomic<bool> isCbFuncF_;

    // Node enable
    std::atomic<bool> nodeEnableF_;
    vehicle_interfaces::unique_thread waitTh_;
    bool enableFuncF_;
    bool exitF_;

private:
    template <typename T>
    void _safeSave(T* ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr = value;
    }

    template <typename T>
    T _safeCall(const T* ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr;
    }

    template <typename T>
    void _safeSharedPtrSave(std::shared_ptr<T> ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr.get() = value;
    }

    template <typename T>
    T _safeSharedPtrCall(const std::shared_ptr<T> ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr.get();
    }

    /**
     * This function will be called only once and run under sub-thread while construction time.
    */
    void _waitService()
    {
        try
        {
            if (!vehicle_interfaces::ConnToService(this->client_, this->exitF_, std::chrono::milliseconds(5000), -1))
            {
                RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::_waitService] Failed to connect to service.");
                return;
            }
            this->enableFuncF_ = true;

            while (!this->syncTime())
                std::this_thread::sleep_for(1000ms);

            if (this->isSyncF_)
                RCLCPP_WARN(this->get_logger(), "[TimeSyncNode::_waitService] Time synchronized.");
            else
                RCLCPP_WARN(this->get_logger(), "[TimeSyncNode::_waitService] Time sync failed.");

            if (this->timeSyncPeriod_ > 0s)
            {
                this->timeSyncTimer_ = std::make_shared<Timer>(this->timeSyncPeriod_.count(), std::bind(&TimeSyncNode::_timerCallback, this));
                this->timeSyncTimer_->start();
            }
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::_waitService] Caught unexpected errors.");
        }
    }

    void _timerCallback()
    {
        auto st = std::chrono::high_resolution_clock::now();
        try
        {
            while (!this->syncTime() && (std::chrono::high_resolution_clock::now() - st < this->retryDur_))
                std::this_thread::sleep_for(500ms);
            if (this->isSyncF_)
            {
                RCLCPP_WARN(this->get_logger(), "[TimeSyncNode::_timerCallback] Time synchronized.");
                if (this->isCbFuncF_)
                    this->cbFunc_();
            }
            else
                RCLCPP_WARN(this->get_logger(), "[TimeSyncNode::_timerCallback] Time sync failed.");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::_timerCallback] Exception: %s.", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::_timerCallback] Caught unexpected errors.");
        }
    }

public:
    TimeSyncNode(const std::string& nodeName, 
                    const std::string& timeSyncServiceName, 
                    double timeSyncPeriod_ms, 
                    double timeSyncAccuracy_ms, 
                    bool timeSyncWaitService) : 
        rclcpp::Node(nodeName), 
        nodeEnableF_(false), 
        isSyncF_(false), 
        isCbFuncF_(false), 
        enableFuncF_(false), 
        exitF_(false)
    {
        if (timeSyncServiceName == "")
        {
            RCLCPP_WARN(this->get_logger(), "[TimeSyncNode] Ignored.");
            return;
        }

        this->clientNode_ = rclcpp::Node::make_shared(nodeName + "_timesync_client");
        this->client_ = this->clientNode_->create_client<vehicle_interfaces::srv::TimeSync>(timeSyncServiceName);

        this->correctDuration_ = std::make_shared<rclcpp::Duration>(0, 0);
        this->timeStampType_ = vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        this->timeSyncPeriod_ = std::chrono::duration<double, std::milli>(timeSyncPeriod_ms);
        this->timeSyncAccuracy_ = std::chrono::duration<double, std::nano>(timeSyncAccuracy_ms * 1000000.0);

        this->retryDur_ = this->timeSyncPeriod_ * 0.1 > std::chrono::seconds(10) ? std::chrono::seconds(10) : this->timeSyncPeriod_ * 0.1;
        this->nodeEnableF_ = true;

        if (timeSyncWaitService)// Wait until first timesync done.
        {
            this->_waitService();
        }
        else// Wait and sync service at background
        {
            this->waitTh_ = vehicle_interfaces::make_unique_thread(&TimeSyncNode::_waitService, this);
        }
        RCLCPP_INFO(this->get_logger(), "[TimeSyncNode] Constructed.");
    }

    ~TimeSyncNode()
    {
        if (this->exitF_)
            return;
        this->exitF_ = true;

        this->enableFuncF_ = false;
    }

    void addTimeSyncCallbackFunc(const std::function<void()>& func)
    {
        if (this->isCbFuncF_ || !this->nodeEnableF_)
            return;
        this->cbFunc_ = func;
        this->isCbFuncF_ = true;
    }

    bool syncTime()
    {
        if (!this->nodeEnableF_ || !this->enableFuncF_)
            return false;
        try
        {
            this->isSyncF_ = false;
            auto request = std::make_shared<vehicle_interfaces::srv::TimeSync::Request>();
            request->request_code = this->timeStampType_;
            request->request_time = this->get_clock()->now();
            auto result = this->client_->async_send_request(request);
            // Wait for the result.

#if ROS_DISTRO == 0
            if (rclcpp::spin_until_future_complete(this->clientNode_, result, 10ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
            if (rclcpp::spin_until_future_complete(this->clientNode_, result, 10ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
            {
                rclcpp::Time nowTime = this->get_clock()->now();
                auto response = result.get();
                rclcpp::Time sendTime = response->request_time;
                if ((nowTime - sendTime).nanoseconds() >  this->timeSyncAccuracy_.count())// If travel time > accuracy, re-sync
                    return false;

                rclcpp::Time refTime = (rclcpp::Time)response->response_time - (nowTime - sendTime) * 0.5;
                this->timeStampType_ = response->response_code;
                if (this->timeStampType_ == vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC)
                    throw vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;

                this->_safeSharedPtrSave(this->correctDuration_, refTime - sendTime, this->correctDurationLock_);
                this->isSyncF_ = true;
                // RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::syncTime] Time sync succeed.");
                return true;
            }
            // RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::syncTime] Failed to call service.");
            return false;
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::syncTime] Unexpected Error.");
            this->isSyncF_ = false;
            return false;
        }
    }

    rclcpp::Time getTimestamp()
    {
        if (!this->nodeEnableF_ || !this->enableFuncF_)
            return this->get_clock()->now();
        return this->get_clock()->now() + this->_safeSharedPtrCall(this->correctDuration_, this->correctDurationLock_);
    }

    rclcpp::Duration getCorrectDuration()
    {
        if (!this->nodeEnableF_ || !this->enableFuncF_)
            return rclcpp::Duration(0, 0);
        return this->_safeSharedPtrCall(this->correctDuration_, this->correctDurationLock_);
    }

    inline uint8_t getTimestampType() const
    {
        if (!this->nodeEnableF_ || !this->enableFuncF_)
            return vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        return this->timeStampType_;
    }
};



class PseudoTimeSyncNode : virtual public rclcpp::Node
{
private:
    std::shared_ptr<rclcpp::Duration> correctDuration_;
    std::atomic<uint8_t> timeStampType_;

    std::mutex correctDurationLock_;

private:
    template <typename T>
    void _safeSave(T* ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr = value;
    }

    template <typename T>
    T _safeCall(const T* ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr;
    }

    template <typename T>
    void _safeSharedPtrSave(std::shared_ptr<T> ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr.get() = value;
    }

    template <typename T>
    T _safeSharedPtrCall(const std::shared_ptr<T> ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr.get();
    }

public:
    PseudoTimeSyncNode(const std::string& nodeName) : 
        rclcpp::Node(nodeName), 
        timeStampType_(vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC)
    {
        this->correctDuration_ = std::make_shared<rclcpp::Duration>(0, 0);
        RCLCPP_INFO(this->get_logger(), "[PseudoTimeSyncNode] Constructed.");
    }

    bool syncTime(rclcpp::Duration offset, uint8_t type)
    {
        std::lock_guard<std::mutex> locker(this->correctDurationLock_);
        try
        {
            *this->correctDuration_ = offset;
            this->timeStampType_ = type;
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    rclcpp::Time getTimestamp()
    {
        return this->get_clock()->now() + this->_safeSharedPtrCall(this->correctDuration_, this->correctDurationLock_);
    }

    rclcpp::Duration getCorrectDuration()
    {
        return this->_safeSharedPtrCall(this->correctDuration_, this->correctDurationLock_);
    }

    inline uint8_t getTimestampType() const { return this->timeStampType_; }
};



/* TimeSyncServer
 */
class TimeSyncServer : public rclcpp::Node
{
private:
    rclcpp::Service<vehicle_interfaces::srv::TimeSync>::SharedPtr server_;
    bool isUTCSync;
    std::mutex cbLock_;

private:
    void _serviceCallback(const std::shared_ptr<vehicle_interfaces::srv::TimeSync::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::TimeSync::Response> response)
    {
        std::lock_guard<std::mutex> locker(this->cbLock_);// Test lock to prevent unexpected crashing.
        RCLCPP_INFO(this->get_logger(), "[TimeSyncServer::_serviceCallback] Request: %d", request->request_code);
        response->request_time = request->request_time;
        response->response_time = this->get_clock()->now();
        //printf("req time: %f\n", response->request_time.stamp.sec + response->request_time.stamp.nanosec / 1000000000.0);
        //printf("res time: %f\n", response->response_time.stamp.sec + response->response_time.stamp.nanosec / 1000000000.0);
        
        if (!this->isUTCSync)
            response->response_code = vehicle_interfaces::msg::Header::STAMPTYPE_NONE_UTC_SYNC;
        else
            response->response_code = vehicle_interfaces::msg::Header::STAMPTYPE_UTC_SYNC;
        
    }

public:
    TimeSyncServer(const std::string& nodeName, const std::string& serviceName) : rclcpp::Node(nodeName)
    {
        this->server_ = this->create_service<vehicle_interfaces::srv::TimeSync>(serviceName, 
            std::bind(&TimeSyncServer::_serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
        this->isUTCSync = false;
        RCLCPP_INFO(this->get_logger(), "[TimeSyncServer] Constructed.");
    }

    void setUTCSyncStatus(bool flag) { this->isUTCSync = flag; }
};

}// namespace vehicle_interfaces
