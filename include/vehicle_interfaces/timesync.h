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
    rclcpp::Time initTime_;
    rclcpp::Time refTime_;
    rclcpp::Duration* correctDuration_;
    std::atomic<uint8_t> timeStampType_;
    
    std::shared_ptr<rclcpp::Node> clientNode_;
    rclcpp::Client<vehicle_interfaces::srv::TimeSync>::SharedPtr client_;

    Timer* timeSyncTimer_;
    std::chrono::duration<double, std::milli> timeSyncTimerInterval_;
    std::chrono::duration<double, std::nano> timeSyncAccuracy_;

    std::atomic<bool> isSyncF_;
    std::mutex correctDurationLock_;

    std::chrono::duration<double, std::milli> retryDur_;

    // Node enable
    std::atomic<bool> nodeEnableF_;

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

    void _timerCallback()
    {
        auto st = std::chrono::high_resolution_clock::now();
        try
        {
            while (!this->syncTime() && (std::chrono::high_resolution_clock::now() - st < this->retryDur_))
                std::this_thread::sleep_for(500ms);
            if (!this->isSyncF_)
                RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::_timerCallback] Time sync failed.");
            else
                RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::_timerCallback] Time synced.");
            RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::_timerCallback] Correct duration: %f us.", 
                        this->_safeCall(this->correctDuration_, this->correctDurationLock_).nanoseconds() / 1000.0);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::_timerCallback] Unexpected Error: %s", e.what());
        }
    }

public:
    TimeSyncNode(const std::string& nodeName, 
                    const std::string& timeServiceName, 
                    double syncInterval_ms, 
                    double syncAccuracy_ms) : 
        rclcpp::Node(nodeName), 
        nodeEnableF_(false)
    {
        if (timeServiceName == "")
        {
            RCLCPP_WARN(this->get_logger(), "[TimeSyncNode] Ignored.");
            return;
        }

        this->clientNode_ = rclcpp::Node::make_shared(nodeName + "_timesync_client");
        this->client_ = this->clientNode_->create_client<vehicle_interfaces::srv::TimeSync>(timeServiceName);
        this->isSyncF_ = false;
        this->initTime_ = rclcpp::Time();
        this->refTime_ = rclcpp::Time();
        this->correctDuration_ = new rclcpp::Duration(0, 0);
        this->timeStampType_ = vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        this->timeSyncTimerInterval_ = std::chrono::duration<double, std::milli>(syncInterval_ms);
        this->timeSyncAccuracy_ = std::chrono::duration<double, std::nano>(syncAccuracy_ms * 1000000.0);

        this->retryDur_ = std::chrono::duration<double, std::milli>(5000.0);// First wait 5sec at most
        this->nodeEnableF_ = true;

        this->_timerCallback();
        this->retryDur_ = this->timeSyncTimerInterval_ * 0.1 > std::chrono::seconds(10) ? std::chrono::seconds(10) : this->timeSyncTimerInterval_ * 0.1;
        if (syncInterval_ms > 0)
        {
            this->timeSyncTimer_ = new Timer(syncInterval_ms, std::bind(&TimeSyncNode::_timerCallback, this));
            this->timeSyncTimer_->start();
        }
        RCLCPP_INFO(this->get_logger(), "[TimeSyncNode] Constructed.");
    }

    bool syncTime()
    {
        if (!this->nodeEnableF_)
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
                this->initTime_ = response->request_time;
                if ((nowTime - this->initTime_).nanoseconds() >  this->timeSyncAccuracy_.count())// If travel time > accuracy, re-sync
                    return false;
                
                this->refTime_ = (rclcpp::Time)response->response_time - (nowTime - this->initTime_) * 0.5;
                this->timeStampType_ = response->response_code;
                if (this->timeStampType_ == vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC)
                    throw vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
                
                this->_safeSave(this->correctDuration_, this->refTime_ - this->initTime_, this->correctDurationLock_);
                
                // RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::syncTime] Response: %d.", this->timeStampType_.load());
                // RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::syncTime] Local time: %f s.", this->initTime_.seconds());
                // RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::syncTime] Reference time: %f s.", this->refTime_.seconds());
                // RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::syncTime] Transport time: %f ms.", (nowTime - this->initTime_).nanoseconds() / 1000000.0);
                // RCLCPP_INFO(this->get_logger(), "[TimeSyncNode::syncTime] Correct duration: %f us.", this->correctDuration_->nanoseconds() / 1000.0);
                this->isSyncF_ = true;
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::syncTime] Failed to call service.");
                return false;
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[TimeSyncNode::syncTime] Unexpected Error: %s", e.what());
            this->isSyncF_ = false;
            throw e;
        }
    }

    rclcpp::Time getTimestamp()
    {
        if (!this->nodeEnableF_)
            return this->get_clock()->now();
        return this->get_clock()->now() + this->_safeCall(this->correctDuration_, this->correctDurationLock_);
    }

    rclcpp::Duration getCorrectDuration()
    {
        if (!this->nodeEnableF_)
            return rclcpp::Duration(0, 0);
        return this->_safeCall(this->correctDuration_, this->correctDurationLock_);
    }

    inline uint8_t getTimestampType() const
    {
        if (!this->nodeEnableF_)
            return vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        return this->timeStampType_;
    }
};



class PseudoTimeSyncNode : virtual public rclcpp::Node
{
private:
    rclcpp::Duration* correctDuration_;
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

public:
    PseudoTimeSyncNode(const std::string& nodeName) : 
        rclcpp::Node(nodeName), 
        timeStampType_(vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC)
    {
        this->correctDuration_ = new rclcpp::Duration(0, 0);
    }

    bool syncTime(rclcpp::Duration offset, uint8_t type)
    {
        std::lock_guard<std::mutex> locker(this->correctDurationLock_);
        *this->correctDuration_ = offset;
        this->timeStampType_ = type;
    }

    rclcpp::Time getTimestamp()
    {
        return this->get_clock()->now() + this->_safeCall(this->correctDuration_, this->correctDurationLock_);
    }

    rclcpp::Duration getCorrectDuration()
    {
        return this->_safeCall(this->correctDuration_, this->correctDurationLock_);
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

private:
    void _serviceCallback(const std::shared_ptr<vehicle_interfaces::srv::TimeSync::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::TimeSync::Response> response)
    {
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
};

}// namespace vehicle_interfaces
