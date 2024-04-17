#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <regex>

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg_json.h"

#include "vehicle_interfaces/msg/chassis.hpp"
#include "vehicle_interfaces/msg/steering_wheel.hpp"
#include "vehicle_interfaces/msg/chassis_info.hpp"
#include "vehicle_interfaces/msg/control_chassis.hpp"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/msg/control_steering_wheel.hpp"

#include "vehicle_interfaces/srv/control_chassis_reg.hpp"
#include "vehicle_interfaces/srv/control_chassis_req.hpp"
#include "vehicle_interfaces/srv/controller_info_reg.hpp"
#include "vehicle_interfaces/srv/controller_info_req.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_reg.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_req.hpp"

using namespace std::chrono_literals;


namespace vehicle_interfaces
{

class BaseControllerServer : public rclcpp::Node
{
protected:
    const vehicle_interfaces::msg::ControllerInfo cInfo_;// ControllerServer information.

protected:
    BaseControllerServer(const vehicle_interfaces::msg::ControllerInfo& cInfo) : 
        rclcpp::Node(cInfo.node_name), 
        cInfo_(cInfo) {}

public:
    vehicle_interfaces::msg::ControllerInfo getInfo() const { return this->cInfo_; }

    virtual void close() = 0;
};

template <typename serviceT, typename topicT, typename controlT>
class ControllerServer : public BaseControllerServer
{
private:
    std::shared_ptr<rclcpp::Service<serviceT> > reqServer_;// Server to response control signal.
    std::mutex reqServerLock_;// Lock reqServer_.
    std::atomic<uint64_t> reqServerFrameId_;// The counter of service response.

    std::shared_ptr<rclcpp::Publisher<topicT> > pub_;// Publisher to publish control signal.
    std::shared_ptr<vehicle_interfaces::Timer> pubTm_;// Timer to publish control signal.
    std::atomic<uint64_t> pubFrameId_;// The counter of control signal published.

    controlT msg_;// Control signal.
    std::mutex msgLock_;// Lock msg_.

    std::atomic<bool> exitF_;// Whether the controller is closed.
    
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

    /**
     * (Sub-thread) Service callback function to response control signal.
     */
    void _reqServerCbFunc(const std::shared_ptr<typename serviceT::Request> request, std::shared_ptr<typename serviceT::Response> response)
    {
        RCLCPP_ERROR(this->get_logger(), "[ControllerServer::_reqServerCbFunc] Function template not specialized.");
    }

    /**
     * Set header for published message.
     * @param[out] msg Header message.
     */
    void _getHeader(vehicle_interfaces::msg::Header& msg)
    {
        msg.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        msg.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
        msg.device_id = this->cInfo_.node_name;
        msg.frame_id = this->pubFrameId_++;
        msg.stamp_type = vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        msg.stamp = this->get_clock()->now();
        msg.stamp_offset = 0;
        msg.ref_publish_time_ms = this->cInfo_.period_ms;
    }

    /**
     * (Sub-thread) Timer callback function to publish control signal.
     * @note This function should be specialized.
     */
    void _publish()
    {
        RCLCPP_ERROR(this->get_logger(), "[ControllerServer::_publish] Function template not specialized.");
    }

public:
    /**
     * ControllerServer constructor.
     * @param[in] cInfo ControllerServer information.
     * @note The ControllerServer will be created according to the information in cInfo.
     * @note - The ControllerServer node name will be cInfo.node_name.
     * @note - The ControllerServer service name or topic name will be cInfo.service_name.
     * @note - The ControllerServer will response control signal through service or topic according to cInfo.controller_mode.
     * @note - The ControllerServer will publish control signal through topic according to cInfo.pub_type.
     * @note - The ControllerServer will publish control signal with period cInfo.period_ms.
     * @note - The control signal type will be ControlChassis or ControlSteeringWheel according to cInfo.msg_type.
     */
    ControllerServer(const vehicle_interfaces::msg::ControllerInfo& cInfo) : 
        BaseControllerServer(cInfo), 
        reqServerFrameId_(0), 
        pubFrameId_(0), 
        exitF_(false)
    {
        if (cInfo.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_SERVICE)
        {
            this->reqServer_ = this->create_service<serviceT>(cInfo.service_name, 
                                std::bind(&ControllerServer::_reqServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
        }

        if (cInfo.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_TOPIC || 
            cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER_SERVER || 
            cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH)
        {
            this->pub_ = this->create_publisher<topicT>(cInfo.service_name, 10);
            this->pubTm_ = std::make_shared<vehicle_interfaces::Timer>(cInfo.period_ms, std::bind(&ControllerServer::_publish, this));
            this->pubTm_->start();
        }
    }

    ~ControllerServer()
    {
        this->close();
    }

    /**
     * Set control signal.
     * @param[in] msg Control signal supports ControlChassis or ControlSteeringWheel.
     */
    void setControlSignal(const controlT& msg)
    {
        this->_safeSave(&this->msg_, msg, this->msgLock_);// Save control signal to be published.
    }

    void close()
    {
        if (this->exitF_)// Ignore process if called repeatedly.
            return;
        this->exitF_ = true;

        if (this->pubTm_)
            this->pubTm_->destroy();
    }
};

/**
 * (Sub-thread) Service callback function to response Chassis signal.
 * @note Template specialization for ControlChassisReq, Chassis, ControlChassis.
 */
template<>
void ControllerServer<
    vehicle_interfaces::srv::ControlChassisReq, 
    vehicle_interfaces::msg::Chassis, 
    vehicle_interfaces::msg::ControlChassis
>
::_reqServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlChassisReq::Request> request, 
                    std::shared_ptr<vehicle_interfaces::srv::ControlChassisReq::Response> response)
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerServer::_reqServerCbFunc] Received request.");
    std::lock_guard<std::mutex> reqServerLocker(this->reqServerLock_);// Restrict only one callback at a time.
    response->response = true;
    response->value = this->_safeCall(&this->msg_, this->msgLock_);
    response->frame_id = this->reqServerFrameId_++;
}

/**
 * (Sub-thread) Timer callback function to publish Chassis signal.
 * @note Template specialization for ControlChassisReq, Chassis, ControlChassis.
 */
template<>
void ControllerServer<
    vehicle_interfaces::srv::ControlChassisReq, 
    vehicle_interfaces::msg::Chassis, 
    vehicle_interfaces::msg::ControlChassis
>
::_publish()
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerServer::_publish] Publishing control signal.");
    auto tmp = this->_safeCall(&this->msg_, this->msgLock_);// Get control signal to be published.
    auto cvt = vehicle_interfaces::msg_to_msg::Chassis::convert(tmp);
    cvt.controller_name = this->cInfo_.service_name;
    this->_getHeader(cvt.header);// Set header.
    this->pub_->publish(cvt);
}

/**
 * (Sub-thread) Service callback function to response SteeringWheel signal.
 * @note Template specialization for ControlSteeringWheelReq, SteeringWheel, ControlSteeringWheel.
 */
template<>
void ControllerServer<
    vehicle_interfaces::srv::ControlSteeringWheelReq, 
    vehicle_interfaces::msg::SteeringWheel, 
    vehicle_interfaces::msg::ControlSteeringWheel
>
::_reqServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReq::Request> request, 
                    std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReq::Response> response)
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerServer::_reqServerCbFunc] Received request.");
    std::lock_guard<std::mutex> reqServerLocker(this->reqServerLock_);// Restrict only one callback at a time.
    response->response = true;
    response->value = this->_safeCall(&this->msg_, this->msgLock_);
    response->frame_id = this->reqServerFrameId_++;
}

/**
 * (Sub-thread) Timer callback function to publish SteeringWheel signal.
 * @note Template specialization for ControlSteeringWheelReq, SteeringWheel, ControlSteeringWheel.
 */
template<>
void ControllerServer<
    vehicle_interfaces::srv::ControlSteeringWheelReq, 
    vehicle_interfaces::msg::SteeringWheel, 
    vehicle_interfaces::msg::ControlSteeringWheel
>
::_publish()
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerServer::_publish] Publishing control signal.");
    auto tmp = this->_safeCall(&this->msg_, this->msgLock_);// Get control signal to be published.
    auto cvt = vehicle_interfaces::msg_to_msg::SteeringWheel::convert(tmp);
    cvt.controller_name = this->cInfo_.service_name;
    this->_getHeader(cvt.header);// Set header.
    this->pub_->publish(cvt);
}


class BaseControllerClient : public rclcpp::Node
{
protected:
    const std::string nodeName_;
    const vehicle_interfaces::msg::ControllerInfo cInfo_;// Registered ControllerServer information.

protected:
    BaseControllerClient(const std::string& nodeName, const vehicle_interfaces::msg::ControllerInfo& cInfo) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        cInfo_(cInfo) {}

public:
    vehicle_interfaces::msg::ControllerInfo getInfo() const { return this->cInfo_; }

    virtual bool getSignal(vehicle_interfaces::msg::ControlChassis& outSignal, std::chrono::high_resolution_clock::time_point& outTimestamp) = 0;

    virtual void close() = 0;
};

template <typename serviceT, typename topicT, typename controlT>
class ControllerClient : public BaseControllerClient
{
private:
    std::shared_ptr<rclcpp::Node> clientNode_;// Node for client.
    std::shared_ptr<rclcpp::Client<serviceT> > reqClient_;// Client to request control signal.
    std::shared_ptr<vehicle_interfaces::Timer> reqClientTm_;// Call _reqClientTmCbFunc().
    std::chrono::duration<float, std::milli> serviceTimeout_;// Timeout for service request.

    std::shared_ptr<rclcpp::Subscription<topicT> > sub_;// Subscription to receive control signal.

    std::shared_ptr<rclcpp::Publisher<topicT> > pub_;// Publisher to publish control signal.
    std::shared_ptr<vehicle_interfaces::Timer> pubTm_;// Timer to publish control signal.
    controlT pubMsg_;// Control signal to be published.
    std::atomic<uint64_t> pubFrameId_;// The counter of control signal published.
    std::mutex pubMsgLock_;// Lock pubMsg_.

    // Convert function for ControlSteeringWheel signal to ControlChassis signal.
    std::function<bool(const vehicle_interfaces::msg::ControlSteeringWheel&, vehicle_interfaces::msg::ControlChassis&)> cvtFunc_;
    std::atomic<bool> cvtFuncF_;

    vehicle_interfaces::msg::ControlChassis msg_;// Control signal.
    std::atomic<uint64_t> frameID_;// The counter of service req/res.
    std::chrono::high_resolution_clock::time_point dataTs_;// Timestamp of grabbed control signal.
    std::mutex msgLock_;// Lock msg_.

    std::function<void(const vehicle_interfaces::msg::ControllerInfo&)> interruptFunc_;// Function called when the controller raises interrupt signal.
    std::atomic<bool> interruptFuncF_;// Whether interruptFunc_ is available.

    std::atomic<bool> availableF_;// Whether the controller is available.
    std::atomic<bool> exitF_;

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

    /**
     * (Sub-thread) Timer callback function to request control signal.
     * @note This function should be specialized.
     */
    void _reqClientTmCbFunc()
    {
        RCLCPP_ERROR(this->get_logger(), "[ControllerClient::_reqClientTmCbFunc] Function template not specialized.");
    }

    /**
     * (Sub-thread) Subscription callback function to receive control signal.
     * @note This function should be specialized.
     */
    void _subCbFunc(const std::shared_ptr<topicT> msg)
    {
        RCLCPP_ERROR(this->get_logger(), "[ControllerClient::_subCbFunc] Function template not specialized.");
    }

    /**
     * (Sub-thread) Timer callback function to publish control signal.
     * @note This function should be specialized.
     */
    void _publish()
    {
        RCLCPP_ERROR(this->get_logger(), "[ControllerClient::_publish] Function template not specialized.");
    }

    /**
     * Set header for published message.
     * @param[out] msg Header message.
     */
    void _getHeader(vehicle_interfaces::msg::Header& msg)
    {
        msg.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        msg.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
        msg.device_id = this->nodeName_;
        msg.frame_id = this->pubFrameId_++;
        msg.stamp_type = vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        msg.stamp = this->get_clock()->now();
        msg.stamp_offset = 0;
        msg.ref_publish_time_ms = this->cInfo_.period_ms;
    }

public:
    /**
     * ControllerClient constructor.
     * @param[in] nodeName Node name for ControllerClient.
     * @param[in] cInfo ControllerServer information.
     * @note The ControllerClient will be created according to the information in cInfo.
     * @note - The ControllerClient node name will be nodeName.
     * @note - The ControllerClient will request control signal through service or topic according to cInfo.controller_mode.
     * @note - The service name or topic name will be cInfo.service_name.
     * @note - If cInfo.controller_mode set to CONTROLLER_MODE_SERVICE, the request period should be set in cInfo.period_ms.
     * @note - If cInfo.controller_mode set to CONTROLLER_MODE_SERVICE, create client node with node name `nodeName + "_cli"`.
     * @note - If cInfo.pub_type set to PUB_TYPE_CONTROLLER_CLIENT or PUB_TYPE_BOTH, create publisher with topic name `nodeName`.
     */
    ControllerClient(const std::string& nodeName, const vehicle_interfaces::msg::ControllerInfo& cInfo) : 
        BaseControllerClient(nodeName, cInfo), 
        pubFrameId_(0), 
        cvtFuncF_(false), 
        interruptFuncF_(false), 
        availableF_(false), 
        exitF_(false)
    {
        if (cInfo.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_SERVICE)
        {
            this->serviceTimeout_ = std::chrono::duration<float, std::milli>(cInfo.timeout_ms);
            this->clientNode_ = std::make_shared<rclcpp::Node>(nodeName + "_cli");
            this->reqClient_ = this->clientNode_->create_client<serviceT>(cInfo.service_name);
            // Create timer.
            this->reqClientTm_ = std::make_shared<vehicle_interfaces::Timer>(cInfo.period_ms, std::bind(&ControllerClient::_reqClientTmCbFunc, this));
            this->reqClientTm_->start();
        }
        else if (cInfo.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_TOPIC)
        {
            this->sub_ = this->create_subscription<topicT>(cInfo.service_name, 10, 
                            std::bind(&ControllerClient::_subCbFunc, this, std::placeholders::_1));
        }
        else
        {
            throw "[ControllerClient] Controller mode error.";
            return;
        }

        if (cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER_CLIENT || 
            cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH)
        {
            this->pub_ = this->create_publisher<topicT>(nodeName, 10);
            this->pubTm_ = std::make_shared<vehicle_interfaces::Timer>(cInfo.period_ms, std::bind(&ControllerClient::_publish, this));
            this->pubTm_->start();
        }
        this->availableF_ = true;
    }

    ~ControllerClient()
    {
        this->close();
    }

    /**
     * Close ControllerClient.
     */
    void close()
    {
        if (this->exitF_)// Ignore process if called repeatedly.
            return;
        this->exitF_ = true;// All looping process will be braked if exitF_ set to true.

        if (this->reqClientTm_)
            this->reqClientTm_->destroy();
        if (this->pubTm_)
            this->pubTm_->destroy();
    }

    /**
     * Set convert function for ControlSteeringWheel signal to ControlChassis signal.
     * @param[in] cvtFunc Convert function.
     * @note This function should be called if the message type is ControlSteeringWheel.
     */
    void setCvtFunc(const std::function<bool(const vehicle_interfaces::msg::ControlSteeringWheel&, vehicle_interfaces::msg::ControlChassis&)>& cvtFunc)
    {
        this->cvtFunc_ = cvtFunc;
        this->cvtFuncF_ = true;
    }

    /**
     * Set interrupt function for Controller interrupt signal raised.
     * @param[in] interruptFunc Interrupt function.
     * @note This function can be ignored if the controller does not support interrupt signal.
     */
    void setInterruptFunc(const std::function<void(const vehicle_interfaces::msg::ControllerInfo&)>& interruptFunc)
    {
        this->interruptFunc_ = interruptFunc;
        this->interruptFuncF_ = true;
    }

    /**
     * Get ControlChassis signal and its timestamp.
     * @param[out] outSignal ControlChassis signal.
     * @param[out] outTimestamp Timestamp of the signal.
     * @return Whether the signal is available.
     */
    bool getSignal(vehicle_interfaces::msg::ControlChassis& outSignal, std::chrono::high_resolution_clock::time_point& outTimestamp)
    {
        if (!this->availableF_)
            return false;
        std::lock_guard<std::mutex> locker(this->msgLock_);
        outSignal = this->msg_;
        outTimestamp = this->dataTs_;
        return true;
    }
};

/**
 * (Sub-thread) Timer callback function to request ControlChassis signal from ControllerServer.
 * @note Template specialization for ControlChassisReq, Chassis, ControlChassis.
 */
template<>
void ControllerClient<
    vehicle_interfaces::srv::ControlChassisReq, 
    vehicle_interfaces::msg::Chassis, 
    vehicle_interfaces::msg::ControlChassis
>
::_reqClientTmCbFunc()
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerClient::_reqClientTmCbFunc] Requesting control signal.");
    auto request = std::make_shared<vehicle_interfaces::srv::ControlChassisReq::Request>();
    request->request = true;
    auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(this->clientNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(this->clientNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto response = result.get();
        if (response->response)
        {
            this->_safeSave(&this->pubMsg_, response->value, this->pubMsgLock_);// Save control signal to be published.

            std::unique_lock<std::mutex> locker(this->msgLock_, std::defer_lock);
            locker.lock();
            this->msg_ = response->value;
            this->frameID_ = response->frame_id;
            this->dataTs_ = std::chrono::high_resolution_clock::now();
            locker.unlock();
            if (this->interruptFuncF_ && response->value.controller_interrupt)
                this->interruptFunc_(this->cInfo_);
        }
    }
}

/**
 * (Sub-thread) Subscription callback function to receive Chassis signal from ControllerServer.
 * @note Template specialization for ControlChassisReq, Chassis, ControlChassis.
 */
template<>
void ControllerClient<
    vehicle_interfaces::srv::ControlChassisReq, 
    vehicle_interfaces::msg::Chassis, 
    vehicle_interfaces::msg::ControlChassis
>
::_subCbFunc(const std::shared_ptr<vehicle_interfaces::msg::Chassis> msg)
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerClient::_subCbFunc] Received control signal.");
    auto tmp = vehicle_interfaces::msg_to_msg::ControlChassis::convert(*msg);
    this->_safeSave(&this->pubMsg_, tmp, this->pubMsgLock_);// Save control signal to be published.

    std::unique_lock<std::mutex> locker(this->msgLock_, std::defer_lock);
    locker.lock();
    this->msg_ = tmp;
    this->frameID_ = msg->header.frame_id;
    this->dataTs_ = std::chrono::high_resolution_clock::now();
    locker.unlock();
    if (this->interruptFuncF_ && tmp.controller_interrupt)
        this->interruptFunc_(this->cInfo_);
}

template<>
void ControllerClient<
    vehicle_interfaces::srv::ControlChassisReq, 
    vehicle_interfaces::msg::Chassis, 
    vehicle_interfaces::msg::ControlChassis
>
::_publish()
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerClient::_publish] Publishing control signal.");
    auto tmp = this->_safeCall(&this->pubMsg_, this->pubMsgLock_);// Get control signal to be published.
    auto cvt = vehicle_interfaces::msg_to_msg::Chassis::convert(tmp);
    cvt.controller_name = this->cInfo_.service_name;
    this->_getHeader(cvt.header);// Set header.
    this->pub_->publish(cvt);
}

/**
 * (Sub-thread) Timer callback function to request ControlSteeringWheel signal from ControllerServer. The ControlSteeringWheel signal will be converted to ControlChassis signal.
 * @note Template specialization for ControlSteeringWheelReq, SteeringWheel, ControlSteeringWheel.
 * @note The cvtFunc_ should be set before calling this function.
 */
template<>
void ControllerClient<
    vehicle_interfaces::srv::ControlSteeringWheelReq, 
    vehicle_interfaces::msg::SteeringWheel, 
    vehicle_interfaces::msg::ControlSteeringWheel
>
::_reqClientTmCbFunc()
{
    if (!this->cvtFuncF_)
        return;
    // RCLCPP_INFO(this->get_logger(), "[ControllerClient::_reqClientTmCbFunc] Requesting control signal.");
    auto request = std::make_shared<vehicle_interfaces::srv::ControlSteeringWheelReq::Request>();
    request->request = true;
    auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(this->clientNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(this->clientNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto response = result.get();
        if (response->response)
        {
            this->_safeSave(&this->pubMsg_, response->value, this->pubMsgLock_);// Save control signal to be published.

            vehicle_interfaces::msg::ControlChassis tmp;
            if (!this->cvtFunc_(response->value, tmp))
                return;
            std::unique_lock<std::mutex> locker(this->msgLock_, std::defer_lock);
            locker.lock();
            this->msg_ = tmp;
            this->frameID_ = response->frame_id;
            this->dataTs_ = std::chrono::high_resolution_clock::now();
            locker.unlock();
            if (this->interruptFuncF_ && tmp.controller_interrupt)
                this->interruptFunc_(this->cInfo_);
        }
    }
}

/**
 * (Sub-thread) Subscription callback function to receive SteeringWheel signal from ControllerServer. The SteeringWheel signal will be converted to ControlChassis signal.
 * @note Template specialization for ControlSteeringWheelReq, SteeringWheel, ControlSteeringWheel.
 * @note The cvtFunc_ should be set before calling this function.
 */
template<>
void ControllerClient<
    vehicle_interfaces::srv::ControlSteeringWheelReq, 
    vehicle_interfaces::msg::SteeringWheel, 
    vehicle_interfaces::msg::ControlSteeringWheel
>
::_subCbFunc(const std::shared_ptr<vehicle_interfaces::msg::SteeringWheel> msg)
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerClient::_subCbFunc] Received control signal.");
    if (!this->cvtFuncF_)
        return;
    auto cvt = vehicle_interfaces::msg_to_msg::ControlSteeringWheel::convert(*msg);
    this->_safeSave(&this->pubMsg_, cvt, this->pubMsgLock_);// Save control signal to be published.

    vehicle_interfaces::msg::ControlChassis tmp;
    if (!this->cvtFunc_(cvt, tmp))
        return;
    std::unique_lock<std::mutex> locker(this->msgLock_, std::defer_lock);
    locker.lock();
    this->msg_ = tmp;
    this->frameID_ = msg->controller_frame_id;
    this->dataTs_ = std::chrono::high_resolution_clock::now();
    locker.unlock();
    if (this->interruptFuncF_ && tmp.controller_interrupt)
        this->interruptFunc_(this->cInfo_);
}

template<>
void ControllerClient<
    vehicle_interfaces::srv::ControlSteeringWheelReq, 
    vehicle_interfaces::msg::SteeringWheel, 
    vehicle_interfaces::msg::ControlSteeringWheel
>
::_publish()
{
    // RCLCPP_INFO(this->get_logger(), "[ControllerClient::_publish] Publishing control signal.");
    auto tmp = this->_safeCall(&this->pubMsg_, this->pubMsgLock_);// Get control signal to be published.
    auto cvt = vehicle_interfaces::msg_to_msg::SteeringWheel::convert(tmp);
    cvt.controller_name = this->cInfo_.service_name;
    this->_getHeader(cvt.header);// Set header.
    this->pub_->publish(cvt);
}

/**
 * Some useful typedefs.
 */
using ChassisControllerServer = ControllerServer<
    vehicle_interfaces::srv::ControlChassisReq, 
    vehicle_interfaces::msg::Chassis, 
    vehicle_interfaces::msg::ControlChassis>;

using SteeringWheelControllerServer = ControllerServer<
    vehicle_interfaces::srv::ControlSteeringWheelReq, 
    vehicle_interfaces::msg::SteeringWheel, 
    vehicle_interfaces::msg::ControlSteeringWheel>;

using ChassisControllerClient = ControllerClient<
    vehicle_interfaces::srv::ControlChassisReq, 
    vehicle_interfaces::msg::Chassis, 
    vehicle_interfaces::msg::ControlChassis>;

using SteeringWheelControllerClient = ControllerClient<
    vehicle_interfaces::srv::ControlSteeringWheelReq, 
    vehicle_interfaces::msg::SteeringWheel, 
    vehicle_interfaces::msg::ControlSteeringWheel>;

}