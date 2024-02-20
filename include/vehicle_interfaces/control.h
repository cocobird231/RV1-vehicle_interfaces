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

#define SERVICE_REQUEST(T) std::shared_ptr<T::Request>
#define SERVICE_RESPONSE(T) std::shared_ptr<T::Response>

using namespace std::chrono_literals;


namespace vehicle_interfaces
{

class BaseControllerServer : public rclcpp::Node
{
private:
    std::shared_ptr<rclcpp::Node> controllerInfoRegClientNode_;// Client node for controller info registration.
    rclcpp::Client<vehicle_interfaces::srv::ControllerInfoReg>::SharedPtr controllerInfoRegClient_;// Client to register controller info.
    std::atomic<bool> availableF_;// Whether the controller is available.

protected:
    const vehicle_interfaces::msg::ControllerInfo cInfo_;

public:
    BaseControllerServer(const vehicle_interfaces::msg::ControllerInfo& cInfo, const std::string& controlServiceName) : 
        rclcpp::Node(cInfo.service_name), 
        cInfo_(cInfo), 
        availableF_(false)
    {
        this->controllerInfoRegClientNode_ = std::make_shared<rclcpp::Node>(cInfo.service_name + "_controlinforeg_client");
        this->controllerInfoRegClient_ = this->controllerInfoRegClientNode_->create_client<vehicle_interfaces::srv::ControllerInfoReg>(controlServiceName + "_Reg");
    }

    bool registerControllerInfo()
    {
        auto request = std::make_shared<vehicle_interfaces::srv::ControllerInfoReg::Request>();
        request->request = this->cInfo_;
        auto result = this->controllerInfoRegClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->controllerInfoRegClientNode_, result, 200ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->controllerInfoRegClientNode_, result, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
            {
                this->availableF_ = true;
                return this->availableF_;
            }
        }
        this->availableF_ = false;
        return false;
    }
};

template <typename serviceT, typename topicT, typename controlT>
class ControllerServer : public BaseControllerServer
{
private:
    std::shared_ptr<rclcpp::Service<serviceT> > reqServer_;// Server to response control signal.
    std::mutex reqServerLock_;// Lock reqServer_.
    std::atomic<uint64_t> reqServerFrameId_;// The counter of service response.

    std::shared_ptr<rclcpp::Publisher<topicT> > pub_;// Publisher to publish control signal.
    vehicle_interfaces::Timer* pubTm_;// Timer to publish control signal.
    std::atomic<uint64_t> pubFrameId_;// The counter of control signal published.

    controlT msg_;// Control signal.
    std::mutex msgLock_;// Lock msg_.
    
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
    void _reqServerCbFunc(const SERVICE_REQUEST(serviceT) request, SERVICE_RESPONSE(serviceT) response)
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
        msg.device_id = this->cInfo_.service_name;
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
    ControllerServer(const vehicle_interfaces::msg::ControllerInfo& cInfo, const std::string& controlServiceName) : 
        BaseControllerServer(cInfo, controlServiceName), 
        pubTm_(nullptr), 
        reqServerFrameId_(0), 
        pubFrameId_(0)
    {
        if (cInfo.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_SERVICE)
        {
            this->reqServer_ = this->create_service<serviceT>(cInfo.service_name, 
                                std::bind(&ControllerServer::_reqServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
        }

        if (cInfo.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_TOPIC || 
            cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER || 
            cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH)
        {
            this->pub_ = this->create_publisher<topicT>(cInfo.service_name, 10);
            this->pubTm_ = new vehicle_interfaces::Timer(cInfo.period_ms, std::bind(&ControllerServer::_publish, this));
            this->pubTm_->start();
        }
    }

    ~ControllerServer()
    {
        // Destroy publisher timer.
        if (this->pubTm_ != nullptr)
        {
            this->pubTm_->destroy();
            delete this->pubTm_;
        }
    }

    /**
     * Set control signal.
     * @param[in] msg Control signal supports ControlChassis or ControlSteeringWheel.
     */
    void setControlSignal(const controlT& msg)
    {
        std::lock_guard<std::mutex> _lock(this->msgLock_);
        this->msg_ = msg;
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
::_reqServerCbFunc(const SERVICE_REQUEST(vehicle_interfaces::srv::ControlChassisReq) request, 
                    SERVICE_RESPONSE(vehicle_interfaces::srv::ControlChassisReq) response)
{
    RCLCPP_INFO(this->get_logger(), "[ControllerServer::_reqServerCbFunc] Received request.");
    std::lock_guard<std::mutex> reqServerLocker(this->reqServerLock_);// Restrict only one callback at a time.
    std::lock_guard<std::mutex> msgLocker(this->msgLock_);
    response->response = true;
    response->value = this->msg_;
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
    RCLCPP_INFO(this->get_logger(), "[ControllerServer::_publish] Publishing control signal.");
    std::lock_guard<std::mutex> _lock(this->msgLock_);
    vehicle_interfaces::msg::Chassis tmp;
    tmp = vehicle_interfaces::msg_to_msg::Chassis::convert(this->msg_);
    tmp.controller_name = this->cInfo_.service_name;
    this->_getHeader(tmp.header);// Set header.
    this->pub_->publish(tmp);
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
::_reqServerCbFunc(const SERVICE_REQUEST(vehicle_interfaces::srv::ControlSteeringWheelReq) request, 
                    SERVICE_RESPONSE(vehicle_interfaces::srv::ControlSteeringWheelReq) response)
{
    RCLCPP_INFO(this->get_logger(), "[ControllerServer::_reqServerCbFunc] Received request.");
    std::lock_guard<std::mutex> reqServerLocker(this->reqServerLock_);// Restrict only one callback at a time.
    std::lock_guard<std::mutex> msgLocker(this->msgLock_);
    response->response = true;
    response->value = this->msg_;
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
    RCLCPP_INFO(this->get_logger(), "[ControllerServer::_publish] Publishing control signal.");
    vehicle_interfaces::msg::SteeringWheel tmp;
    std::lock_guard<std::mutex> _lock(this->msgLock_);
    tmp = vehicle_interfaces::msg_to_msg::SteeringWheel::convert(this->msg_);
    tmp.controller_name = this->cInfo_.service_name;
    this->_getHeader(tmp.header);// Set header.
    this->pub_->publish(tmp);
}


class BaseControllerClient : public rclcpp::Node
{
public:
    BaseControllerClient(const std::string& nodeName) : rclcpp::Node(nodeName) {}
    virtual void close() = 0;
    virtual vehicle_interfaces::msg::ControllerInfo getInfo() const = 0;
    virtual bool getSignal(vehicle_interfaces::msg::ControlChassis& outSignal, std::chrono::high_resolution_clock::time_point& outTimestamp) = 0;
};

template <typename serviceT, typename topicT, typename controlT>
class ControllerClient : public BaseControllerClient
{
private:
    const vehicle_interfaces::msg::ControllerInfo cInfo_;

    std::shared_ptr<rclcpp::Node> clientNode_;// Node for client.
    std::shared_ptr<rclcpp::Client<serviceT> > reqClient_;// Client to request control signal.
    vehicle_interfaces::Timer* reqClientTm_;// Call _reqClientTmCbFunc().
    std::chrono::duration<float, std::milli> serviceTimeout_;// Timeout for service request.

    std::shared_ptr<rclcpp::Subscription<topicT> > sub_;// Subscription to receive control signal.

    std::shared_ptr<rclcpp::Node> pubNode_;// Node for publisher.
    std::shared_ptr<rclcpp::Publisher<topicT> > pub_;// Publisher to publish control signal.
    vehicle_interfaces::Timer* pubTm_;// Timer to publish control signal.
    std::atomic<uint64_t> pubFrameId_;// The counter of control signal published.

    std::function<bool(const vehicle_interfaces::msg::ControlSteeringWheel&, vehicle_interfaces::msg::ControlChassis&)> cvtFunc_;
    std::atomic<bool> cvtFuncF_;

    vehicle_interfaces::msg::ControlChassis msg_;// Control signal.
    std::atomic<uint64_t> frameID_;// The counter of service req/res.
    std::chrono::high_resolution_clock::time_point dataTs_;// Timestamp of grabbed control signal.
    std::mutex msgLock_;// Lock msg_.
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
        msg.device_id = this->cInfo_.service_name;
        msg.frame_id = this->pubFrameId_++;
        msg.stamp_type = vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        msg.stamp = this->get_clock()->now();
        msg.stamp_offset = 0;
        msg.ref_publish_time_ms = this->cInfo_.period_ms;
    }

public:
    ControllerClient(const vehicle_interfaces::msg::ControllerInfo& cInfo) : 
        BaseControllerClient(cInfo.service_name + "_controllerclient"),
        cInfo_(cInfo), 
        reqClientTm_(nullptr), 
        pubTm_(nullptr), 
        pubFrameId_(0), 
        cvtFuncF_(false), 
        availableF_(false), 
        exitF_(false)
    {
        std::string nodeName = cInfo.service_name + "_controllerclient";
        if (cInfo.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_SERVICE)
        {
            this->serviceTimeout_ = std::chrono::duration<float, std::milli>(cInfo.timeout_ms);
            this->clientNode_ = std::make_shared<rclcpp::Node>(nodeName + "_client");
            this->reqClient_ = this->clientNode_->create_client<serviceT>(cInfo.service_name);
            // Create timer.
            this->reqClientTm_ = new vehicle_interfaces::Timer(cInfo.period_ms, std::bind(&ControllerClient::_reqClientTmCbFunc, this));
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

        if (cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLSERVER || 
            cInfo.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH)
        {
            this->pubNode_ = std::make_shared<rclcpp::Node>(cInfo.service_name + "_controllerclient_pub");
            this->pub_ = this->pubNode_->create_publisher<topicT>(cInfo.service_name + "_controllerclient", 10);
            this->pubTm_ = new vehicle_interfaces::Timer(cInfo.period_ms, std::bind(&ControllerClient::_publish, this));
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
        // Destroy publisher timer.
        if (this->pubTm_ != nullptr)
        {
            this->pubTm_->destroy();
            delete this->pubTm_;
        }
        // Destroy client timer.
        if (this->reqClientTm_ != nullptr)
        {
            this->reqClientTm_->destroy();
            delete this->reqClientTm_;
        }
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
     * Get ControllerInfo for this ControllerClient.
     * @return ControllerInfo message.
     */
    vehicle_interfaces::msg::ControllerInfo getInfo() const
    {
        return this->cInfo_;
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
    RCLCPP_INFO(this->get_logger(), "[ControllerClient::_reqClientTmCbFunc] Requesting control signal.");
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
            std::lock_guard<std::mutex> locker(this->msgLock_);
            this->msg_ = response->value;
            this->frameID_ = response->frame_id;
            this->dataTs_ = std::chrono::high_resolution_clock::now();
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
    RCLCPP_INFO(this->get_logger(), "[ControllerClient::_subCbFunc] Received control signal.");
    std::lock_guard<std::mutex> locker(this->msgLock_);
    this->msg_ = vehicle_interfaces::msg_to_msg::ControlChassis::convert(*msg);
    this->frameID_ = msg->header.frame_id;
    this->dataTs_ = std::chrono::high_resolution_clock::now();
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
    RCLCPP_INFO(this->get_logger(), "[ControllerClient::_reqClientTmCbFunc] Requesting control signal.");
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
            vehicle_interfaces::msg::ControlChassis tmp;
            if (!this->cvtFunc_(response->value, tmp))
                return;
            std::lock_guard<std::mutex> locker(this->msgLock_);
            this->msg_ = tmp;
            this->frameID_ = response->frame_id;
            this->dataTs_ = std::chrono::high_resolution_clock::now();
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
    RCLCPP_INFO(this->get_logger(), "[ControllerClient::_subCbFunc] Received control signal.");
    if (!this->cvtFuncF_)
        return;
    vehicle_interfaces::msg::ControlChassis tmp;
    if (!this->cvtFunc_(vehicle_interfaces::msg_to_msg::ControlSteeringWheel::convert(*msg), tmp))
        return;
    std::lock_guard<std::mutex> locker(this->msgLock_);
    this->msg_ = tmp;
    this->frameID_ = msg->controller_frame_id;
    this->dataTs_ = std::chrono::high_resolution_clock::now();
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

/**
 * The Controller class declared the communications between ControlServer and Controller. 
 * The user should manually convert the control signal to either ControlChassis or ControlSteeringWheel message type.
 * 
 * The Controller class have 4 different operation mode which defined by ControllerInfo message while construction, 
 * including:
 * - ControlChassis as server (ControlChassisReq)
 * - ControlSteeringWheel as server (ControlSteeringWheelReq)
 * - ControlChassis as client (ControlChassisReg)
 * - ControlSteeringWheel as client (ControlSteeringWheelReg)
 * 
 * Note: The Controller class will established several threads for ROS2 node and timers since construction, 
 * and automatically recycled while destruction.
 */
// class Controller : public vehicle_interfaces::PseudoTimeSyncNode, public vehicle_interfaces::QoSUpdateNode
// {
// private:
//     std::string nodeName_;// Node name, topic name.
//     const vehicle_interfaces::msg::ControllerInfo info_;
//     std::mutex infoLock_;
//     rclcpp::executors::SingleThreadedExecutor *exec_;
//     std::thread *th_;
//     std::atomic<bool> enableF_;

//     // Service
//     std::string serviceNodeName_;// Server, client node name.
//     std::shared_ptr<rclcpp::Node> serviceNode_;
//     // Controller set to server, receive request controller signal from ControlServerController.
//     rclcpp::Service<vehicle_interfaces::srv::ControlChassisReq>::SharedPtr controlChassisReqServer_;
//     rclcpp::Service<vehicle_interfaces::srv::ControlSteeringWheelReq>::SharedPtr controlSteeringWheelReqServer_;
//     std::mutex serverCbLock_;
//     // Controller set to client, send controller signal to ControlServerController.
//     rclcpp::Client<vehicle_interfaces::srv::ControlChassisReg>::SharedPtr controlChassisRegClient_;
//     rclcpp::Client<vehicle_interfaces::srv::ControlSteeringWheelReg>::SharedPtr controlSteeringWheelRegClient_;
//     vehicle_interfaces::Timer* chassisRegTm_;
//     vehicle_interfaces::Timer* steeringWheelRegTm_;

//     // Publisher
//     rclcpp::Publisher<vehicle_interfaces::msg::Chassis>::SharedPtr chassisPub_;
//     rclcpp::Publisher<vehicle_interfaces::msg::SteeringWheel>::SharedPtr steeringWheelPub_;
//     std::atomic<bool> pubF_;

//     // Store controller data.
//     vehicle_interfaces::msg::ControlChassis controlChassisData_;
//     vehicle_interfaces::msg::ControlSteeringWheel controlSteeringWheelData_;
//     std::mutex dataLock_;

//     // Controller signal
//     std::atomic<u_int64_t> frameID_;// The counter of service req/res.
//     std::chrono::duration<float, std::milli> serviceTimeout_;

// private:
//     template <typename T>
//     void _safeSave(T* ptr, const T value, std::mutex& lock)
//     {
//         std::lock_guard<std::mutex> _lock(lock);
//         *ptr = value;
//     }

//     template <typename T>
//     T _safeCall(const T* ptr, std::mutex& lock)
//     {
//         std::lock_guard<std::mutex> _lock(lock);
//         return *ptr;
//     }

//     void _initData()
//     {
//         // Init controlChassisData_.
//         this->controlChassisData_.drive_motor = { 0, 0, 0, 0 };
//         this->controlChassisData_.steering_motor = { 0, 0, 0, 0 };
//         this->controlChassisData_.brake_motor = { 0, 0, 0, 0 };
//         this->controlChassisData_.parking_signal = { true, true, true, true };

//         // Init controlSteeringWheelData_.
//         // All init by default.
//     }

//     bool _publishChassisMsg()
//     {
//         if (!this->pubF_)
//             return false;
//         auto data = _safeCall(&this->controlChassisData_, this->dataLock_);
//         auto info = _safeCall(&this->info_, this->infoLock_);
//         if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//         {
//             auto msg = vehicle_interfaces::msg::Chassis();
//             msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
//             msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
//             msg.header.device_id = this->nodeName_;
//             msg.header.frame_id = this->frameID_.load();
//             msg.header.stamp_type = this->getTimestampType();
//             msg.header.stamp = this->getTimestamp();
//             msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
//             msg.header.ref_publish_time_ms = info.period_ms;

//             msg.unit_type = data.unit_type;
//             msg.drive_motor = data.drive_motor;
//             msg.steering_motor = data.steering_motor;
//             msg.brake_motor = data.brake_motor;
//             msg.parking_signal = data.parking_signal;
//             msg.controller_name = this->nodeName_;
//             this->chassisPub_->publish(msg);
//         }
//     }

//     bool _publishSteeringWheelMsg()
//     {
//         if (!this->pubF_)
//             return false;
//         auto data = _safeCall(&this->controlSteeringWheelData_, this->dataLock_);
//         auto info = _safeCall(&this->info_, this->infoLock_);
//         if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//         {
//             auto msg = vehicle_interfaces::msg::SteeringWheel();
//             msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
//             msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
//             msg.header.device_id = this->nodeName_;
//             msg.header.frame_id = this->frameID_.load();
//             msg.header.stamp_type = this->getTimestampType();
//             msg.header.stamp = this->getTimestamp();
//             msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
//             msg.header.ref_publish_time_ms = info.period_ms;

//             msg.gear = data.gear;
//             msg.steering = data.steering;
//             msg.pedal_throttle = data.pedal_throttle;
//             msg.pedal_brake = data.pedal_brake;
//             msg.pedal_clutch = data.pedal_clutch;
//             msg.func_0 = data.func_0;
//             msg.func_1 = data.func_1;
//             msg.func_2 = data.func_2;
//             msg.func_3 = data.func_3;
//             msg.controller_name = this->nodeName_;
//             this->steeringWheelPub_->publish(msg);
//         }
//     }
//     // ControlServerController set to ControlChassisReq server
//     void _controlChassisReqCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlChassisReq::Request> request, 
//                             std::shared_ptr<vehicle_interfaces::srv::ControlChassisReq::Response> response)
//     {
//         std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
//         try
//         {
//             response->value = this->_safeCall(&this->controlChassisData_, this->dataLock_);
//             response->frame_id = this->frameID_.load();
//             this->frameID_++;
//             this->_publishChassisMsg();
//             response->response = true;
//         }
//         catch(const std::exception& e)
//         {
//             RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlChassisReqCbFunc] %s", e.what());
//             response->response = false;
//         }
//         catch(...)
//         {
//             RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlChassisReqCbFunc] Caught unknown exception.");
//             response->response = false;
//         }
//     }

//     // ControlServerController set to ControlSteeringWheelReq server
//     void _controlSteeringWheelReqCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReq::Request> request, 
//                             std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReq::Response> response)
//     {
//         std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
//         try
//         {
//             response->value = this->_safeCall(&this->controlSteeringWheelData_, this->dataLock_);
//             response->frame_id = this->frameID_.load();
//             this->frameID_++;
//             this->_publishSteeringWheelMsg();
//             response->response = true;
//         }
//         catch(const std::exception& e)
//         {
//             RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlSteeringWheelReqCbFunc] %s", e.what());
//             response->response = false;
//         }
//         catch(...)
//         {
//             RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlSteeringWheelReqCbFunc] Caught unknown exception.");
//             response->response = false;
//         }
//     }

//     // ControlServerController set to ControlChassisReg client. Called by Timer.
//     void _controlChassisRegCbFunc()
//     {
//         auto request = std::make_shared<vehicle_interfaces::srv::ControlChassisReg::Request>();
//         request->value = this->_safeCall(&this->controlChassisData_, this->dataLock_);
//         request->frame_id = this->frameID_.load();
//         this->frameID_++;
//         this->_publishChassisMsg();
//         auto result = this->controlChassisRegClient_->async_send_request(request);
// #if ROS_DISTRO == 0
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
// #else
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
// #endif
//         {
//             auto response = result.get();
//             if (!response->response)
//             {
//                 RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlChassisRegCbFunc] Request failed.");
//             }
//         }
//     }

//     // ControlServerController set to ControlSteeringWheelReg client. Called by Timer.
//     void _controlSteeringWheelRegCbFunc()
//     {
//         auto request = std::make_shared<vehicle_interfaces::srv::ControlSteeringWheelReg::Request>();
//         request->value = this->_safeCall(&this->controlSteeringWheelData_, this->dataLock_);
//         request->frame_id = this->frameID_.load();
//         this->frameID_++;
//         this->_publishSteeringWheelMsg();
//         auto result = this->controlSteeringWheelRegClient_->async_send_request(request);
// #if ROS_DISTRO == 0
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
// #else
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
// #endif
//         {
//             auto response = result.get();
//             if (!response->response)
//             {
//                 RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlSteeringWheelRegCbFunc] Request failed.");
//             }
//         }
//     }

// public:
//     Controller(const std::shared_ptr<vehicle_interfaces::GenericParams>& params, vehicle_interfaces::msg::ControllerInfo info) : 
//         vehicle_interfaces::PseudoTimeSyncNode(info.service_name + "_controller"), 
//         vehicle_interfaces::QoSUpdateNode(info.service_name + "_controller", params->qosService, params->qosDirPath), 
//         rclcpp::Node(info.service_name + "_controller"), 
//         info_(info), 
//         exec_(nullptr), 
//         th_(nullptr), 
//         enableF_(false), 
//         chassisRegTm_(nullptr), 
//         steeringWheelRegTm_(nullptr), 
//         pubF_(false), 
//         frameID_(0)
//     {
//         this->nodeName_ = info.service_name + "_controller";
//         this->serviceTimeout_ = std::chrono::duration<float, std::milli>(info.timeout_ms);
//         this->pubF_ = (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER) || 
//                         (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH);
//         this->_initData();

//         // Controller set to client, ControlServerController set to server
//         if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_CLIENT)
//         {
//             this->serviceNodeName_ = this->nodeName_ + "_client";
//             this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
//             if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//             {
//                 this->controlChassisRegClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlChassisReg>(info.service_name);
//                 this->chassisRegTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&Controller::_controlChassisRegCbFunc, this));
//                 if (this->pubF_)
//                     this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
//                 this->chassisRegTm_->start();
//             }
//             else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
//             {
//                 this->controlSteeringWheelRegClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlSteeringWheelReg>(info.service_name);
//                 this->steeringWheelRegTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&Controller::_controlSteeringWheelRegCbFunc, this));
//                 if (this->pubF_)
//                     this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
//                 this->steeringWheelRegTm_->start();
//             }
//             else
//             {
//                 throw "[Controller] construct info.msg_type error.";
//             }
//         }
//         // Controller set to server, ControlServerController set to client.
//         else if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_SERVER)
//         {
//             this->serviceNodeName_ = this->nodeName_ + "_server";
//             this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
//             if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//             {
//                 this->controlChassisReqServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlChassisReq>(info.service_name, 
//                     std::bind(&Controller::_controlChassisReqCbFunc, this, std::placeholders::_1, std::placeholders::_2));
//                 if (this->pubF_)
//                     this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
//             }
//             else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
//             {
//                 this->controlSteeringWheelReqServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlSteeringWheelReq>(info.service_name, 
//                     std::bind(&Controller::_controlSteeringWheelReqCbFunc, this, std::placeholders::_1, std::placeholders::_2));
//                 if (this->pubF_)
//                     this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
//             }
//             else
//             {
//                 throw "[Controller] construct info.msg_type error.";
//             }
//         }
//         else
//         {
//             throw "[Controller] construct info.service_mode error.";
//         }
//         // this->exec_ = new rclcpp::executors::SingleThreadedExecutor();
//         // this->exec_->add_node(this->serviceNode_);
//         // this->th_ = new std::thread(vehicle_interfaces::SpinExecutor, this->exec_, this->serviceNodeName_, 1000);
//         this->enableF_ = true;
//         RCLCPP_INFO(this->get_logger(), "[Controller] Controller constructed: %s", this->serviceNodeName_.c_str());
//         if (this->pubF_)
//             RCLCPP_INFO(this->get_logger(), "[Controller] Create publisher: %s", this->serviceNodeName_.c_str());
//     }

//     ~Controller()
//     {
//         if (this->exec_ != nullptr)
//             this->exec_->cancel();
//         if (this->th_ != nullptr)
//             this->th_->join();
//         if (this->chassisRegTm_ != nullptr)
//         {
//             this->chassisRegTm_->destroy();
//             delete this->chassisRegTm_;
//         }
//         if (this->steeringWheelRegTm_ != nullptr)
//         {
//             this->steeringWheelRegTm_->destroy();
//             delete this->steeringWheelRegTm_;
//         }
//     }

//     void setControlChassisData(const vehicle_interfaces::msg::ControlChassis& msg)
//     {
//         this->_safeSave(&this->controlChassisData_, msg, this->dataLock_);
//     }

//     void setControlSteeringWheelData(const vehicle_interfaces::msg::ControlSteeringWheel& msg)
//     {
//         this->_safeSave(&this->controlSteeringWheelData_, msg, this->dataLock_);
//     }
// };

/**
 * The ControlServerController class holds the registered Controller information under ControlServer, and implements the control
 * signal requisition from Controller.
 * 
 * The ControlServerController class have 4 different operation mode which defined by ControllerInfo message while construction, 
 * including:
 * - ControlChassis as server (ControlChassisReg)
 * - ControlSteeringWheel as server (ControlSteeringWheelReg)
 * - ControlChassis as client (ControlChassisReq)
 * - ControlSteeringWheel as client (ControlSteeringWheelReq)
 * 
 * Note: The ControlServerController class will established several threads for ROS2 node and timers since construction, 
 * and automatically recycled while destruction.
 */
// class ControlServerController : public vehicle_interfaces::PseudoTimeSyncNode, public vehicle_interfaces::QoSUpdateNode
// {
// private:
//     std::string nodeName_;
//     const vehicle_interfaces::msg::ControllerInfo info_;
//     std::mutex infoLock_;
//     rclcpp::executors::SingleThreadedExecutor *exec_;
//     std::thread *th_;
//     std::atomic<bool> enableF_;

//     // ControlServerController set to server, receive controller signal from Controller.
//     std::string serviceNodeName_;
//     std::shared_ptr<rclcpp::Node> serviceNode_;
//     rclcpp::Service<vehicle_interfaces::srv::ControlChassisReg>::SharedPtr controlChassisRegServer_;
//     rclcpp::Service<vehicle_interfaces::srv::ControlSteeringWheelReg>::SharedPtr controlSteeringWheelRegServer_;
//     std::mutex serverCbLock_;

//     // ControlServerController set to client, send request controller signal to Controller.
//     rclcpp::Client<vehicle_interfaces::srv::ControlChassisReq>::SharedPtr controlChassisReqClient_;
//     rclcpp::Client<vehicle_interfaces::srv::ControlSteeringWheelReq>::SharedPtr controlSteeringWheelReqClient_;
//     vehicle_interfaces::Timer* chassisReqTm_;
//     vehicle_interfaces::Timer* steeringWheelReqTm_;

//     // Publisher
//     rclcpp::Publisher<vehicle_interfaces::msg::Chassis>::SharedPtr chassisPub_;
//     rclcpp::Publisher<vehicle_interfaces::msg::SteeringWheel>::SharedPtr steeringWheelPub_;
//     std::atomic<bool> pubF_;

//     std::function<bool(const vehicle_interfaces::msg::ControlSteeringWheel&, vehicle_interfaces::msg::ControlChassis&)> cvtFunc_;
//     std::atomic<bool> cvtFuncF_;

//     vehicle_interfaces::msg::ControlChassis controlChassisData_;
//     vehicle_interfaces::msg::ControlSteeringWheel controlSteeringWheelData_;
//     std::atomic<u_int64_t> frameID_;
//     std::chrono::high_resolution_clock::time_point dataTs_;
//     std::mutex dataLock_;

//     std::chrono::duration<float, std::milli> serviceTimeout_;

// private:
//     template <typename T>
//     void _safeSave(T* ptr, const T value, std::mutex& lock)
//     {
//         std::lock_guard<std::mutex> _lock(lock);
//         *ptr = value;
//     }

//     template <typename T>
//     T _safeCall(const T* ptr, std::mutex& lock)
//     {
//         std::lock_guard<std::mutex> _lock(lock);
//         return *ptr;
//     }

//     void _initData()
//     {
//         // Init controlChassisData_.
//         this->controlChassisData_.drive_motor = { 0, 0, 0, 0 };
//         this->controlChassisData_.steering_motor = { 0, 0, 0, 0 };
//         this->controlChassisData_.brake_motor = { 0, 0, 0, 0 };
//         this->controlChassisData_.parking_signal = { true, true, true, true };

//         // Init controlSteeringWheelData_.
//         // All init by default.
//     }

//     bool _publishChassisMsg()
//     {
//         if (!this->pubF_)
//             return false;
//         auto data = _safeCall(&this->controlChassisData_, this->dataLock_);
//         auto info = _safeCall(&this->info_, this->infoLock_);
//         if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//         {
//             auto msg = vehicle_interfaces::msg::Chassis();
//             msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
//             msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
//             msg.header.device_id = this->nodeName_;
//             msg.header.frame_id = this->frameID_.load();
//             msg.header.stamp_type = this->getTimestampType();
//             msg.header.stamp = this->getTimestamp();
//             msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
//             msg.header.ref_publish_time_ms = info.period_ms;

//             msg.unit_type = data.unit_type;
//             msg.drive_motor = data.drive_motor;
//             msg.steering_motor = data.steering_motor;
//             msg.brake_motor = data.brake_motor;
//             msg.parking_signal = data.parking_signal;
//             msg.controller_name = this->nodeName_;
//             this->chassisPub_->publish(msg);
//         }
//     }

//     bool _publishSteeringWheelMsg()
//     {
//         if (!this->pubF_)
//             return false;
//         auto data = _safeCall(&this->controlSteeringWheelData_, this->dataLock_);
//         auto info = _safeCall(&this->info_, this->infoLock_);
//         if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//         {
//             auto msg = vehicle_interfaces::msg::SteeringWheel();
//             msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
//             msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
//             msg.header.device_id = this->nodeName_;
//             msg.header.frame_id = this->frameID_.load();
//             msg.header.stamp_type = this->getTimestampType();
//             msg.header.stamp = this->getTimestamp();
//             msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
//             msg.header.ref_publish_time_ms = info.period_ms;

//             msg.gear = data.gear;
//             msg.steering = data.steering;
//             msg.pedal_throttle = data.pedal_throttle;
//             msg.pedal_brake = data.pedal_brake;
//             msg.pedal_clutch = data.pedal_clutch;
//             msg.func_0 = data.func_0;
//             msg.func_1 = data.func_1;
//             msg.func_2 = data.func_2;
//             msg.func_3 = data.func_3;
//             msg.controller_name = this->nodeName_;
//             this->steeringWheelPub_->publish(msg);
//         }
//     }

//     // ControlServerController set to ControlChassisReg server.
//     void _controlChassisRegCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlChassisReg::Request> request, 
//                             std::shared_ptr<vehicle_interfaces::srv::ControlChassisReg::Response> response)
//     {
//         std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
//         this->dataTs_ = std::chrono::high_resolution_clock::now();
//         this->_safeSave(&this->controlChassisData_, request->value, this->dataLock_);
//         this->frameID_ = request->frame_id;
//         this->_publishChassisMsg();
//     }

//     // ControlServerController set to ControlSteeringWheelReg server.
//     void _controlSteeringWheelRegCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReg::Request> request, 
//                             std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReg::Response> response)
//     {
//         std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
//         if (!this->cvtFuncF_.load())
//         {
//             response->response = false;
//             return;
//         }
//         // Publish SteeringWheel
//         this->dataTs_ = std::chrono::high_resolution_clock::now();
//         this->_safeSave(&this->controlSteeringWheelData_, request->value, this->dataLock_);
//         this->frameID_ = request->frame_id;
//         this->_publishSteeringWheelMsg();
//         // Convert to ControlChassis
//         vehicle_interfaces::msg::ControlChassis tmp;
//         if (!this->cvtFunc_(request->value, tmp))
//         {
//             response->response = false;
//             return;
//         }
//         this->_safeSave(&this->controlChassisData_, tmp, this->dataLock_);
//     }

//     // ControlServerController set to ControlChassisReq client. Called by Timer.
//     void _controlChassisReqCbFunc()
//     {
//         auto request = std::make_shared<vehicle_interfaces::srv::ControlChassisReq::Request>();
//         request->request = true;
//         auto result = this->controlChassisReqClient_->async_send_request(request);
// #if ROS_DISTRO == 0
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
// #else
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
// #endif
//         {
//             auto response = result.get();
//             if (response->response)
//             {
//                 this->_safeSave(&this->controlChassisData_, response->value, this->dataLock_);
//                 this->frameID_ = response->frame_id;
//                 this->_publishChassisMsg();
//             }
//         }
//     }

//     // ControlServerController set to ControlSteeringWheelReq client. Called by Timer.
//     void _controlSteeringWheelReqCbFunc()
//     {
//         auto request = std::make_shared<vehicle_interfaces::srv::ControlSteeringWheelReq::Request>();
//         request->request = true;
//         auto result = this->controlSteeringWheelReqClient_->async_send_request(request);
// #if ROS_DISTRO == 0
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
// #else
//         if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
// #endif
//         {
//             auto response = result.get();
//             if (response->response)
//             {
//                 if (!this->cvtFuncF_.load())
//                     return;
//                 // Publish SteeringWheel
//                 this->dataTs_ = std::chrono::high_resolution_clock::now();
//                 this->_safeSave(&this->controlSteeringWheelData_, response->value, this->dataLock_);
//                 this->frameID_ = response->frame_id;
//                 this->_publishSteeringWheelMsg();
//                 // Convert to ControlChassis
//                 vehicle_interfaces::msg::ControlChassis tmp;
//                 if (!this->cvtFunc_(response->value, tmp))
//                     return;
//                 this->_safeSave(&this->controlChassisData_, tmp, this->dataLock_);
//             }
//         }
//     }

// public:
//     ControlServerController(const std::shared_ptr<vehicle_interfaces::GenericParams>& params, const vehicle_interfaces::msg::ControllerInfo& info) : 
//         vehicle_interfaces::PseudoTimeSyncNode(info.service_name + "_controlserver"), 
//         vehicle_interfaces::QoSUpdateNode(info.service_name + "_controlserver", params->qosService, params->qosDirPath), 
//         rclcpp::Node(info.service_name + "_controlserver"), 
//         info_(info), 
//         exec_(nullptr), 
//         th_(nullptr), 
//         enableF_(false), 
//         chassisReqTm_(nullptr), 
//         steeringWheelReqTm_(nullptr), 
//         pubF_(false), 
//         frameID_(0), 
//         cvtFuncF_(false)
//     {
//         this->nodeName_ = info.service_name + "_controlserver";
//         this->serviceTimeout_ = std::chrono::duration<float, std::milli>(info.timeout_ms);
//         this->pubF_ = (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLSERVER) || 
//                         (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH);
//         this->_initData();

//         // Controller set to client, ControlServerController set to server
//         if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_CLIENT)
//         {
//             this->serviceNodeName_ = this->nodeName_ + "_server";
//             this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
//             if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//             {
//                 this->controlChassisRegServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlChassisReg>(info.service_name, 
//                     std::bind(&ControlServerController::_controlChassisRegCbFunc, this, std::placeholders::_1, std::placeholders::_2));
//                 if (this->pubF_)
//                     this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
//             }
//             else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
//             {
//                 this->controlSteeringWheelRegServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlSteeringWheelReg>(info.service_name, 
//                     std::bind(&ControlServerController::_controlSteeringWheelRegCbFunc, this, std::placeholders::_1, std::placeholders::_2));
//                 if (this->pubF_)
//                     this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
//             }
//             else
//             {
//                 throw "[ControlServerController] construct info.msg_type error.";
//             }
//         }
//         // Controller set to server, ControlServerController set to client.
//         else if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_SERVER)
//         {
//             this->serviceNodeName_ = this->nodeName_ + "_client";
//             this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
//             if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
//             {
//                 this->controlChassisReqClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlChassisReq>(info.service_name);
//                 this->chassisReqTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&ControlServerController::_controlChassisReqCbFunc, this));
//                 if (this->pubF_)
//                     this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
//                 this->chassisReqTm_->start();
//             }
//             else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
//             {
//                 this->controlSteeringWheelReqClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlSteeringWheelReq>(info.service_name);
//                 this->steeringWheelReqTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&ControlServerController::_controlSteeringWheelReqCbFunc, this));
//                 if (this->pubF_)
//                     this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
//                 this->steeringWheelReqTm_->start();
//             }
//             else
//             {
//                 throw "[ControlServerController] construct info.msg_type error.";
//             }
//         }
//         else
//         {
//             throw "[ControlServerController] construct info.service_mode error.";
//         }
//         // this->exec_ = new rclcpp::executors::SingleThreadedExecutor();
//         // this->exec_->add_node(this->serviceNode_);
//         // this->th_ = new std::thread(vehicle_interfaces::SpinExecutor, this->exec_, this->serviceNodeName_, 1000);
//         this->enableF_ = true;
//         RCLCPP_INFO(this->get_logger(), "[ControlServerController] Controller constructed: %s", this->serviceNodeName_.c_str());
//         if (this->pubF_)
//             RCLCPP_INFO(this->get_logger(), "[ControlServerController] Create publisher: %s", this->serviceNodeName_.c_str());
//     }

//     ~ControlServerController()
//     {
//         if (this->exec_ != nullptr)
//             this->exec_->cancel();
//         if (this->th_ != nullptr)
//             this->th_->join();
//         if (this->chassisReqTm_ != nullptr)
//         {
//             this->chassisReqTm_->destroy();
//             delete this->chassisReqTm_;
//         }
//         if (this->steeringWheelReqTm_ != nullptr)
//         {
//             this->steeringWheelReqTm_->destroy();
//             delete this->steeringWheelReqTm_;
//         }
//     }

//     void setCvtFunc(const std::function<bool(const vehicle_interfaces::msg::ControlSteeringWheel&, vehicle_interfaces::msg::ControlChassis&)>& cvtFunc)
//     {
//         this->cvtFunc_ = cvtFunc;
//         this->cvtFuncF_ = true;
//     }

//     vehicle_interfaces::msg::ControllerInfo getInfo()
//     {
//         std::lock_guard<std::mutex> locker(this->infoLock_);
//         return this->info_;
//     }

//     bool getSignal(vehicle_interfaces::msg::ControlChassis& outSignal, std::chrono::high_resolution_clock::time_point& outTimestamp)
//     {
//         if (!this->enableF_)
//             return false;
//         std::lock_guard<std::mutex> locker(this->dataLock_);
//         outSignal = this->controlChassisData_;
//         outTimestamp = this->dataTs_;
//     }
// };

}