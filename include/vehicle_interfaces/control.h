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
class Controller : public vehicle_interfaces::PseudoTimeSyncNode, public vehicle_interfaces::QoSUpdateNode
{
private:
    std::string nodeName_;// Node name, topic name.
    const vehicle_interfaces::msg::ControllerInfo info_;
    std::mutex infoLock_;
    rclcpp::executors::SingleThreadedExecutor *exec_;
    std::thread *th_;
    std::atomic<bool> enableF_;

    // Service
    std::string serviceNodeName_;// Server, client node name.
    std::shared_ptr<rclcpp::Node> serviceNode_;
    // Controller set to server, receive request controller signal from ControlServerController.
    rclcpp::Service<vehicle_interfaces::srv::ControlChassisReq>::SharedPtr controlChassisReqServer_;
    rclcpp::Service<vehicle_interfaces::srv::ControlSteeringWheelReq>::SharedPtr controlSteeringWheelReqServer_;
    std::mutex serverCbLock_;
    // Controller set to client, send controller signal to ControlServerController.
    rclcpp::Client<vehicle_interfaces::srv::ControlChassisReg>::SharedPtr controlChassisRegClient_;
    rclcpp::Client<vehicle_interfaces::srv::ControlSteeringWheelReg>::SharedPtr controlSteeringWheelRegClient_;
    vehicle_interfaces::Timer* chassisRegTm_;
    vehicle_interfaces::Timer* steeringWheelRegTm_;

    // Publisher
    rclcpp::Publisher<vehicle_interfaces::msg::Chassis>::SharedPtr chassisPub_;
    rclcpp::Publisher<vehicle_interfaces::msg::SteeringWheel>::SharedPtr steeringWheelPub_;
    std::atomic<bool> pubF_;

    // Store controller data.
    vehicle_interfaces::msg::ControlChassis controlChassisData_;
    vehicle_interfaces::msg::ControlSteeringWheel controlSteeringWheelData_;
    std::mutex dataLock_;

    // Controller signal
    std::atomic<u_int64_t> frameID_;// The counter of service req/res.
    std::chrono::duration<float, std::milli> serviceTimeout_;

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

    void _initData()
    {
        // Init controlChassisData_.
        this->controlChassisData_.drive_motor = { 0, 0, 0, 0 };
        this->controlChassisData_.steering_motor = { 0, 0, 0, 0 };
        this->controlChassisData_.brake_motor = { 0, 0, 0, 0 };
        this->controlChassisData_.parking_signal = { true, true, true, true };

        // Init controlSteeringWheelData_.
        // All init by default.
    }

    bool _publishChassisMsg()
    {
        if (!this->pubF_)
            return false;
        auto data = _safeCall(&this->controlChassisData_, this->dataLock_);
        auto info = _safeCall(&this->info_, this->infoLock_);
        if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
        {
            auto msg = vehicle_interfaces::msg::Chassis();
            msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
            msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
            msg.header.device_id = this->nodeName_;
            msg.header.frame_id = this->frameID_.load();
            msg.header.stamp_type = this->getTimestampType();
            msg.header.stamp = this->getTimestamp();
            msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
            msg.header.ref_publish_time_ms = info.period_ms;

            msg.unit_type = data.unit_type;
            msg.drive_motor = data.drive_motor;
            msg.steering_motor = data.steering_motor;
            msg.brake_motor = data.brake_motor;
            msg.parking_signal = data.parking_signal;
            msg.controller_name = this->nodeName_;
            this->chassisPub_->publish(msg);
        }
    }

    bool _publishSteeringWheelMsg()
    {
        if (!this->pubF_)
            return false;
        auto data = _safeCall(&this->controlSteeringWheelData_, this->dataLock_);
        auto info = _safeCall(&this->info_, this->infoLock_);
        if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
        {
            auto msg = vehicle_interfaces::msg::SteeringWheel();
            msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
            msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
            msg.header.device_id = this->nodeName_;
            msg.header.frame_id = this->frameID_.load();
            msg.header.stamp_type = this->getTimestampType();
            msg.header.stamp = this->getTimestamp();
            msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
            msg.header.ref_publish_time_ms = info.period_ms;

            msg.gear = data.gear;
            msg.steering = data.steering;
            msg.pedal_throttle = data.pedal_throttle;
            msg.pedal_brake = data.pedal_brake;
            msg.pedal_clutch = data.pedal_clutch;
            msg.func_0 = data.func_0;
            msg.func_1 = data.func_1;
            msg.func_2 = data.func_2;
            msg.func_3 = data.func_3;
            msg.controller_name = this->nodeName_;
            this->steeringWheelPub_->publish(msg);
        }
    }
    // ControlServerController set to ControlChassisReq server
    void _controlChassisReqCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlChassisReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControlChassisReq::Response> response)
    {
        std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
        try
        {
            response->value = this->_safeCall(&this->controlChassisData_, this->dataLock_);
            response->frame_id = this->frameID_.load();
            this->frameID_++;
            this->_publishChassisMsg();
            response->response = true;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlChassisReqCbFunc] %s", e.what());
            response->response = false;
        }
        catch(...)
        {
            RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlChassisReqCbFunc] Caught unknown exception.");
            response->response = false;
        }
    }

    // ControlServerController set to ControlSteeringWheelReq server
    void _controlSteeringWheelReqCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReq::Response> response)
    {
        std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
        try
        {
            response->value = this->_safeCall(&this->controlSteeringWheelData_, this->dataLock_);
            response->frame_id = this->frameID_.load();
            this->frameID_++;
            this->_publishSteeringWheelMsg();
            response->response = true;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlSteeringWheelReqCbFunc] %s", e.what());
            response->response = false;
        }
        catch(...)
        {
            RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlSteeringWheelReqCbFunc] Caught unknown exception.");
            response->response = false;
        }
    }

    // ControlServerController set to ControlChassisReg client. Called by Timer.
    void _controlChassisRegCbFunc()
    {
        auto request = std::make_shared<vehicle_interfaces::srv::ControlChassisReg::Request>();
        request->value = this->_safeCall(&this->controlChassisData_, this->dataLock_);
        request->frame_id = this->frameID_.load();
        this->frameID_++;
        this->_publishChassisMsg();
        auto result = this->controlChassisRegClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (!response->response)
            {
                RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlChassisRegCbFunc] Request failed.");
            }
        }
    }

    // ControlServerController set to ControlSteeringWheelReg client. Called by Timer.
    void _controlSteeringWheelRegCbFunc()
    {
        auto request = std::make_shared<vehicle_interfaces::srv::ControlSteeringWheelReg::Request>();
        request->value = this->_safeCall(&this->controlSteeringWheelData_, this->dataLock_);
        request->frame_id = this->frameID_.load();
        this->frameID_++;
        this->_publishSteeringWheelMsg();
        auto result = this->controlSteeringWheelRegClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (!response->response)
            {
                RCLCPP_ERROR(this->serviceNode_->get_logger(), "[Controller::_controlSteeringWheelRegCbFunc] Request failed.");
            }
        }
    }

public:
    Controller(const std::shared_ptr<vehicle_interfaces::GenericParams>& params, vehicle_interfaces::msg::ControllerInfo info) : 
        vehicle_interfaces::PseudoTimeSyncNode(info.service_name + "_controller"), 
        vehicle_interfaces::QoSUpdateNode(info.service_name + "_controller", params->qosService, params->qosDirPath), 
        rclcpp::Node(info.service_name + "_controller"), 
        info_(info), 
        exec_(nullptr), 
        th_(nullptr), 
        enableF_(false), 
        chassisRegTm_(nullptr), 
        steeringWheelRegTm_(nullptr), 
        pubF_(false), 
        frameID_(0)
    {
        this->nodeName_ = info.service_name + "_controller";
        this->serviceTimeout_ = std::chrono::duration<float, std::milli>(info.timeout_ms);
        this->pubF_ = (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER) || 
                        (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH);
        this->_initData();

        // Controller set to client, ControlServerController set to server
        if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_CLIENT)
        {
            this->serviceNodeName_ = this->nodeName_ + "_client";
            this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
            if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
            {
                this->controlChassisRegClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlChassisReg>(info.service_name);
                this->chassisRegTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&Controller::_controlChassisRegCbFunc, this));
                if (this->pubF_)
                    this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
                this->chassisRegTm_->start();
            }
            else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
            {
                this->controlSteeringWheelRegClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlSteeringWheelReg>(info.service_name);
                this->steeringWheelRegTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&Controller::_controlSteeringWheelRegCbFunc, this));
                if (this->pubF_)
                    this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
                this->steeringWheelRegTm_->start();
            }
            else
            {
                throw "[Controller] construct info.msg_type error.";
            }
        }
        // Controller set to server, ControlServerController set to client.
        else if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_SERVER)
        {
            this->serviceNodeName_ = this->nodeName_ + "_server";
            this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
            if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
            {
                this->controlChassisReqServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlChassisReq>(info.service_name, 
                    std::bind(&Controller::_controlChassisReqCbFunc, this, std::placeholders::_1, std::placeholders::_2));
                if (this->pubF_)
                    this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
            }
            else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
            {
                this->controlSteeringWheelReqServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlSteeringWheelReq>(info.service_name, 
                    std::bind(&Controller::_controlSteeringWheelReqCbFunc, this, std::placeholders::_1, std::placeholders::_2));
                if (this->pubF_)
                    this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
            }
            else
            {
                throw "[Controller] construct info.msg_type error.";
            }
        }
        else
        {
            throw "[Controller] construct info.service_mode error.";
        }
        // TODO: Need to be tested.
        // this->exec_ = new rclcpp::executors::SingleThreadedExecutor();
        // this->exec_->add_node(this->serviceNode_);
        // this->th_ = new std::thread(vehicle_interfaces::SpinExecutor, this->exec_, this->serviceNodeName_, 1000);
        this->enableF_ = true;
        RCLCPP_INFO(this->get_logger(), "[Controller] Controller constructed: %s", this->serviceNodeName_.c_str());
        if (this->pubF_)
            RCLCPP_INFO(this->get_logger(), "[Controller] Create publisher: %s", this->serviceNodeName_.c_str());
    }

    ~Controller()
    {
        if (this->exec_ != nullptr)
            this->exec_->cancel();
        if (this->th_ != nullptr)
            this->th_->join();
        if (this->chassisRegTm_ != nullptr)
        {
            this->chassisRegTm_->destroy();
            delete this->chassisRegTm_;
        }
        if (this->steeringWheelRegTm_ != nullptr)
        {
            this->steeringWheelRegTm_->destroy();
            delete this->steeringWheelRegTm_;
        }
    }

    void setControlChassisData(const vehicle_interfaces::msg::ControlChassis& msg)
    {
        this->_safeSave(&this->controlChassisData_, msg, this->dataLock_);
    }

    void setControlSteeringWheelData(const vehicle_interfaces::msg::ControlSteeringWheel& msg)
    {
        this->_safeSave(&this->controlSteeringWheelData_, msg, this->dataLock_);
    }
};

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
class ControlServerController : public vehicle_interfaces::PseudoTimeSyncNode, public vehicle_interfaces::QoSUpdateNode
{
private:
    std::string nodeName_;
    const vehicle_interfaces::msg::ControllerInfo info_;
    std::mutex infoLock_;
    rclcpp::executors::SingleThreadedExecutor *exec_;
    std::thread *th_;
    std::atomic<bool> enableF_;

    // ControlServerController set to server, receive controller signal from Controller.
    std::string serviceNodeName_;
    std::shared_ptr<rclcpp::Node> serviceNode_;
    rclcpp::Service<vehicle_interfaces::srv::ControlChassisReg>::SharedPtr controlChassisRegServer_;
    rclcpp::Service<vehicle_interfaces::srv::ControlSteeringWheelReg>::SharedPtr controlSteeringWheelRegServer_;
    std::mutex serverCbLock_;

    // ControlServerController set to client, send request controller signal to Controller.
    rclcpp::Client<vehicle_interfaces::srv::ControlChassisReq>::SharedPtr controlChassisReqClient_;
    rclcpp::Client<vehicle_interfaces::srv::ControlSteeringWheelReq>::SharedPtr controlSteeringWheelReqClient_;
    vehicle_interfaces::Timer* chassisReqTm_;
    vehicle_interfaces::Timer* steeringWheelReqTm_;

    // Publisher
    rclcpp::Publisher<vehicle_interfaces::msg::Chassis>::SharedPtr chassisPub_;
    rclcpp::Publisher<vehicle_interfaces::msg::SteeringWheel>::SharedPtr steeringWheelPub_;
    std::atomic<bool> pubF_;

    std::function<bool(const vehicle_interfaces::msg::ControlSteeringWheel&, vehicle_interfaces::msg::ControlChassis&)> cvtFunc_;
    std::atomic<bool> cvtFuncF_;

    vehicle_interfaces::msg::ControlChassis controlChassisData_;
    vehicle_interfaces::msg::ControlSteeringWheel controlSteeringWheelData_;
    std::atomic<u_int64_t> frameID_;
    std::chrono::high_resolution_clock::time_point dataTs_;
    std::mutex dataLock_;

    std::chrono::duration<float, std::milli> serviceTimeout_;

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

    void _initData()
    {
        // Init controlChassisData_.
        this->controlChassisData_.drive_motor = { 0, 0, 0, 0 };
        this->controlChassisData_.steering_motor = { 0, 0, 0, 0 };
        this->controlChassisData_.brake_motor = { 0, 0, 0, 0 };
        this->controlChassisData_.parking_signal = { true, true, true, true };

        // Init controlSteeringWheelData_.
        // All init by default.
    }

    bool _publishChassisMsg()
    {
        if (!this->pubF_)
            return false;
        auto data = _safeCall(&this->controlChassisData_, this->dataLock_);
        auto info = _safeCall(&this->info_, this->infoLock_);
        if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
        {
            auto msg = vehicle_interfaces::msg::Chassis();
            msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
            msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
            msg.header.device_id = this->nodeName_;
            msg.header.frame_id = this->frameID_.load();
            msg.header.stamp_type = this->getTimestampType();
            msg.header.stamp = this->getTimestamp();
            msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
            msg.header.ref_publish_time_ms = info.period_ms;

            msg.unit_type = data.unit_type;
            msg.drive_motor = data.drive_motor;
            msg.steering_motor = data.steering_motor;
            msg.brake_motor = data.brake_motor;
            msg.parking_signal = data.parking_signal;
            msg.controller_name = this->nodeName_;
            this->chassisPub_->publish(msg);
        }
    }

    bool _publishSteeringWheelMsg()
    {
        if (!this->pubF_)
            return false;
        auto data = _safeCall(&this->controlSteeringWheelData_, this->dataLock_);
        auto info = _safeCall(&this->info_, this->infoLock_);
        if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
        {
            auto msg = vehicle_interfaces::msg::SteeringWheel();
            msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
            msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_STEERINGWHEEL;
            msg.header.device_id = this->nodeName_;
            msg.header.frame_id = this->frameID_.load();
            msg.header.stamp_type = this->getTimestampType();
            msg.header.stamp = this->getTimestamp();
            msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
            msg.header.ref_publish_time_ms = info.period_ms;

            msg.gear = data.gear;
            msg.steering = data.steering;
            msg.pedal_throttle = data.pedal_throttle;
            msg.pedal_brake = data.pedal_brake;
            msg.pedal_clutch = data.pedal_clutch;
            msg.func_0 = data.func_0;
            msg.func_1 = data.func_1;
            msg.func_2 = data.func_2;
            msg.func_3 = data.func_3;
            msg.controller_name = this->nodeName_;
            this->steeringWheelPub_->publish(msg);
        }
    }

    // ControlServerController set to ControlChassisReg server.
    void _controlChassisRegCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlChassisReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControlChassisReg::Response> response)
    {
        std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
        this->dataTs_ = std::chrono::high_resolution_clock::now();
        this->_safeSave(&this->controlChassisData_, request->value, this->dataLock_);
        this->frameID_ = request->frame_id;
        this->_publishChassisMsg();
    }

    // ControlServerController set to ControlSteeringWheelReg server.
    void _controlSteeringWheelRegCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControlSteeringWheelReg::Response> response)
    {
        std::lock_guard<std::mutex> serverCbLocker(this->serverCbLock_);
        if (!this->cvtFuncF_.load())
        {
            response->response = false;
            return;
        }
        // Publish SteeringWheel
        this->dataTs_ = std::chrono::high_resolution_clock::now();
        this->_safeSave(&this->controlSteeringWheelData_, request->value, this->dataLock_);
        this->frameID_ = request->frame_id;
        this->_publishSteeringWheelMsg();
        // Convert to ControlChassis
        vehicle_interfaces::msg::ControlChassis tmp;
        if (!this->cvtFunc_(request->value, tmp))
        {
            response->response = false;
            return;
        }
        this->_safeSave(&this->controlChassisData_, tmp, this->dataLock_);
    }

    // ControlServerController set to ControlChassisReq client. Called by Timer.
    void _controlChassisReqCbFunc()
    {
        auto request = std::make_shared<vehicle_interfaces::srv::ControlChassisReq::Request>();
        request->request = true;
        auto result = this->controlChassisReqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
            {
                this->_safeSave(&this->controlChassisData_, response->value, this->dataLock_);
                this->frameID_ = response->frame_id;
                this->_publishChassisMsg();
            }
        }
    }

    // ControlServerController set to ControlSteeringWheelReq client. Called by Timer.
    void _controlSteeringWheelReqCbFunc()
    {
        auto request = std::make_shared<vehicle_interfaces::srv::ControlSteeringWheelReq::Request>();
        request->request = true;
        auto result = this->controlSteeringWheelReqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->serviceNode_, result, this->serviceTimeout_) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
            {
                if (!this->cvtFuncF_.load())
                    return;
                // Publish SteeringWheel
                this->dataTs_ = std::chrono::high_resolution_clock::now();
                this->_safeSave(&this->controlSteeringWheelData_, response->value, this->dataLock_);
                this->frameID_ = response->frame_id;
                this->_publishSteeringWheelMsg();
                // Convert to ControlChassis
                vehicle_interfaces::msg::ControlChassis tmp;
                if (!this->cvtFunc_(response->value, tmp))
                    return;
                this->_safeSave(&this->controlChassisData_, tmp, this->dataLock_);
            }
        }
    }

public:
    ControlServerController(const std::shared_ptr<vehicle_interfaces::GenericParams>& params, const vehicle_interfaces::msg::ControllerInfo& info) : 
        vehicle_interfaces::PseudoTimeSyncNode(info.service_name + "_controlserver"), 
        vehicle_interfaces::QoSUpdateNode(info.service_name + "_controlserver", params->qosService, params->qosDirPath), 
        rclcpp::Node(info.service_name + "_controlserver"), 
        info_(info), 
        exec_(nullptr), 
        th_(nullptr), 
        enableF_(false), 
        chassisReqTm_(nullptr), 
        steeringWheelReqTm_(nullptr), 
        pubF_(false), 
        frameID_(0), 
        cvtFuncF_(false)
    {
        this->nodeName_ = info.service_name + "_controlserver";
        this->serviceTimeout_ = std::chrono::duration<float, std::milli>(info.timeout_ms);
        this->pubF_ = (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLSERVER) || 
                        (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH);
        this->_initData();

        // Controller set to client, ControlServerController set to server
        if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_CLIENT)
        {
            this->serviceNodeName_ = this->nodeName_ + "_server";
            this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
            if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
            {
                this->controlChassisRegServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlChassisReg>(info.service_name, 
                    std::bind(&ControlServerController::_controlChassisRegCbFunc, this, std::placeholders::_1, std::placeholders::_2));
                if (this->pubF_)
                    this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
            }
            else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
            {
                this->controlSteeringWheelRegServer_ = this->serviceNode_->create_service<vehicle_interfaces::srv::ControlSteeringWheelReg>(info.service_name, 
                    std::bind(&ControlServerController::_controlSteeringWheelRegCbFunc, this, std::placeholders::_1, std::placeholders::_2));
                if (this->pubF_)
                    this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
            }
            else
            {
                throw "[ControlServerController] construct info.msg_type error.";
            }
        }
        // Controller set to server, ControlServerController set to client.
        else if (info.service_mode == vehicle_interfaces::msg::ControllerInfo::SERVICE_MODE_SERVER)
        {
            this->serviceNodeName_ = this->nodeName_ + "_client";
            this->serviceNode_ = rclcpp::Node::make_shared(this->serviceNodeName_);
            if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
            {
                this->controlChassisReqClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlChassisReq>(info.service_name);
                this->chassisReqTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&ControlServerController::_controlChassisReqCbFunc, this));
                if (this->pubF_)
                    this->chassisPub_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(info.service_name, 10);
                this->chassisReqTm_->start();
            }
            else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
            {
                this->controlSteeringWheelReqClient_ = this->serviceNode_->create_client<vehicle_interfaces::srv::ControlSteeringWheelReq>(info.service_name);
                this->steeringWheelReqTm_ = new vehicle_interfaces::Timer(info.period_ms, std::bind(&ControlServerController::_controlSteeringWheelReqCbFunc, this));
                if (this->pubF_)
                    this->steeringWheelPub_ = this->create_publisher<vehicle_interfaces::msg::SteeringWheel>(info.service_name, 10);
                this->steeringWheelReqTm_->start();
            }
            else
            {
                throw "[ControlServerController] construct info.msg_type error.";
            }
        }
        else
        {
            throw "[ControlServerController] construct info.service_mode error.";
        }
        // TODO: Need to be tested.
        // this->exec_ = new rclcpp::executors::SingleThreadedExecutor();
        // this->exec_->add_node(this->serviceNode_);
        // this->th_ = new std::thread(vehicle_interfaces::SpinExecutor, this->exec_, this->serviceNodeName_, 1000);
        this->enableF_ = true;
        RCLCPP_INFO(this->get_logger(), "[ControlServerController] Controller constructed: %s", this->serviceNodeName_.c_str());
        if (this->pubF_)
            RCLCPP_INFO(this->get_logger(), "[ControlServerController] Create publisher: %s", this->serviceNodeName_.c_str());
    }

    ~ControlServerController()
    {
        if (this->exec_ != nullptr)
            this->exec_->cancel();
        if (this->th_ != nullptr)
            this->th_->join();
        if (this->chassisReqTm_ != nullptr)
        {
            this->chassisReqTm_->destroy();
            delete this->chassisReqTm_;
        }
        if (this->steeringWheelReqTm_ != nullptr)
        {
            this->steeringWheelReqTm_->destroy();
            delete this->steeringWheelReqTm_;
        }
    }

    void setCvtFunc(const std::function<bool(const vehicle_interfaces::msg::ControlSteeringWheel&, vehicle_interfaces::msg::ControlChassis&)>& cvtFunc)
    {
        this->cvtFunc_ = cvtFunc;
        this->cvtFuncF_ = true;
    }

    vehicle_interfaces::msg::ControllerInfo getInfo()
    {
        std::lock_guard<std::mutex> locker(this->infoLock_);
        return this->info_;
    }

    bool getSignal(vehicle_interfaces::msg::ControlChassis& outSignal, std::chrono::high_resolution_clock::time_point& outTimestamp)
    {
        if (!this->enableF_)
            return false;
        std::lock_guard<std::mutex> locker(this->dataLock_);
        outSignal = this->controlChassisData_;
        outTimestamp = this->dataTs_;
    }
};

}