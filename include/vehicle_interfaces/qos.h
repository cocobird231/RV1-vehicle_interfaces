#pragma once
#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <vector>
#include <map>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/timesync.h"
#include "vehicle_interfaces/msg/qos_profile.hpp"
#include "vehicle_interfaces/msg/qos_update.hpp"
#include "vehicle_interfaces/srv/qos_reg.hpp"
#include "vehicle_interfaces/srv/qos_req.hpp"

#include <fstream>
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace vehicle_interfaces
{

typedef std::map<std::string, rclcpp::QoS*> QoSMap;

// double ms to rmw_time_s
inline rmw_time_s CvtMsgToRMWTime(const double& time_ms)
{
    return { time_ms / 1000, (time_ms - (uint64_t)time_ms) * 1000000 };
}

// rmw_time_s to double ms
inline double CvtRMWTimeToMsg(const rmw_time_s& rmwT)
{
    return rmwT.sec * 1000 + rmwT.nsec / 1000000.0;
}

// Return 1 if rmwT1 > rmwT2; -1 if rmwT1 < rmwT2; 0 if rmwT1 = rmwT2
inline int CompRMWTime(const rmw_time_s& rmwT1, const rmw_time_s& rmwT2)
{
    uint64_t t1 = static_cast<uint64_t>(rmwT1.sec * 1000000000) + rmwT1.nsec;
    uint64_t t2 = static_cast<uint64_t>(rmwT2.sec * 1000000000) + rmwT2.nsec;
    uint64_t ret = t1 - t2;
    return ret > 0 ? 1 : (ret < 0 ? -1 : 0);
}

vehicle_interfaces::msg::QosProfile CvtRMWQoSToMsg(const rmw_qos_profile_t& prof)
{
    vehicle_interfaces::msg::QosProfile ret;
    ret.history = prof.history;
    ret.depth = prof.depth;
    ret.reliability = prof.reliability;
    ret.durability = prof.durability;
    ret.deadline_ms = CvtRMWTimeToMsg(prof.deadline);
    ret.lifespan_ms = CvtRMWTimeToMsg(prof.lifespan);
    ret.liveliness = prof.liveliness;
    ret.liveliness_lease_duration_ms = CvtRMWTimeToMsg(prof.liveliness_lease_duration);
    return ret;
}

rmw_qos_profile_t CvtMsgToRMWQoS(const vehicle_interfaces::msg::QosProfile& msg)
{
    rmw_qos_profile_t prof;
    prof.history = (rmw_qos_history_policy_e)msg.history;
    prof.depth = msg.depth;
    prof.reliability = (rmw_qos_reliability_policy_e)msg.reliability;
    prof.durability = (rmw_qos_durability_policy_e)msg.durability;
    prof.deadline = CvtMsgToRMWTime(msg.deadline_ms);
    prof.lifespan = CvtMsgToRMWTime(msg.lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_e)msg.liveliness;
    prof.liveliness_lease_duration = CvtMsgToRMWTime(msg.liveliness_lease_duration_ms);
    return prof;
}

std::string getQoSProfEnumName(rmw_qos_reliability_policy_e value)
{
    switch (value)
    {
        case rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
            return "BEST_EFFORT";
            break;
        case rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            return "RELIABLE";
            break;
        case rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
            return "SYSTEM_DEFAULT";
            break;
        case rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
            return "UNKNOWN";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

bool DumpRMWQoSToJSON(const fs::path& qosFilePath, const rmw_qos_profile_t& profile)
{
    try
    {
        nlohmann::json json;
        json["history"] = (int8_t)profile.history;
        json["depth"] = profile.depth;
        json["reliability"] = (int8_t)profile.reliability;
        json["durability"] = (int8_t)profile.durability;
        json["deadline_ms"] = CvtRMWTimeToMsg(profile.deadline);
        json["lifespan_ms"] = CvtRMWTimeToMsg(profile.lifespan);
        json["liveliness"] = (int8_t)profile.liveliness;
        json["liveliness_lease_duration_ms"] = CvtRMWTimeToMsg(profile.liveliness_lease_duration);
        std::ofstream outFile(qosFilePath);
        outFile << json;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool LoadRMWQoSFromJSON(const fs::path& qosFilePath, rmw_qos_profile_t& profile)
{
    try
    {
        nlohmann::json json;
        json.update(nlohmann::json::parse(std::ifstream(qosFilePath)));
        profile.history = (rmw_qos_history_policy_e)json["history"];
        profile.depth = json["depth"];
        profile.reliability = (rmw_qos_reliability_policy_e)json["reliability"];
        profile.durability = (rmw_qos_durability_policy_e)json["durability"];
        profile.deadline = CvtMsgToRMWTime(json["deadline_ms"]);
        profile.lifespan = CvtMsgToRMWTime(json["lifespan_ms"]);
        profile.liveliness = (rmw_qos_liveliness_policy_e)json["liveliness"];
        profile.liveliness_lease_duration = CvtMsgToRMWTime(json["liveliness_lease_duration_ms"]);
        return true;
    }
    catch(...)
    {
        return false;
    }
}

/* The QoSUpdateNode class implements the QoS update mechanisms, including a subscription of QoS update topic and a client for QoS update service.
 * The publish or subscription node can easily inherit the QoSUpdateNode, and adding callback function with addQoSCallbackFunc() to get callback 
 * while QoS policy updated. 
 * The QoS update mechanisms relying on the name of topics, so it's necessary to add topic names to QoSUpdateNode by calling addQoSTracking() to 
 * tracks whether the QoS of specific topic needs to be update.
 */
class QoSUpdateNode : virtual public rclcpp::Node
{
private:
    // QoS Service Definition
    std::shared_ptr<rclcpp::Node> reqClientNode_;// QOS service node
    rclcpp::Client<vehicle_interfaces::srv::QosReq>::SharedPtr reqClient_;// QoS service client

    // QoS Topic Definition
    rclcpp::Subscription<vehicle_interfaces::msg::QosUpdate>::SharedPtr subscription_;// QoS subscription
    std::atomic<uint64_t> qosID_;// Unique ID that describes current QoS' profile
    std::vector<std::string> qosTopicNameVec_;// Topics QoS need to be tracked
    std::mutex subscriptionLock_;// Prevent QoS callback conflict

    // QoS Changed Callback
    std::atomic<bool> callbackF_;
    std::function<void(QoSMap)> qosCallbackFunc_;

    // QoS file path
    fs::path qosDirPath_;

    // Node enable
    std::atomic<bool> nodeEnableF_;

private:
    void _topic_callback(const vehicle_interfaces::msg::QosUpdate::SharedPtr msg)
    {
        if (!this->callbackF_)// No callback function assigned
            return;
        
        if (msg->qid == this->qosID_)// Ignore update in same qos ID
            return;

        RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::_topic_callback] qid: %d", msg->qid);
        
        std::vector<std::string> topicVec;

        std::unique_lock<std::mutex> locker(this->subscriptionLock_, std::defer_lock);
        locker.lock();
        for (auto& myTopic : this->qosTopicNameVec_)
        {
            for (auto& newTopic : msg->topic_table)
            {
                if (myTopic == newTopic)
                {
                    topicVec.push_back(myTopic);
                    break;
                }
            }
        }
        locker.unlock();
        
        QoSMap qmap;
        for (const auto& topic : topicVec)
        {
            try
            {
                qmap[topic] = this->requestQoS(topic);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "[QoSUpdateNode::requestQoS] Request error: %s", topic.c_str());
            }
        }
        
        if (qmap.size() > 0)
        {
            this->qosCallbackFunc_(qmap);

            locker.lock();
            {// Dump QoS profile
                for (const auto& [k, v] : qmap)
                {
                    std::string tn = k;
                    vehicle_interfaces::replace_all(tn, "/", "_");
                    DumpRMWQoSToJSON(this->qosDirPath_ / (tn + ".json"), v->get_rmw_qos_profile());
                    RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::_topic_callback] Dump QoS profile: %s", 
                        (this->qosDirPath_ / (tn + ".json")).generic_string().c_str());
                }
            }
            locker.unlock();
        }
        
        this->qosID_ = msg->qid;
    }

public:
    QoSUpdateNode(const std::string& nodeName, const std::string& qosServiceName, const std::string& qosDirPath) : rclcpp::Node(nodeName), 
        qosDirPath_(qosDirPath), 
        qosID_(0), 
        callbackF_(false), 
        nodeEnableF_(false)
    {
        if (qosServiceName == "")
            return;
        
        {// Check QoS directory
            char buf[512];
            sprintf(buf, "mkdir -p %s", qosDirPath_.generic_string().c_str());
            system(buf);
        }

        this->reqClientNode_ = rclcpp::Node::make_shared(nodeName + "_qosreq_client");
        this->reqClient_ = this->reqClientNode_->create_client<vehicle_interfaces::srv::QosReq>(qosServiceName + "_Req");

        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::QosUpdate>(qosServiceName, 
            10, std::bind(&QoSUpdateNode::_topic_callback, this, std::placeholders::_1));
        this->nodeEnableF_ = true;
        RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode] Constructed.");
    }

    void addQoSTracking(std::string topicName, rclcpp::QoS* outQoS)
    {
        if (!this->nodeEnableF_)
            return;
        
        if (topicName.length() <= 0)
            return;

        if (strlen(this->get_namespace()) > 0)// Namespace exists
        {
            if (topicName.find(this->get_namespace()) == std::string::npos)
            {
                if (topicName[0] == '/')
                    topicName = std::string(this->get_namespace()) + topicName;
                else
                    topicName = std::string(this->get_namespace()) + "/" + topicName;
            }
        }

        std::lock_guard<std::mutex> locker(this->subscriptionLock_);

        for (auto& i : this->qosTopicNameVec_)
            if (i == topicName)
                return;

        {// Load QoS profile
            // Topic name trans
            std::string tn = topicName;
            vehicle_interfaces::replace_all(tn, "/", "_");

            rmw_qos_profile_t prof;
            if (LoadRMWQoSFromJSON(this->qosDirPath_ / (tn + ".json"), prof))
            {
                outQoS = new rclcpp::QoS(rclcpp::QoSInitialization(prof.history, prof.depth), prof);
                RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::addQoSTracking] Found QoS profile: depth: %d reliability: %d durability: %d", 
                    prof.depth, prof.reliability, prof.durability);
            }
            else
            {
                outQoS = new rclcpp::QoS(10);
                RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::addQoSTracking] QoS profile not found. Set to default.");
            }
        }
        
        this->qosTopicNameVec_.push_back(topicName);
    }

    void addQoSCallbackFunc(const std::function<void(QoSMap)>& func)
    {
        if (!this->nodeEnableF_)
            return;

        std::lock_guard<std::mutex> locker(this->subscriptionLock_);
        this->qosCallbackFunc_ = func;
        this->callbackF_ = true;
    }

    rclcpp::QoS* requestQoS(std::string topicName)
    {
        if (!this->nodeEnableF_)
            throw "Request QoS Failed";// Request QoS failed
        RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::requestQoS] Request %s QoS profile...", topicName.c_str());

        auto request = std::make_shared<vehicle_interfaces::srv::QosReq::Request>();
        request->topic_name = topicName;
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 100ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 100ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::requestQoS] Response: %d, qid: %ld", res->response, res->qid);
            if (res->response)
            {
                rmw_qos_profile_t prof = CvtMsgToRMWQoS(res->qos_profile);
                RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::requestQoS] Profile get: %s, %d", getQoSProfEnumName(prof.reliability).c_str(), prof.depth);
                return new rclcpp::QoS(rclcpp::QoSInitialization(prof.history, prof.depth), prof);
            }
        }
        RCLCPP_ERROR(this->get_logger(), "[QoSUpdateNode::requestQoS] Request %s QoS profile failed.", topicName.c_str());
        throw "Request QoS Failed";// Request QoS failed
    }

    rclcpp::QoS* getLatestQoS()
    {
        // TODO
        throw "Unsupported.";
    }
};



class QoSServer : public rclcpp::Node
{
private:
    rclcpp::Service<vehicle_interfaces::srv::QosReg>::SharedPtr regServer_;
    rclcpp::Service<vehicle_interfaces::srv::QosReq>::SharedPtr reqServer_;
    rclcpp::Publisher<vehicle_interfaces::msg::QosUpdate>::SharedPtr pub_;
    Timer* pubTimer_;

    std::string nodeName_;

    std::map<std::string, rmw_qos_profile_t> qmap_;// QoS map
    std::map<std::string, rmw_qos_profile_t> qmapTmp_;// Tmp QoS map
    std::vector<std::string> updateList_;// Publish update signal
    std::mutex qmapLock_;

    std::atomic<uint64_t> qid_;

    // Controllable parameters
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr paramsCallbackHandler;
    bool enablePubF_;
    double pubInterval_ms_;
    std::mutex paramsLock_;

private:
    void _getParams()
    {
        std::lock_guard<std::mutex> locker(this->paramsLock_);
        this->get_parameter("enabled_publish", this->enablePubF_);
        this->get_parameter("publish_interval_ms", this->pubInterval_ms_);
    }

    rcl_interfaces::msg::SetParametersResult _paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_paramsCallback]");
        std::unique_lock<std::mutex> locker(this->paramsLock_, std::defer_lock);

        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = "";

        for (const auto& param : params)
        {
            try
            {
                if (param.get_name() == "enabled_publish")
                {
                    this->enablePubF_ = param.as_bool();
                    ReasonResult<bool> ret;
                    if (this->enablePubF_)
                        ret = this->startPublish();
                    else
                        ret = this->stopPublish();
                    res.successful = ret.result;
                    res.reason = ret.reason;
                }
                else if (param.get_name() == "publish_interval_ms")
                {
                    locker.lock();
                    this->pubInterval_ms_ = param.as_double();
                    this->pubTimer_->setInterval(this->pubInterval_ms_);
                    locker.unlock();
                    res.successful = true;
                }
            }
            catch (...)
            {
                res.successful = false;
                res.reason = "[QoSServer::_paramsCallback] Caught Unknown Exception!";
            }
        }
        return res;
    }

    void _regServiceCallback(const std::shared_ptr<vehicle_interfaces::srv::QosReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::QosReg::Response> response)
    {
        if (request->save_qmap)
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] Save qmap request");
            this->setQmap();
        }
        else if (request->clear_profiles)
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] Clear tmp qmap");
            this->clearTmpQoSProfile();
        }
        else if (request->remove_profile)
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] request: remove %s [%d]", 
                request->topic_name.c_str(), request->dev_type);
            this->removeTmpQoSProfile(request->topic_name);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] request: set %s [%d]", 
                request->topic_name.c_str(), request->dev_type);
            this->setTmpQoSProfile(request->topic_name, CvtMsgToRMWQoS(request->qos_profile));
        }
        response->response = true;
        response->qid = this->qid_.load();
        return;
    }

    void _reqServiceCallback(const std::shared_ptr<vehicle_interfaces::srv::QosReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::QosReq::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_reqServiceCallback] request: %s [%d]", request->topic_name.c_str(), request->dev_type);

        // Prepare qmap lock
        std::unique_lock<std::mutex> locker(this->qmapLock_, std::defer_lock);

        locker.lock();
        auto qmapTmp = this->qmap_;
        locker.unlock();

        // Check topic name
        if (qmapTmp.find(request->topic_name) == qmapTmp.end())
        {
            // Topic name not listed in qmap
            rclcpp::QoS* ret = new rclcpp::QoS(10);
            response->qos_profile = CvtRMWQoSToMsg(ret->get_rmw_qos_profile());
            response->response = false;
            response->qid = this->qid_.load();
        }
        else
        {
            // Found topic name
            response->qos_profile = CvtRMWQoSToMsg(qmapTmp[request->topic_name]);
            response->response = true;
            response->qid = this->qid_.load();
        }

        // Logger
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_reqServiceCallback] response: qid: %04d %s (found: %s)", 
            response->qid, request->topic_name.c_str(), response->response ? "true" : "false");
    }

    // Publish QoS update signal
    void _timerCallback()
    {
        // TODO
        std::unique_lock<std::mutex> qmapLocker(this->qmapLock_, std::defer_lock);
        std::unique_lock<std::mutex> paramsLocker(this->paramsLock_, std::defer_lock);

        static uint64_t frame_id = 0;
        auto msg = vehicle_interfaces::msg::QosUpdate();

        paramsLocker.lock();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_INFO;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_NONE;
        msg.header.device_id = this->nodeName_;
        msg.header.frame_id = frame_id++;
        msg.header.stamp_type = vehicle_interfaces::msg::Header::STAMPTYPE_NO_SYNC;
        msg.header.stamp = this->get_clock()->now();
        msg.header.stamp_offset = 0;
        msg.header.ref_publish_time_ms = this->pubInterval_ms_;
        paramsLocker.unlock();

        qmapLocker.lock();
        msg.qid = this->qid_.load();
        msg.topic_table = this->updateList_;
        qmapLocker.unlock();

        this->pub_->publish(msg);
    }

    // Return indicator map describes which topic_name profile should be updated
    std::vector<std::string> _qmapUpdateCheck()
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);

        std::vector<std::string> updateVec;
        for (auto& [k, v] : this->qmapTmp_)
        {
            try
            {
                bool compF = v.history == this->qmap_[k].history && 
                            v.depth == this->qmap_[k].depth && 
                            v.reliability == this->qmap_[k].reliability && 
                            v.durability == this->qmap_[k].durability && 
                            0 == CompRMWTime(v.deadline, this->qmap_[k].deadline) &&
                            0 == CompRMWTime(v.lifespan, this->qmap_[k].lifespan) &&
                            v.liveliness == this->qmap_[k].liveliness && 
                            0 == CompRMWTime(v.liveliness_lease_duration, this->qmap_[k].liveliness_lease_duration);
                if (!compF)
                    updateVec.push_back(k);
            }
            catch (...)
            {
                updateVec.push_back(k);
            }
        }
        return updateVec;
    }

public:
    QoSServer(const std::string& nodeName, const std::string& serviceName) : rclcpp::Node(nodeName), 
        qid_(0), enablePubF_(false), pubInterval_ms_(10000), nodeName_(nodeName)
    {
        this->declare_parameter<bool>("enabled_publish", this->enablePubF_);
        this->declare_parameter<double>("publish_interval_ms", this->pubInterval_ms_);
        this->_getParams();

        this->regServer_ = this->create_service<vehicle_interfaces::srv::QosReg>(serviceName + "_Reg", 
            std::bind(&QoSServer::_regServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        this->reqServer_ = this->create_service<vehicle_interfaces::srv::QosReq>(serviceName + "_Req", 
            std::bind(&QoSServer::_reqServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        this->pub_ = this->create_publisher<vehicle_interfaces::msg::QosUpdate>(serviceName, 10);
        this->pubTimer_ = new Timer(this->pubInterval_ms_, std::bind(&QoSServer::_timerCallback, this));

        this->paramsCallbackHandler = this->add_on_set_parameters_callback(std::bind(&QoSServer::_paramsCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[QoSServer] Constructed.");
    }

    ReasonResult<bool> startPublish()
    {
        if (this->pubTimer_ != nullptr)
        {
            this->pubTimer_->start();
            RCLCPP_INFO(this->get_logger(), "[QoSServer::startPublish] Start publish timer");
            return true;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::startPublish] Publish timer not set");
        return {false, "[QoSServer::startPublish] Publish timer not set"};
    }

    ReasonResult<bool> stopPublish()
    {
        if (this->pubTimer_ != nullptr)
        {
            this->pubTimer_->stop();
            RCLCPP_INFO(this->get_logger(), "[QoSServer::stopPublish] Publish timer stopped");
            return true;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::stopPublish] Publish timer not set");
        return {false, "[QoSServer::startPublish] Publish timer not set"};
    }

    void setTmpQoSProfile(const std::string& topicName, const rmw_qos_profile_t& prof)
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        this->qmapTmp_[topicName] = prof;
    }

    void removeTmpQoSProfile(const std::string& topicName)
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        this->qmapTmp_.erase(topicName);
    }

    void clearTmpQoSProfile()
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        this->qmapTmp_.clear();
    }

    void setQmap()
    {
        auto updateVec = this->_qmapUpdateCheck();
        if (updateVec.size() > 0)
        {
            std::unique_lock<std::mutex> locker(this->qmapLock_, std::defer_lock);
            locker.lock();
            this->qmap_ = this->qmapTmp_;
            this->qid_ += 1;
            this->updateList_ = updateVec;
            locker.unlock();
        }
    }
};

}// namespace vehicle_interfaces
