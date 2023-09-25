#pragma once
#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <vector>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/qos_profile.hpp"
#include "vehicle_interfaces/msg/qos_update.hpp"
#include "vehicle_interfaces/srv/qos_reg.hpp"
#include "vehicle_interfaces/srv/qos_req.hpp"

#include <fstream>
#include "nlohmann/json.hpp"

#ifndef ROS_DISTRO// 0: eloquent, 1: foxy, 2: humble
#define ROS_DISTRO 2
#endif

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace vehicle_interfaces
{

typedef std::map<std::string, rclcpp::QoS*> QoSMap;
typedef std::pair<std::string, rclcpp::QoS*> QoSPair;

#if ROS_DISTRO < 2// Foxy and older
// double ms to rmw_time_t
inline rmw_time_t CvtMsgToRMWTime(const double& time_ms)
{
    return { time_ms / 1000, (time_ms - (uint64_t)time_ms) * 1000000 };
}

// rmw_time_t to double ms
inline double CvtRMWTimeToMsg(const rmw_time_t& rmwT)
{
    return rmwT.sec * 1000 + rmwT.nsec / 1000000.0;
}

// Return 1 if rmwT1 > rmwT2; -1 if rmwT1 < rmwT2; 0 if rmwT1 = rmwT2
inline int CompRMWTime(const rmw_time_t& rmwT1, const rmw_time_t& rmwT2)
{
    uint64_t t1 = static_cast<uint64_t>(rmwT1.sec * 1000000000) + rmwT1.nsec;
    uint64_t t2 = static_cast<uint64_t>(rmwT2.sec * 1000000000) + rmwT2.nsec;
    uint64_t ret = t1 - t2;
    return ret > 0 ? 1 : (ret < 0 ? -1 : 0);
}
#else// Humble and newer
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
#endif



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

#if ROS_DISTRO < 2// Foxy and older
std::string getQoSProfEnumName(rmw_qos_reliability_policy_t value)
{
    switch (value)
    {
        case rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
            return "BEST_EFFORT";
            break;
        case rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE:
            return "RELIABLE";
            break;
        case rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
            return "SYSTEM_DEFAULT";
            break;
        case rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_UNKNOWN:
            return "UNKNOWN";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

rmw_qos_profile_t CvtMsgToRMWQoS(const vehicle_interfaces::msg::QosProfile& msg)
{
    rmw_qos_profile_t prof = rclcpp::QoS(10).get_rmw_qos_profile();
    prof.history = (rmw_qos_history_policy_t)msg.history;
    prof.depth = msg.depth;
    prof.reliability = (rmw_qos_reliability_policy_t)msg.reliability;
    prof.durability = (rmw_qos_durability_policy_t)msg.durability;
    prof.deadline = CvtMsgToRMWTime(msg.deadline_ms);
    prof.lifespan = CvtMsgToRMWTime(msg.lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_t)msg.liveliness;
    prof.liveliness_lease_duration = CvtMsgToRMWTime(msg.liveliness_lease_duration_ms);
    // printf("history: %d\ndepth: %d\nreliability: %d\ndurability: %d\n \\
    //         deadline:%d,%d\nlifespan: %d,%d\nliveliness: %d\nliveliness_lease_duration: %d,%d\n", 
    //         prof.history, prof.depth, prof.reliability, prof.durability, 
    //         prof.deadline.sec, prof.deadline.nsec, prof.lifespan.sec, prof.lifespan.nsec, 
    //         prof.liveliness, prof.liveliness_lease_duration.sec, prof.liveliness_lease_duration.nsec);
    return prof;
}

bool LoadRMWQoSFromJSON(const fs::path& qosFilePath, rmw_qos_profile_t& profile)
{
    try
    {
        nlohmann::json json;
        json.update(nlohmann::json::parse(std::ifstream(qosFilePath)));
        profile.history = (rmw_qos_history_policy_t)json["history"];
        profile.depth = json["depth"];
        profile.reliability = (rmw_qos_reliability_policy_t)json["reliability"];
        profile.durability = (rmw_qos_durability_policy_t)json["durability"];
        profile.deadline = CvtMsgToRMWTime(json["deadline_ms"]);
        profile.lifespan = CvtMsgToRMWTime(json["lifespan_ms"]);
        profile.liveliness = (rmw_qos_liveliness_policy_t)json["liveliness"];
        profile.liveliness_lease_duration = CvtMsgToRMWTime(json["liveliness_lease_duration_ms"]);
        return true;
    }
    catch(...)
    {
        return false;
    }
}

#else// Humble and newer
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

rmw_qos_profile_t CvtMsgToRMWQoS(const vehicle_interfaces::msg::QosProfile& msg)
{
    rmw_qos_profile_t prof = rclcpp::QoS(10).get_rmw_qos_profile();
    prof.history = (rmw_qos_history_policy_e)msg.history;
    prof.depth = msg.depth;
    prof.reliability = (rmw_qos_reliability_policy_e)msg.reliability;
    prof.durability = (rmw_qos_durability_policy_e)msg.durability;
    prof.deadline = CvtMsgToRMWTime(msg.deadline_ms);
    prof.lifespan = CvtMsgToRMWTime(msg.lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_e)msg.liveliness;
    prof.liveliness_lease_duration = CvtMsgToRMWTime(msg.liveliness_lease_duration_ms);
    // printf("history: %d\ndepth: %d\nreliability: %d\ndurability: %d\n \\
    //         deadline:%d,%d\nlifespan: %d,%d\nliveliness: %d\nliveliness_lease_duration: %d,%d\n", 
    //         prof.history, prof.depth, prof.reliability, prof.durability, 
    //         prof.deadline.sec, prof.deadline.nsec, prof.lifespan.sec, prof.lifespan.nsec, 
    //         prof.liveliness, prof.liveliness_lease_duration.sec, prof.liveliness_lease_duration.nsec);
    return prof;
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
#endif



// Only converts the depth, history, reliability and durability parameters. Rest of the parameters will set to default.
rclcpp::QoS CvtRMWQoSToRclQoS(const rmw_qos_profile_t& prof)
{
    rclcpp::QoS ret(prof.depth);
    ret.history(prof.history);
    ret.reliability(prof.reliability);
    ret.durability(prof.durability);
    // ret.deadline(prof.deadline);
    // ret.lifespan(prof.lifespan);
    // ret.liveliness(prof.liveliness);
    // ret.liveliness_lease_duration(prof.liveliness_lease_duration);
    return ret;
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

inline bool CompRMWQoS(const rmw_qos_profile_t& p1, const rmw_qos_profile_t& p2)
{
    return p1.history == p2.history && 
            p1.depth == p2.depth && 
            p1.reliability == p2.reliability && 
            p1.durability == p2.durability && 
            0 == CompRMWTime(p1.deadline, p2.deadline) &&
            0 == CompRMWTime(p1.lifespan, p2.lifespan) &&
            p1.liveliness == p2.liveliness && 
            0 == CompRMWTime(p1.liveliness_lease_duration, p2.liveliness_lease_duration);
}


using TopicType = DescriptiveValue<uint8_t>;

class TopicProp
{
public:
    const static TopicType PUBLISHER;
    const static TopicType SUBSCRIPTION;
    const static TopicType BOTH;

    const static DescriptiveValue<TopicType> PUBLISHER_PREFIX;
    const static DescriptiveValue<TopicType> SUBSCRIPTION_PREFIX;
    const static DescriptiveValue<TopicType> BOTH_PREFIX;

    enum Exception {FULLNAME_RETRIEVE_ERROR};

    std::string name;
    std::string fullName;
    TopicType type;

private:
    void _setProp()
    {
        if (this->type == TopicProp::PUBLISHER)
            this->fullName = "#P!" + this->name;
        else if (this->type == TopicProp::SUBSCRIPTION)
            this->fullName = "#S!" + this->name;
        else
            this->fullName = "#B!" + this->name;
    }

public:
    TopicProp(std::string topicName) : name(topicName), type(TopicProp::BOTH)
    {
        this->_setProp();
    }

    TopicProp(std::string topicName, TopicType topicType) : name(topicName), type(topicType)
    {
        this->_setProp();
    }

    TopicProp(std::string topicName, std::string topicType) : name(topicName)
    {
        if (topicType == "publisher" || topicType == "pub")
            this->type = TopicProp::PUBLISHER;
        else if (topicType == "subscription" || topicType == "sub")
            this->type = TopicProp::SUBSCRIPTION;
        else
            this->type = TopicProp::BOTH;
        this->_setProp();
    }

    static TopicProp retrieveTopicProp(std::string fullTopicName)
    {
        std::string prefix = fullTopicName.substr(0, 3);
        if (prefix == TopicProp::PUBLISHER_PREFIX.str)
            return {fullTopicName.substr(3), TopicProp::PUBLISHER};
        else if (prefix == TopicProp::SUBSCRIPTION_PREFIX.str)
            return {fullTopicName.substr(3), TopicProp::SUBSCRIPTION};
        else if (prefix == TopicProp::BOTH_PREFIX.str)
            return {fullTopicName.substr(3), TopicProp::BOTH};
        else
            throw TopicProp::Exception::FULLNAME_RETRIEVE_ERROR;
    }
};

const TopicType TopicProp::PUBLISHER = {0, "publisher"};
const TopicType TopicProp::SUBSCRIPTION = {1, "subscription"};
const TopicType TopicProp::BOTH = {2, "both"};

const DescriptiveValue<TopicType> TopicProp::PUBLISHER_PREFIX = {TopicProp::PUBLISHER, "#P!"};
const DescriptiveValue<TopicType> TopicProp::SUBSCRIPTION_PREFIX = {TopicProp::SUBSCRIPTION, "#S!"};
const DescriptiveValue<TopicType> TopicProp::BOTH_PREFIX = {TopicProp::BOTH, "#B!"};

/**
 * The QoSUpdateNode class implements the QoS update mechanisms, including a subscription of QoS update topic and a client for QoS update service.
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
    std::set<std::string> qosTopicNameVec_;// Topics QoS need to be tracked
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

        RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::_topic_callback] Received QoS update signal. (qid: %d)", msg->qid);
        
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
                qmap[topic] = new rclcpp::QoS(std::move(this->requestQoS(topic)));
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "[QoSUpdateNode::_topic_callback] Request QoS failed: %s.", topic.c_str());
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
                    RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::_topic_callback] Dump QoS profile: %s.", 
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
            sprintf(buf, "mkdir -p %s", this->qosDirPath_.generic_string().c_str());
            system(buf);
        }

        this->reqClientNode_ = rclcpp::Node::make_shared(nodeName + "_qosreq_client");
        this->reqClient_ = this->reqClientNode_->create_client<vehicle_interfaces::srv::QosReq>(qosServiceName + "_Req");

        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::QosUpdate>(qosServiceName, 
            10, std::bind(&QoSUpdateNode::_topic_callback, this, std::placeholders::_1));
        this->nodeEnableF_ = true;
        // RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode] Constructed.");
    }

    QoSPair addQoSTracking(std::string topicName)
    {
        if (!this->nodeEnableF_)
            return {"", new rclcpp::QoS(10)};
        
        if (topicName.length() <= 0)
            return {"", new rclcpp::QoS(10)};

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

        std::unique_lock<std::mutex> locker(this->subscriptionLock_, std::defer_lock);
        locker.lock();
        this->qosTopicNameVec_.insert(topicName);
        locker.unlock();

        // Preparing return qmap
        QoSPair ret = {topicName, new rclcpp::QoS(10)};

        // Load QoS profile
        std::string tn = topicName;// Topic name trans
        vehicle_interfaces::replace_all(tn, "/", "_");

        rmw_qos_profile_t prof;
        if (LoadRMWQoSFromJSON(this->qosDirPath_ / (tn + ".json"), prof))
        {
            ret.second = new rclcpp::QoS(CvtRMWQoSToRclQoS(prof));
            // RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::addQoSTracking] Found QoS profile: depth: %d reliability: %d durability: %d.", 
            //     ret.second->get_rmw_qos_profile().depth, ret.second->get_rmw_qos_profile().reliability, ret.second->get_rmw_qos_profile().durability);
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::addQoSTracking] QoS profile not found. Set to default.");
        }
        return ret;
    }

    void addQoSCallbackFunc(const std::function<void(QoSMap)>& func)
    {
        if (!this->nodeEnableF_)
            return;

        std::lock_guard<std::mutex> locker(this->subscriptionLock_);
        this->qosCallbackFunc_ = func;
        this->callbackF_ = true;
    }

    rclcpp::QoS requestQoS(std::string topicName)
    {
        if (!this->nodeEnableF_)
            throw "Request QoS Failed";// Request QoS failed
        // RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::requestQoS] Request %s QoS profile...", topicName.c_str());

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
            if (res->response)
            {
                rmw_qos_profile_t prof = CvtMsgToRMWQoS(res->qos_profile);
                // RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::requestQoS] Profile get: %s, %d.", getQoSProfEnumName(prof.reliability).c_str(), prof.depth);
                return CvtRMWQoSToRclQoS(prof);
            }
        }
        // RCLCPP_ERROR(this->get_logger(), "[QoSUpdateNode::requestQoS] Request %s QoS profile failed.", topicName.c_str());
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

    fs::path recordFilePath_;

private:
    void _getParams()
    {
        std::lock_guard<std::mutex> locker(this->paramsLock_);
        this->get_parameter("enabled_publish", this->enablePubF_);
        this->get_parameter("publish_interval_ms", this->pubInterval_ms_);
    }

    rcl_interfaces::msg::SetParametersResult _paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        // RCLCPP_INFO(this->get_logger(), "[QoSServer::_paramsCallback]");
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
        if (request->topic_name == "" && !request->save_qmap && !request->clear_profiles)
        {
            response->response = false;
            response->qid = this->qid_.load();
            return;
        }

        auto tp = TopicProp(request->topic_name, request->topic_type);

        if (request->save_qmap)
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] Save qmap request.");
            this->setQmap();
        }
        else if (request->clear_profiles)
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] Recover tmp qmap.");
            this->recoverTmpQoSProfile();
        }
        else if (request->remove_profile)
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] request: remove %s [%s].", 
                tp.name.c_str(), tp.type.str.c_str());
            this->removeTmpQoSProfile(tp.fullName);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] request: set %s (%02d/%02d/%02d) [%s].", 
                tp.fullName.c_str(), request->qos_profile.history, request->qos_profile.depth, request->qos_profile.reliability, tp.type.str.c_str());
            this->setTmpQoSProfile(tp.fullName, CvtMsgToRMWQoS(request->qos_profile));
        }
        response->response = true;
        response->qid = this->qid_.load();
        return;
    }

    void _reqServiceCallback(const std::shared_ptr<vehicle_interfaces::srv::QosReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::QosReq::Response> response)
    {
        if (request->topic_name == "")
        {
            response->response = false;
            response->qid = this->qid_.load();
            return;
        }

        auto tp = TopicProp(request->topic_name, request->topic_type);

        RCLCPP_INFO(this->get_logger(), "[QoSServer::_reqServiceCallback] Request: %s [%s].", tp.name.c_str(), tp.type.str.c_str());

        // Prepare qmap lock
        std::unique_lock<std::mutex> locker(this->qmapLock_, std::defer_lock);

        locker.lock();
        auto qmapTmp = this->qmap_;
        locker.unlock();

REQ_CHECK_TOPIC_NAME:
        // Check topic name
        if (qmapTmp.find(tp.fullName) == qmapTmp.end())
        {
            if (tp.type == TopicProp::BOTH)
            {
                // Topic name not listed in qmap
                rclcpp::QoS* ret = new rclcpp::QoS(10);
                response->qos_profile = CvtRMWQoSToMsg(ret->get_rmw_qos_profile());
                response->response = false;
                response->qid = this->qid_.load();
            }
            else// topic_type = "publisher" or "subscription" not found. Use "both" then search again.
            {
                tp = TopicProp(request->topic_name, TopicProp::BOTH);
                goto REQ_CHECK_TOPIC_NAME;
            }
        }
        else
        {
            // Found topic name
            response->qos_profile = CvtRMWQoSToMsg(qmapTmp[tp.fullName]);
            response->response = true;
            response->qid = this->qid_.load();
        }

        // Logger
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_reqServiceCallback] Response: qid: %04d %s [%s] (found: %s).", 
            response->qid, tp.name.c_str(), tp.type.str.c_str(), response->response ? "true" : "false");
    }

    // Publish QoS update signal
    void _timerCallback()
    {
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
    std::pair<std::vector<std::string>, bool> _qmapUpdateCheck()
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        
        std::pair<std::vector<std::string>, bool> updateVec;// Need to be trans to non-fullname
        updateVec.second = this->qmap_.size() != this->qmapTmp_.size();// True if different sizes
        for (auto& [k, v] : this->qmapTmp_)
        {
            try
            {
                if (!CompRMWQoS(v, this->qmap_[k]))
                    updateVec.first.push_back(TopicProp::retrieveTopicProp(k).name);
            }
            catch (...)
            {
                updateVec.first.push_back(TopicProp::retrieveTopicProp(k).name);
            }
        }
        updateVec.second |= updateVec.first.size() > 0;

        return updateVec;
    }

public:
    QoSServer(const std::string& nodeName, const std::string& serviceName, const std::string& recordFilePath) : rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        recordFilePath_(recordFilePath), 
        qid_(0), 
        enablePubF_(false), 
        pubInterval_ms_(10000)
    {
        {// Check QoS directory
            char buf[512];
            sprintf(buf, "mkdir -p %s", this->recordFilePath_.parent_path().generic_string().c_str());
            system(buf);
        }

        if (!this->loadQmapFromJSON(this->recordFilePath_))
            RCLCPP_WARN(this->get_logger(), "[QoSServer] Record file not found: %s", this->recordFilePath_.generic_string().c_str());
        else
        {
            this->setQmap();// If loaded, the qid will started at 1, forced clients to check once while start up.
            printf("Qmap found, size: %d, qid: %d\n", this->qmap_.size(), this->qid_.load());
            for (const auto& [k, v] : this->qmap_)
                printf("%s (%02d/%02d/%02d)\n", k.c_str(), v.history, v.depth, v.reliability);
        }

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
            RCLCPP_INFO(this->get_logger(), "[QoSServer::startPublish] Start publish timer.");
            return true;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::startPublish] Publish timer not set.");
        return {false, "[QoSServer::startPublish] Publish timer not set."};
    }

    ReasonResult<bool> stopPublish()
    {
        if (this->pubTimer_ != nullptr)
        {
            this->pubTimer_->stop();
            RCLCPP_INFO(this->get_logger(), "[QoSServer::stopPublish] Publish timer stopped.");
            return true;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::stopPublish] Publish timer not set.");
        return {false, "[QoSServer::stopPublish] Publish timer not set."};
    }

    // Set single profile in tmp qmap.
    void setTmpQoSProfile(const std::string& topicName, const rmw_qos_profile_t& prof)
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        this->qmapTmp_[topicName] = prof;
    }

    // Remove single profile in tmp qmap.
    void removeTmpQoSProfile(const std::string& topicName)
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        this->qmapTmp_.erase(topicName);
    }

    // Clear qmapTmp, if process setQmap() will change qid.
    void clearTmpQoSProfile()
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        this->qmapTmp_.clear();
    }

    // Recover qmapTmp from qmap, if process setQmap() will not change qid.
    void recoverTmpQoSProfile()
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        this->qmapTmp_ = this->qmap_;
    }

    // Set qmap and dump file if tmp qmap has changed.
    void setQmap()
    {
        auto [updateVec, updateF] = this->_qmapUpdateCheck();
        if (updateF)
        {
            std::unique_lock<std::mutex> locker(this->qmapLock_, std::defer_lock);
            locker.lock();
            this->qmap_ = this->qmapTmp_;
            this->qid_ += 1;
            this->updateList_ = updateVec;
            locker.unlock();
            this->dumpQmapToJSON(this->recordFilePath_);
        }
    }

    // Load record file, stored into qmapTmp.
    bool loadQmapFromJSON(fs::path filePath)
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        try
        {
            nlohmann::json json;
            json.update(nlohmann::json::parse(std::ifstream(filePath)));
            for (const auto& [fullTopicName, prof] : json.items())
            {
                rmw_qos_profile_t _profile;
                _profile.history = (rmw_qos_history_policy_e)prof["history"];
                _profile.depth = prof["depth"];
                _profile.reliability = (rmw_qos_reliability_policy_e)prof["reliability"];
                _profile.durability = (rmw_qos_durability_policy_e)prof["durability"];
                _profile.deadline = CvtMsgToRMWTime(prof["deadline_ms"]);
                _profile.lifespan = CvtMsgToRMWTime(prof["lifespan_ms"]);
                _profile.liveliness = (rmw_qos_liveliness_policy_e)prof["liveliness"];
                _profile.liveliness_lease_duration = CvtMsgToRMWTime(prof["liveliness_lease_duration_ms"]);
                this->qmapTmp_[fullTopicName] = _profile;
            }
        }
        catch (...)
        {
            this->qmapTmp_ = this->qmap_;// Recover tmp if load failed
            return false;
        }
        return true;
    }

    // Dump qmap to file
    bool dumpQmapToJSON(fs::path filePath)
    {
        std::lock_guard<std::mutex> locker(this->qmapLock_);
        try
        {
            nlohmann::json json;
            for (auto& [fullTopicName, prof] : this->qmap_)
            {
                json[fullTopicName]["history"] = (int8_t)prof.history;
                json[fullTopicName]["depth"] = prof.depth;
                json[fullTopicName]["reliability"] = (int8_t)prof.reliability;
                json[fullTopicName]["durability"] = (int8_t)prof.durability;
                json[fullTopicName]["deadline_ms"] = CvtRMWTimeToMsg(prof.deadline);
                json[fullTopicName]["lifespan_ms"] = CvtRMWTimeToMsg(prof.lifespan);
                json[fullTopicName]["liveliness"] = (int8_t)prof.liveliness;
                json[fullTopicName]["liveliness_lease_duration_ms"] = CvtRMWTimeToMsg(prof.liveliness_lease_duration);
            }
            std::ofstream outFile(filePath);
            outFile << json;
        }
        catch (...)
        {
            return false;
        }
        return true;
    }
};

}// namespace vehicle_interfaces
