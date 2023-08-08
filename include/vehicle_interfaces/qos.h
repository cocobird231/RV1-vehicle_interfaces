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
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/timesync.h"
#include "vehicle_interfaces/msg/qos_update.hpp"
#include "vehicle_interfaces/srv/qos_reg.hpp"
#include "vehicle_interfaces/srv/qos_req.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace vehicle_interfaces
{

// double ms to rmw_time_s
inline rmw_time_s CvtMsgToRMWTime(const double& time_ms)
{
    return { time_ms / 1000, (time_ms - (uint64_t)time_ms) * 1000000 };
}

// rmw_time_s to double
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

std::shared_ptr<vehicle_interfaces::srv::QosReq::Response> CvtRMWQoSToMsg(const rmw_qos_profile_t& prof)
{
    std::shared_ptr<vehicle_interfaces::srv::QosReq::Response> ret;
    ret->history = prof.history;
    ret->depth = prof.depth;
    ret->reliability = prof.reliability;
    ret->durability = prof.durability;
    ret->deadline_ms = CvtRMWTimeToMsg(prof.deadline);
    ret->lifespan_ms = CvtRMWTimeToMsg(prof.lifespan);
    ret->liveliness = prof.liveliness;
    ret->liveliness_lease_duration_ms = CvtRMWTimeToMsg(prof.liveliness_lease_duration);
    return ret;
}

rmw_qos_profile_t CvtMsgToRMWQoS(const std::shared_ptr<vehicle_interfaces::srv::QosReg::Request> req)
{
    rmw_qos_profile_t prof;
    prof.history = (rmw_qos_history_policy_t)req->history;
    prof.depth = req->depth;
    prof.reliability = (rmw_qos_reliability_policy_t)req->reliability;
    prof.durability = (rmw_qos_durability_policy_t)req->durability;
    prof.deadline = CvtMsgToRMWTime(req->deadline_ms);
    prof.lifespan = CvtMsgToRMWTime(req->lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_t)req->liveliness;
    prof.liveliness_lease_duration = CvtMsgToRMWTime(req->liveliness_lease_duration_ms);
    return prof;
}

rmw_qos_profile_t CvtMsgToRMWQoS(const std::shared_ptr<vehicle_interfaces::srv::QosReq::Response> res)
{
    rmw_qos_profile_t prof;
    prof.history = (rmw_qos_history_policy_t)res->history;
    prof.depth = res->depth;
    prof.reliability = (rmw_qos_reliability_policy_t)res->reliability;
    prof.durability = (rmw_qos_durability_policy_t)res->durability;
    prof.deadline = CvtMsgToRMWTime(res->deadline_ms);
    prof.lifespan = CvtMsgToRMWTime(res->lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_t)res->liveliness;
    prof.liveliness_lease_duration = CvtMsgToRMWTime(res->liveliness_lease_duration_ms);
    return prof;
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
    std::function<void(QoSUpdateNode*, std::map<std::string, rclcpp::QoS*>)> qosCallbackFunc_;

    // Node enable
    std::atomic<bool> nodeEnableF_;

private:
    void _topic_callback(const vehicle_interfaces::msg::QosUpdate::SharedPtr msg)
    {
        if (!this->callbackF_)// No callback function assigned
            return;
        
        if (msg->qid == this->qosID_)// Ignore update in same qos ID
            return;

        bool errF = false;
        
        std::map<std::string, rclcpp::QoS*> qmap;

        std::unique_lock<std::mutex> locker(this->subscriptionLock_, std::defer_lock);
        locker.lock();
        for (auto& myTopic : this->qosTopicNameVec_)
        {
            for (auto& newTopic : msg->topic_table)
            {
                if (myTopic == newTopic)
                {
                    try
                    {
                        qmap[myTopic] = this->requestQoS(myTopic);
                    }
                    catch(...)
                    {
                        errF = true;
                    }
                    break;
                }
            }
        }
        locker.unlock();

        if (errF)
            return;
        
        if (qmap.size() > 0)
            this->qosCallbackFunc_(this, qmap);
        
        this->qosID_ = msg->qid;
    }

    void _connToService(rclcpp::ClientBase::SharedPtr client)
    {
        int errCnt = 5;
        while (!client->wait_for_service(1s) && errCnt-- > 0)
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "[QoSUpdateNode::_connToService] Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::_connToService] Service not available, waiting again...");
        }
        if (errCnt < 0)
            RCLCPP_ERROR(this->get_logger(), "[QoSUpdateNode::_connToService] Connect to service failed.");
        else
            RCLCPP_INFO(this->get_logger(), "[QoSUpdateNode::_connToService] Service connected.");
    }

public:
    QoSUpdateNode(std::string nodeName, std::string qosServiceName) : rclcpp::Node(nodeName), 
        qosID_(0), 
        callbackF_(false), 
        nodeEnableF_(false)
    {
        if (qosServiceName == "")
            return;
        this->reqClientNode_ = rclcpp::Node::make_shared(nodeName + "_qosreq_client");
        this->reqClient_ = this->reqClientNode_->create_client<vehicle_interfaces::srv::QosReq>(qosServiceName + "_Req");
        printf("[QoSUpdateNode] Connecting to qosreq server: %s\n", qosServiceName.c_str());
        this->_connToService(this->reqClient_);

        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::QosUpdate>(qosServiceName, 
            10, std::bind(&QoSUpdateNode::_topic_callback, this, std::placeholders::_1));
        this->nodeEnableF_ = true;
    }

    void addQoSTracking(const std::string& topicName)
    {
        if (!this->nodeEnableF_)
            return;
        std::lock_guard<std::mutex> locker(this->subscriptionLock_);
        for (auto& i : this->qosTopicNameVec_)
            if (i == topicName)
                return;
        this->qosTopicNameVec_.push_back(topicName);
        this->qosID_ = 0;
    }

    void addQoSCallbackFunc(const std::function<void(QoSUpdateNode*, std::map<std::string, rclcpp::QoS*>)>& func)
    {
        if (!this->nodeEnableF_)
            return;
        std::lock_guard<std::mutex> locker(this->subscriptionLock_);
        this->qosCallbackFunc_ = func;
        this->callbackF_ = true;
    }

    rclcpp::QoS* requestQoS(const std::string& topicName)
    {
        if (!this->nodeEnableF_)
            throw "Request QoS Failed";// Request QoS failed
        auto request = std::make_shared<vehicle_interfaces::srv::QosReq::Request>();
        request->topic_name = topicName;
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 10ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 10ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            if (res->response)
            {
                std::lock_guard<std::mutex> locker(this->subscriptionLock_);

                rmw_qos_profile_t prof;
                prof.history = (rmw_qos_history_policy_t)res->history;
                prof.depth = res->depth;
                prof.reliability = (rmw_qos_reliability_policy_t)res->reliability;
                prof.durability = (rmw_qos_durability_policy_t)res->durability;
                prof.deadline = CvtMsgToRMWTime(res->deadline_ms);
                prof.lifespan = CvtMsgToRMWTime(res->lifespan_ms);
                prof.liveliness = (rmw_qos_liveliness_policy_t)res->liveliness;
                prof.liveliness_lease_duration = CvtMsgToRMWTime(res->liveliness_lease_duration_ms);

                // this->qosID_ = res->qid;
                return new rclcpp::QoS(rclcpp::QoSInitialization(prof.history, prof.depth), prof);
            }
        }
        throw "Request QoS Failed";// Request QoS failed
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
        std::unique_lock<std::mutex> locker(this->paramsLock_, std::defer_lock);

        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
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
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] request: remove %s [%d]");
            this->removeTmpQoSProfile(request->topic_name);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_regServiceCallback] request: set %s [%d]");
            this->setTmpQoSProfile(request->topic_name, CvtMsgToRMWQoS(request));
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
            response = CvtRMWQoSToMsg(ret->get_rmw_qos_profile());
            response->response = false;
            response->qid = this->qid_.load();
        }
        else
        {
            // Found topic name
            response = CvtRMWQoSToMsg(qmapTmp[request->topic_name]);
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
    QoSServer(const std::string& nodeName, const std::string& serviceName, double pubInterval_ms) : rclcpp::Node(nodeName), 
        qid_(0), enablePubF_(false), pubInterval_ms_(pubInterval_ms), nodeName_(nodeName)
    {
        this->declare_parameter<bool>("enabled_publish", this->enablePubF_);
        this->declare_parameter<double>("publish_interval_ms", this->pubInterval_ms_);
        this->_getParams();

        this->regServer_ = this->create_service<vehicle_interfaces::srv::QosReg>(serviceName + "_Reg", 
            std::bind(&QoSServer::_regServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        this->reqServer_ = this->create_service<vehicle_interfaces::srv::QosReq>(serviceName + "_Req", 
            std::bind(&QoSServer::_reqServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

        this->pub_ = this->create_publisher<vehicle_interfaces::msg::QosUpdate>(serviceName, 10);

        if (pubInterval_ms > 0)
            this->pubTimer_ = new Timer(pubInterval_ms, std::bind(&QoSServer::_timerCallback, this));
        else
            this->pubTimer_ = nullptr;
        
        RCLCPP_INFO(this->get_logger(), "[QoSServer] Constructed");
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
