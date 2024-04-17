#pragma once
#include <condition_variable>
#include <stdexcept>
#include <memory>

#include "vehicle_interfaces/interactive_publisher.h"
#include "vehicle_interfaces/interactive_subscription.h"
#include "vehicle_interfaces/timer.h"
#include "vehicle_interfaces/utils.h"

#include "vehicle_interfaces/msg/id_table.hpp"

#include "vehicle_interfaces/msg/qos_profile.hpp"
#include "vehicle_interfaces/msg/topic_device_info.hpp"

#include "vehicle_interfaces/srv/qos_reg.hpp"
#include "vehicle_interfaces/srv/qos_req.hpp"
#include "vehicle_interfaces/srv/topic_device_info_reg.hpp"

#include <fstream>
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;

namespace vehicle_interfaces
{

/**
 * ============================================================================
 * RMW time functions.
 * ============================================================================
 */
#if ROS_DISTRO < 2// Foxy and older
/**
 * @brief Convert time double ms to rmw_time_t.
 * @param[in] time_ms The double ms.
 * @return The rmw_time_t.
 */
inline rmw_time_t CvtMsgToRMWTime(const double& time_ms)
{
    return { time_ms / 1000, (time_ms - (uint64_t)time_ms) * 1000000 };
}

/**
 * @brief Convert rmw_time_t to time double ms.
 * @param[in] rmwT The rmw_time_t.
 * @return The double ms.
 */
inline double CvtRMWTimeToMsg(const rmw_time_t& rmwT)
{
    return rmwT.sec * 1000 + rmwT.nsec / 1000000.0;
}

/**
 * @brief Compare rmw_time_t.
 * @param[in] rmwT1 The first rmw_time_t.
 * @param[in] rmwT2 The second rmw_time_t.
 * @return 1 if rmwT1 > rmwT2; -1 if rmwT1 < rmwT2; 0 if rmwT1 = rmwT2.
*/
inline int CompRMWTime(const rmw_time_t& rmwT1, const rmw_time_t& rmwT2)
{
    uint64_t t1 = static_cast<uint64_t>(rmwT1.sec * 1000000000) + rmwT1.nsec;
    uint64_t t2 = static_cast<uint64_t>(rmwT2.sec * 1000000000) + rmwT2.nsec;
    uint64_t ret = t1 - t2;
    return ret > 0 ? 1 : (ret < 0 ? -1 : 0);
}
#else// Humble and newer
/**
 * @brief Convert time double ms to rmw_time_s.
 * @param[in] time_ms The double ms.
 * @return The rmw_time_s.
 */
inline rmw_time_s CvtMsgToRMWTime(const double& time_ms)
{
    return { time_ms / 1000, (time_ms - (uint64_t)time_ms) * 1000000 };
}

/**
 * @brief Convert rmw_time_s to time double ms.
 * @param[in] rmwT The rmw_time_s.
 * @return The double ms.
 */
inline double CvtRMWTimeToMsg(const rmw_time_s& rmwT)
{
    return rmwT.sec * 1000 + rmwT.nsec / 1000000.0;
}

/**
 * @brief Compare rmw_time_s.
 * @param[in] rmwT1 The first rmw_time_s.
 * @param[in] rmwT2 The second rmw_time_s.
 * @return 1 if rmwT1 > rmwT2; -1 if rmwT1 < rmwT2; 0 if rmwT1 = rmwT2.
*/
inline int CompRMWTime(const rmw_time_s& rmwT1, const rmw_time_s& rmwT2)
{
    uint64_t t1 = static_cast<uint64_t>(rmwT1.sec * 1000000000) + rmwT1.nsec;
    uint64_t t2 = static_cast<uint64_t>(rmwT2.sec * 1000000000) + rmwT2.nsec;
    uint64_t ret = t1 - t2;
    return ret > 0 ? 1 : (ret < 0 ? -1 : 0);
}
#endif



/**
 * ============================================================================
 * RMW QoS functions.
 * ============================================================================
 */
#if ROS_DISTRO < 2// Foxy and older
/**
 * @brief Get the name of the QoS reliability policy.
 * @param[in] value The QoS reliability policy.
 * @return The name of the QoS reliability policy.
 */
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
#else// Humble and newer
/**
 * @brief Get the name of the QoS reliability policy.
 * @param[in] value The QoS reliability policy.
 * @return The name of the QoS reliability policy.
 */
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
#endif

/**
 * @brief Convert the rmw_qos_profile_t to the QosProfile message.
 * @param[in] prof The rmw_qos_profile_t.
 * @return The QosProfile message.
 */
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

/**
 * @brief Convert QosProfile message to rmw_qos_profile_t.
 * @param[in] msg The QosProfile message.
 * @return The rmw_qos_profile_t.
 */
rmw_qos_profile_t CvtMsgToRMWQoS(const vehicle_interfaces::msg::QosProfile& msg)
{
    rmw_qos_profile_t prof = rclcpp::QoS(10).get_rmw_qos_profile();
#if ROS_DISTRO < 2// Foxy and older
    prof.history = (rmw_qos_history_policy_t)msg.history;
    prof.depth = msg.depth;
    prof.reliability = (rmw_qos_reliability_policy_t)msg.reliability;
    prof.durability = (rmw_qos_durability_policy_t)msg.durability;
    prof.deadline = CvtMsgToRMWTime(msg.deadline_ms);
    prof.lifespan = CvtMsgToRMWTime(msg.lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_t)msg.liveliness;
    prof.liveliness_lease_duration = CvtMsgToRMWTime(msg.liveliness_lease_duration_ms);
#else
    prof.history = (rmw_qos_history_policy_e)msg.history;
    prof.depth = msg.depth;
    prof.reliability = (rmw_qos_reliability_policy_e)msg.reliability;
    prof.durability = (rmw_qos_durability_policy_e)msg.durability;
    prof.deadline = CvtMsgToRMWTime(msg.deadline_ms);
    prof.lifespan = CvtMsgToRMWTime(msg.lifespan_ms);
    prof.liveliness = (rmw_qos_liveliness_policy_e)msg.liveliness;
    prof.liveliness_lease_duration = CvtMsgToRMWTime(msg.liveliness_lease_duration_ms);
#endif
    // printf("history: %d\ndepth: %d\nreliability: %d\ndurability: %d\n \\
    //         deadline:%d,%d\nlifespan: %d,%d\nliveliness: %d\nliveliness_lease_duration: %d,%d\n", 
    //         prof.history, prof.depth, prof.reliability, prof.durability, 
    //         prof.deadline.sec, prof.deadline.nsec, prof.lifespan.sec, prof.lifespan.nsec, 
    //         prof.liveliness, prof.liveliness_lease_duration.sec, prof.liveliness_lease_duration.nsec);
    return prof;
}

/**
 * @brief Compare two rmw_qos_profile_t.
 * @param[in] p1 The first rmw_qos_profile_t.
 * @param[in] p2 The second rmw_qos_profile_t.
 * @return true if the two QoS profiles are the same; false otherwise.
 */
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

/**
 * @brief Convert the rmw_qos_profile_t to the rclcpp::QoS.
 * @param[in] prof The rmw_qos_profile_t.
 * @return The rclcpp::QoS.
 * @note The function does not support the deadline, lifespan, and liveliness settings.
 */
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



/**
 * ============================================================================
 * RMW QoS read and write functions.
 * ============================================================================
 */

/**
 * @brief Load the QoS profile from the JSON file.
 * @param[in] qosFilePath The path of the JSON file.
 * @param[out] profile The QoS profile.
 * @return True if the QoS profile is loaded successfully; false otherwise.
 */
bool LoadRMWQoSFromJSON(const fs::path& qosFilePath, rmw_qos_profile_t& profile)
{
    try
    {
        nlohmann::json json;
        json.update(nlohmann::json::parse(std::ifstream(qosFilePath)));
#if ROS_DISTRO < 2// Foxy and older
        profile.history = (rmw_qos_history_policy_t)json["history"];
        profile.depth = json["depth"];
        profile.reliability = (rmw_qos_reliability_policy_t)json["reliability"];
        profile.durability = (rmw_qos_durability_policy_t)json["durability"];
        profile.deadline = CvtMsgToRMWTime(json["deadline_ms"]);
        profile.lifespan = CvtMsgToRMWTime(json["lifespan_ms"]);
        profile.liveliness = (rmw_qos_liveliness_policy_t)json["liveliness"];
        profile.liveliness_lease_duration = CvtMsgToRMWTime(json["liveliness_lease_duration_ms"]);
#else
        profile.history = (rmw_qos_history_policy_e)json["history"];
        profile.depth = json["depth"];
        profile.reliability = (rmw_qos_reliability_policy_e)json["reliability"];
        profile.durability = (rmw_qos_durability_policy_e)json["durability"];
        profile.deadline = CvtMsgToRMWTime(json["deadline_ms"]);
        profile.lifespan = CvtMsgToRMWTime(json["lifespan_ms"]);
        profile.liveliness = (rmw_qos_liveliness_policy_e)json["liveliness"];
        profile.liveliness_lease_duration = CvtMsgToRMWTime(json["liveliness_lease_duration_ms"]);
#endif
        return true;
    }
    catch(...)
    {
        return false;
    }
}

/**
 * @brief Dump the QoS profile to the JSON file.
 * @param[in] qosFilePath The path of the JSON file.
 * @param[in] profile The QoS profile.
 * @return True if the QoS profile is dumped successfully; false otherwise.
 * @note The function will overwrite the file if it exists.
 */
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



/**
 * ============================================================================
 * RMW QoS and InteractiveNode command functions.
 * ============================================================================
 */

/**
 * @brief Convert the rmw_qos_profile_t to the InteractivceNode node command arguments.
 * @details The function converts the rmw_qos_profile_t to the arguments of the node command.
 * @param[in] prof The QoS profile.
 * @return The arguments of the node command.
 */
std::vector<std::string> CvtRMWQoSToInteractiveNodeCommandArgs(const rmw_qos_profile_t& prof)
{
    std::vector<std::string> ret;
    ret.push_back("history:" + std::to_string(prof.history));
    ret.push_back("depth:" + std::to_string(prof.depth));
    ret.push_back("reliability:" + std::to_string(prof.reliability));
    ret.push_back("durability:" + std::to_string(prof.durability));
    ret.push_back("deadline:" + std::to_string(CvtRMWTimeToMsg(prof.deadline)));
    ret.push_back("lifespan:" + std::to_string(CvtRMWTimeToMsg(prof.lifespan)));
    ret.push_back("liveliness:" + std::to_string(prof.liveliness));
    ret.push_back("liveliness_lease_duration:" + std::to_string(CvtRMWTimeToMsg(prof.liveliness_lease_duration)));
    return ret;
}

/**
 * @brief Convert the InteractivceNode node command arguments to rclcpp::QoS.
 * @details The function converts the arguments of the node command to rclcpp::QoS.
 * @param[in] args The arguments of the node command funtion.
 * @return The rclcpp::QoS object.
 */
rmw_qos_profile_t CvtInteractiveNodeCommandArgsToRMWQoS(const std::vector<std::string>& args)
{
    /**
     * The arguments of the node command arguments should be in the following format:
     * "history:0"
     * "depth:10"
     * "reliability:0"
     * "durability:0"
     * "deadline:0"
     * "lifespan:0"
     * "liveliness:0"
     * "liveliness_lease_duration:0"
     */
    rmw_qos_profile_t qosProp = rmw_qos_profile_default;
    for (const auto& i : args)
    {
        auto arg = vehicle_interfaces::split(i, ":");
        if (arg.size() != 2)
            continue;
        try
        {
            if (arg[0] == "history")
                qosProp.history = rmw_qos_history_policy_e(stoi(arg[1]));
            else if (arg[0] == "depth")
                qosProp.depth = stoi(arg[1]);
            else if (arg[0] == "reliability")
                qosProp.reliability = rmw_qos_reliability_policy_e(stoi(arg[1]));
            else if (arg[0] == "durability")
                qosProp.durability = rmw_qos_durability_policy_e(stoi(arg[1]));
            else if (arg[0] == "deadline")
                qosProp.deadline = vehicle_interfaces::CvtMsgToRMWTime(stoi(arg[1]));
            else if (arg[0] == "lifespan")
                qosProp.lifespan = vehicle_interfaces::CvtMsgToRMWTime(stoi(arg[1]));
            else if (arg[0] == "liveliness")
                qosProp.liveliness = rmw_qos_liveliness_policy_e(stoi(arg[1]));
            else if (arg[0] == "liveliness_lease_duration")
                qosProp.liveliness_lease_duration = vehicle_interfaces::CvtMsgToRMWTime(stoi(arg[1]));
        }
        catch(...)
        {
            continue;
        }
    }
    return qosProp;
}



/**
 * ============================================================================
 * QoSNode functions and classes.
 * ============================================================================
 */

/**
 * @brief The QoSNode class.
 * @details The class is used to register the topic device to the QoSServer.
 */
class QoSNode : virtual public rclcpp::Node
{
private:
    const std::string qosServiceName_;// The name of the QoSServer service.
    const fs::path qosDirPath_;// The directory path of the QoS configuration files.

    rclcpp::Client<vehicle_interfaces::srv::TopicDeviceInfoReg>::SharedPtr topicDevInfoRegClient_;// The client to register the topic device to the QoSServer.
    rclcpp::Node::SharedPtr topicDevInfoRegClientNode_;// The node to create the client to register the topic device to the QoSServer.

    std::map<std::string, std::pair<vehicle_interfaces::msg::TopicDeviceInfo, bool> > topicDevInfoStatusMap_;// The map to store the topic device information and the registration status by the node name.
    std::mutex topicDevStatusMapMutex_;// The mutex to lock the topic device status map.
    std::condition_variable topicDevStatusMapCV_;// The condition variable to notify the topic device status map change.
    vehicle_interfaces::unique_thread topicDevInfoRegTh_;// The thread to register the topic device information to the QoSServer. This thread runs the _topicDevInfoRegTh() function.
    std::atomic<bool> stopTopicDevInfoRegThF_;// Flag to stop the topic device registration thread.

    rclcpp::Subscription<vehicle_interfaces::msg::IDTable>::SharedPtr topicDevInfoRegSub_;// The subscription to the QoSServer topic device registration topic.
    std::mutex topicDevInfoRegSubMutex_;// The mutex to lock the topic device registration subscription callback function.

    std::atomic<bool> nodeEnableF_;// Flag indicating whether the QoSNode is enabled.
    std::atomic<bool> exitF_;// Flag indicating whether the service thread is running.

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
     * @brief QoS update event handler.
     * @details The function is called when the interactive service server receives a QoS update request.
     * @param[in] node Pointer to the interactive node. The node should be a MultiInteractiveTopic or InteractiveTopic.
     * @param[in] deviceID The device ID of the request sender.
     * @param[in] args The arguments of the node command function.
     */
    void _qosUpdateEventHandler(InteractiveNode *node, const std::string deviceID, const std::vector<std::string> args)
    {
        auto qos = CvtInteractiveNodeCommandArgsToRMWQoS(args);
        if (auto qosNode = dynamic_cast<MultiInteractiveTopicNode *>(node))
        {
            qosNode->setQoS(CvtRMWQoSToRclQoS(qos));
        }
        else if (auto qosNode = dynamic_cast<InteractiveTopicNode *>(node))
        {
            qosNode->setQoS(CvtRMWQoSToRclQoS(qos));
        }
        std::string fileName = node->getNodeName();
        vehicle_interfaces::replace_all(fileName, "/", "_");
        DumpRMWQoSToJSON(this->qosDirPath_ / (fileName + ".json"), qos);
    }

    /**
     * @brief Send TopicDeviceInfoReg request.
     * @details The function sends the TopicDeviceInfoReg request to the QoSServer. The function will retry if the request fails.
     * @param[in] tInfo The topic device information.
     * @param[in] retry The number of retries.
     * @return True if the request is successful; false otherwise.
     */
    bool _sendTopicDeviceInfoReg(const vehicle_interfaces::msg::TopicDeviceInfo tInfo, int retry = 2)
    {
        auto request = std::make_shared<vehicle_interfaces::srv::TopicDeviceInfoReg::Request>();
        request->request = tInfo;
TOPIC_DEV_REG_TAG:
        retry--;
        auto result = this->topicDevInfoRegClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->topicDevInfoRegClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->topicDevInfoRegClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
            {
                RCLCPP_INFO(this->get_logger(), "[QoSNode::_sendTopicDeviceInfoReg] %s[%s] registered to the QoS server.", 
                    request->request.topic_name.c_str(), 
                    request->request.device_type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_PUBLISHER ? "Publisher" : 
                    (request->request.device_type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_SUBSCRIPTION ? "Subscription" : "Unknown"));
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[QoSNode::_sendTopicDeviceInfoReg] %s[%s] failed to register to the QoS server. Retrying...", request->request.topic_name.c_str(), 
                    request->request.device_type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_PUBLISHER ? "Publisher" : 
                    (request->request.device_type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_SUBSCRIPTION ? "Subscription" : "Unknown"));
                if (retry > 0)
                    goto TOPIC_DEV_REG_TAG;
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::_sendTopicDeviceInfoReg] Failed to connect to the QoS server. Retrying...");
            if (retry > 0)
                goto TOPIC_DEV_REG_TAG;
            return false;
        }
    }

    /**
     * @brief The thread to register the topic device information to the QoSServer.
     * @note The function is using topicDevStatusMapCV_ and topicDevStatusMapMutex_.
     * @note The function will stop when the exitF_ or stopTopicDevInfoRegThF_ is true.
     */
    void _topicDevInfoRegTh()
    {
        std::unique_lock<std::mutex> lock(this->topicDevStatusMapMutex_, std::defer_lock);
        while (!this->exitF_ && !this->stopTopicDevInfoRegThF_)
        {
            lock.lock();
            // this->topicDevStatusMapCV_.wait(lock, [this](){ return this->exitF_ || this->stopTopicDevInfoRegThF_ || !this->topicDevInfoStatusMap_.empty(); });
            this->topicDevStatusMapCV_.wait(lock);
            if (this->exitF_ || this->stopTopicDevInfoRegThF_)
            {
                lock.unlock();
                break;
            }
            auto tmpMap = this->topicDevInfoStatusMap_;
            lock.unlock();
            for (auto& [nodeName, topicDevStatus] : tmpMap)
            {
                const auto& tInfo = topicDevStatus.first;
                const auto& status = topicDevStatus.second;
                if (!status)
                {
                    if (this->_sendTopicDeviceInfoReg(tInfo))
                    {
                        std::lock_guard<std::mutex> lock(this->topicDevStatusMapMutex_);
                        this->topicDevInfoStatusMap_[nodeName].second = true;
                    }
                }
            }
        }
    }

    /**
     * @brief The callback function of the topic device registration subscription.
     * @details The function is called when the QoSServer publishes the topic device registration information.
     * @note The function will modify the status of topicDevInfoStatusMap_ and notify the topicDevStatusMapCV_.
     */
    void _topicDevInfoRegSubCbFunc(const std::shared_ptr<vehicle_interfaces::msg::IDTable> msg)
    {
        std::lock_guard<std::mutex> topicDevInfoRegSubLock(this->topicDevInfoRegSubMutex_);
        std::unique_lock<std::mutex> topicDevInfoStatusMapLock(this->topicDevStatusMapMutex_, std::defer_lock);
        topicDevInfoStatusMapLock.lock();
        for (auto& [nodeName, topicDevStatus] : this->topicDevInfoStatusMap_)
        {
            bool foundF = false;
            for (const auto& i : msg->idtable)
            {
                if (i == nodeName)
                {
                    foundF = true;
                    topicDevStatus.second = true;
                    break;
                }
            }
            if (!foundF)
            {
                RCLCPP_WARN(this->get_logger(), "[QoSNode::_topicDevInfoRegSubCbFunc] %s is not registered to the QoS server.", nodeName.c_str());
                topicDevStatus.second = false;
            }
        }
        this->topicDevStatusMapCV_.notify_all();
        topicDevInfoStatusMapLock.unlock();
    }

public:
    QoSNode(const std::string nodeName, const std::string& qosServiceName, const std::string& qosDirPath) : 
        rclcpp::Node(nodeName), 
        qosServiceName_(qosServiceName), 
        qosDirPath_(qosDirPath), 
        stopTopicDevInfoRegThF_(false), 
        nodeEnableF_(false), 
        exitF_(false)
    {
        if (qosServiceName == "")
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode] Ignored.");
            return;
        }

        {// Check QoS directory
            char buf[512];
            sprintf(buf, "mkdir -p %s", this->qosDirPath_.generic_string().c_str());
            system(buf);
        }

        this->topicDevInfoRegClientNode_ = rclcpp::Node::make_shared(nodeName + "_qos_topic_device_info_reg_client");
        this->topicDevInfoRegClient_ = this->topicDevInfoRegClientNode_->create_client<vehicle_interfaces::srv::TopicDeviceInfoReg>(this->qosServiceName_);
        this->topicDevInfoRegTh_ = vehicle_interfaces::make_unique_thread(&QoSNode::_topicDevInfoRegTh, this);

        // Subscribe the QoS registration topic.
        // This topic is used to receive the registered topic device information from the QoSServer.
        this->topicDevInfoRegSub_ = this->create_subscription<vehicle_interfaces::msg::IDTable>(this->qosServiceName_, 10, std::bind(&QoSNode::_topicDevInfoRegSubCbFunc, this, std::placeholders::_1));
        this->nodeEnableF_ = true;
        RCLCPP_INFO(this->get_logger(), "[QoSNode] Constructed.");
    }

    ~QoSNode()
    {
        this->exitF_ = true;
    }

    /**
     * @brief Add the interactive node to the QoSNode.
     * @details The function adds the node information to the QoSNode.
     * @param[in] node Pointer to the interactive node. The node should be a MultiInteractiveTopic or InteractiveTopic.
     * @param[in] tryReadQoSProfile If true, the function will try to read the QoS profile from the qosDirPath_.
     * @return True if the node is added to the QoSNode; false otherwise.
     * @note The function is using topicDevStatusMapCV_ and topicDevStatusMapMutex_.
     */
    bool addQoSNodeCommand(std::shared_ptr<InteractiveNode> node, bool tryReadQoSProfile = true)
    {
        if (!this->nodeEnableF_)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSNodeCommand] Ignored.");
            return false;
        }
        std::lock_guard<std::mutex> lock(this->topicDevStatusMapMutex_);
        if (this->topicDevInfoStatusMap_.find(node->getNodeName()) != this->topicDevInfoStatusMap_.end())
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSNodeCommand] %s is already added to the list.", node->getNodeName().c_str());
            return false;
        }

        // Master privilege.
        vehicle_interfaces::InteractiveNodeMasterPrivilege privi;
        privi.masterID = this->qosServiceName_;
        privi.targetAlive = vehicle_interfaces::InteractiveNodeMasterPrivilegeTargetAlive::TARGET_ALIVE_ALL;
        privi.targetActivity = vehicle_interfaces::InteractiveNodeMasterPrivilegeTargetActivity::TARGET_ACTIVITY_ALL;
        privi.nodeCommandSet = { "qos_update" };
        privi.requestInteractiveNode = false;

        // Add master privilege to the node and add topic device information to the list.
        if (auto topicNode = std::dynamic_pointer_cast<MultiInteractiveTopicNode>(node))
        {
            topicNode->addMasterPrivilege(privi);// MultiInteractiveTopicNode call addMasterPrivilege() to add init master target status.
            vehicle_interfaces::msg::TopicDeviceInfo tInfo;
            tInfo.node_name = topicNode->getNodeName();
            tInfo.topic_name = topicNode->getTopicName();
            tInfo.device_type = topicNode->getTopicType();
            this->topicDevInfoStatusMap_[tInfo.node_name] = { tInfo, false };
        }
        else if (auto topicNode = std::dynamic_pointer_cast<InteractiveTopicNode>(node))
        {
            topicNode->addMasterPrivilege(privi);
            vehicle_interfaces::msg::TopicDeviceInfo tInfo;
            tInfo.node_name = topicNode->getNodeName();
            tInfo.topic_name = topicNode->getTopicName();
            tInfo.device_type = topicNode->getTopicType();
            this->topicDevInfoStatusMap_[tInfo.node_name] = { tInfo, false };
        }
        else// Neither MultiInteractiveTopicNode nor InteractiveTopicNode.
        {
            RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSNodeCommand] %s is not a valid interactive topic node.", node->getNodeName().c_str());
            return false;
        }

        if (tryReadQoSProfile)// Try to read QoS profile
        {
            std::string fileName = node->getNodeName();
            vehicle_interfaces::replace_all(fileName, "/", "_");
            rmw_qos_profile_t qos = rmw_qos_profile_default;
            if (LoadRMWQoSFromJSON(this->qosDirPath_ / (fileName + ".json"), qos))
            {
                RCLCPP_INFO(this->get_logger(), "[QoSNode::addQoSNodeCommand] %s QoS profile loaded.", node->getNodeName().c_str());
                printf("QoS profile: %d/%d/%d/%d\n", qos.history, qos.depth, qos.reliability, qos.durability);
                uint8_t preStatus = node->getTargetAlive();
                node->callTargetAliveCbFunc(this->qosServiceName_, vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE);
                if (auto topicNode = std::dynamic_pointer_cast<MultiInteractiveTopicNode>(node))
                {
                    topicNode->setQoS(CvtRMWQoSToRclQoS(qos));
                }
                else if (auto topicNode = std::dynamic_pointer_cast<InteractiveTopicNode>(node))
                {
                    topicNode->setQoS(CvtRMWQoSToRclQoS(qos));
                }
                node->callTargetAliveCbFunc(this->qosServiceName_, preStatus);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[QoSNode::addQoSNodeCommand] %s QoS profile not found.", node->getNodeName().c_str());
            }
        }

        node->addNodeCommandEventHandler("qos_update", std::bind(&QoSNode::_qosUpdateEventHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(this->get_logger(), "[QoSNode::addQoSNodeCommand] Add %s to list.", node->getNodeName().c_str());
        this->topicDevStatusMapCV_.notify_all();
        return true;
    }

    /**
     * @brief Stop the topic device registration thread.
     * @details The function stops the topic device registration thread if no more addQoSNodeCommand() is called.
     * @note Strongly recommend to call this function after all addQoSNodeCommand() is called.
     * @note The device registration will still working passively if QoSServer starts publishing the IDTable.
     * @note The function is using topicDevStatusMapCV_ and topicDevStatusMapMutex_.
     */
    void addQoSNodeCommandDone()
    {
        std::lock_guard<std::mutex> lock(this->topicDevStatusMapMutex_);
        this->stopTopicDevInfoRegThF_ = true;
        this->topicDevStatusMapCV_.notify_one();
    }
};



/**
 * ============================================================================
 * TopicQoS functions and classes.
 * ============================================================================
 */

/**
 * @brief Result of the QoS comparison.
 * @details The enum is used to indicate the result of the QoS comparison.
 */
enum TopicQoSCompareResult
{
    QOS_CMP_SAME = 0,
    QOS_CMP_PUB_DIFF = 1,
    QOS_CMP_SUB_DIFF = 2,
    QOS_CMP_BOTH_DIFF = 3,
    QOS_CMP_TOPIC_DIFF = -1
};

/**
 * @brief The TopicQoSException enum.
 * @details The enum is used to indicate the exception of the TopicQoS class.
 */
enum TopicQoSException
{
    QOS_EXC_INVALID_TOPIC_NAME = 0, 
    QOS_EXC_INVALID_PUB_QOS, 
    QOS_EXC_INVALID_SUB_QOS, 
    QOS_EXC_INVALID_TOPIC_TYPE, 
    QOS_EXC_INVALID_PROPERTY
};

/**
 * @brief The TopicQoSExceptionMsg array.
 * @details The array is used to store the exception messages of the TopicQoS class.
 */
const char* TopicQoSExceptionMsg[] = 
{
    "Invalid topic name",
    "Invalid publisher QoS",
    "Invalid subscriber QoS",
    "Invalid topic type", 
    "Invalid property"
};



/**
 * @brief The TopicQoS class.
 * @details The class is used to store the QoS profiles for the topic.
 */
class TopicQoS
{
private:
    std::string topicName_;// The name of the topic.
    rmw_qos_profile_t *pubQoS_;// The QoS profile for the publisher.
    rmw_qos_profile_t *subQoS_;// The QoS profile for the subscriber.

public:
    /**
     * @brief Construct a new TopicQoS object without topic name and QoS profiles.
     * @details The function constructs a new TopicQoS object without topic name and QoS profiles.
     * @note The function does not allocate memory for the QoS profiles.
     * @note The topic name and QoS profiles should be set before using the object.
     */
    TopicQoS() : topicName_(""), pubQoS_(nullptr), subQoS_(nullptr) {}

    /**
     * @brief Construct a new TopicQoS object with the topic name.
     * @details The function constructs a new TopicQoS object with the topic name.
     * @param[in] topicName The name of the topic.
     * @note The function does not allocate memory for the QoS profiles.
     * @note The QoS profiles should be set before using the object.
     */
    TopicQoS(const std::string& topicName) : topicName_(topicName), pubQoS_(nullptr), subQoS_(nullptr) {}

    /**
     * @brief Construct a new TopicQoS object with the topic name and QoS profiles.
     * @details The function constructs a new TopicQoS object with the topic name and QoS profiles.
     * @param[in] topicName The name of the topic.
     * @param[in] pubQoS The QoS profile for the publisher.
     * @param[in] subQoS The QoS profile for the subscriber.
     */
    TopicQoS(const std::string& topicName, const rmw_qos_profile_t& pubQoS, const rmw_qos_profile_t& subQoS) : 
        topicName_(topicName), 
        pubQoS_(new rmw_qos_profile_t(pubQoS)), 
        subQoS_(new rmw_qos_profile_t(subQoS)) {}

    TopicQoS(const TopicQoS& q) : 
        topicName_(q.topicName_), 
        pubQoS_(q.pubQoS_ == nullptr ? nullptr : new rmw_qos_profile_t(*q.pubQoS_)), 
        subQoS_(q.subQoS_ == nullptr ? nullptr : new rmw_qos_profile_t(*q.subQoS_)) {}

    ~TopicQoS()
    {
        if (this->pubQoS_ != nullptr)
            delete this->pubQoS_;
        if (this->subQoS_ != nullptr)
            delete this->subQoS_;
    }

    /**
     * @brief Set the topic name.
     * @param[in] name The name of the topic.
     */
    void setTopicName(const std::string& name)
    {
        this->topicName_ = name;
    }

    /**
     * @brief Set the QoS profile for the publisher and subscriber.
     * @param[in] qos The QoS profile.
     */
    void setQoS(const rmw_qos_profile_t& qos)
    {
        if (this->pubQoS_ == nullptr)
            this->pubQoS_ = new rmw_qos_profile_t(qos);
        else
            *this->pubQoS_ = qos;
        if (this->subQoS_ == nullptr)
            this->subQoS_ = new rmw_qos_profile_t(qos);
        else
            *this->subQoS_ = qos;
    }

    /**
     * @brief Set the QoS profile for the publisher.
     * @param[in] qos The QoS profile.
     */
    void setPubQoS(const rmw_qos_profile_t& qos)
    {
        if (this->pubQoS_ == nullptr)
            this->pubQoS_ = new rmw_qos_profile_t(qos);
        else
            *this->pubQoS_ = qos;
    }

    /**
     * @brief Set the QoS profile for the subscriber.
     * @param[in] qos The QoS profile.
     */
    void setSubQoS(const rmw_qos_profile_t& qos)
    {
        if (this->subQoS_ == nullptr)
            this->subQoS_ = new rmw_qos_profile_t(qos);
        else
            *this->subQoS_ = qos;
    }

    /**
     * @brief Get the topic name.
     * @return The name of the topic.
     */
    std::string getTopicName() const
    {
        return this->topicName_;
    }

    /**
     * @brief Is topic name valid.
     * @return True if the topic name is valid; false otherwise.
     */
    bool isTopicNameValid() const
    {
        return this->topicName_ != "";
    }

    /**
     * @brief Is the QoS profile for the publisher valid.
     * @return True if the QoS profile is valid; false otherwise.
     */
    bool isPubQoSValid() const
    {
        return this->pubQoS_ != nullptr;
    }

    /**
     * @brief Is the QoS profile for the subscriber valid.
     * @return True if the QoS profile is valid; false otherwise.
     */
    bool isSubQoSValid() const
    {
        return this->subQoS_ != nullptr;
    }

    /**
     * @brief Compare the QoS profiles.
     * @details The function compares the QoS profiles with the given QoS profile.
     * @param[in] q The QoS profile to compare.
     * @return True if the QoS profiles are the same; false otherwise.
     */
    bool operator==(const TopicQoS& q) const
    {
        if (this->topicName_ != q.topicName_)
            return false;

        bool myPubF = this->isPubQoSValid();
        bool mySubF = this->isSubQoSValid();
        bool qPubF = q.isPubQoSValid();
        bool qSubF = q.isSubQoSValid();
        bool pubF = false;
        bool subF = false;

        if (myPubF && qPubF)
            pubF = CompRMWQoS(*this->pubQoS_, *q.pubQoS_) == 0;
        else if (!myPubF && !qSubF)
            pubF = true;
        if (mySubF && qSubF)
            subF = CompRMWQoS(*this->subQoS_, *q.subQoS_) == 0;
        else if (!mySubF && !qSubF)
            subF = true;
        return pubF && subF;
    }

    /**
     * @brief Compare the QoS profiles.
     * @details The function compares the QoS profiles with the given QoS profile.
     * @param[in] q The QoS profile to compare.
     * @return True if the QoS profiles are different; false otherwise.
     */
    bool operator!=(const TopicQoS& q)
    {
        return !this->operator==(q);
    }

    /**
     * @brief Compare the QoS profiles.
     * @details The function compares the QoS profiles with the given QoS profile.
     * @param[in] q The QoS profile to compare.
     * @return 0 if the QoS profiles are the same; 
     * 1 if the publisher QoS profile is different; 
     * 2 if the subscriber QoS profile is different; 
     * 3 if both publisher and subscriber QoS profiles are different; 
     * -1 if the topic names are different.
     */
    TopicQoSCompareResult operator%(const TopicQoS& q) const
    {
        if (this->topicName_ != q.topicName_)
            return TopicQoSCompareResult::QOS_CMP_TOPIC_DIFF;

        bool myPubF = this->isPubQoSValid();
        bool mySubF = this->isSubQoSValid();
        bool qPubF = q.isPubQoSValid();
        bool qSubF = q.isSubQoSValid();
        bool pubF = false;
        bool subF = false;

        if (myPubF && qPubF)
            pubF = CompRMWQoS(*this->pubQoS_, *q.pubQoS_) == 0;
        else if (!myPubF && !qSubF)
            pubF = true;

        if (mySubF && qSubF)
            subF = CompRMWQoS(*this->subQoS_, *q.subQoS_) == 0;
        else if (!mySubF && !qSubF)
            subF = true;

        int ret = 0;
        if (pubF)
            ret += 1;
        if (subF)
            ret += 2;
        return (TopicQoSCompareResult)ret;
    }

    /**
     * @brief Get the QoS profile by the type "publisher" or "subscription".
     * @param[in] type The type of the QoS profile.
     * @return The QoS profile.
     * @note The function will throw TopicQoSException if the input type is invalid or the QoS profile is invalid.
     */
    const rmw_qos_profile_t& operator[](const std::string type) const
    {
        if (type == "publisher")
        {
            if (this->pubQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_PUB_QOS;
            return *this->pubQoS_;
        }
        else if (type == "subscription")
        {
            if (this->subQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_SUB_QOS;
            return *this->subQoS_;
        }
        else
            throw TopicQoSException::QOS_EXC_INVALID_TOPIC_TYPE;
    }

    /**
     * @brief Get the QoS profile by the type "publisher" or "subscription".
     * @param[in] type The type of the QoS profile.
     * @return The QoS profile.
     * @note The function will throw TopicQoSException if the input type is invalid or the QoS profile is invalid.
     */
    const rmw_qos_profile_t& operator[](const char* type) const
    {
        if (type == "publisher")
        {
            if (this->pubQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_PUB_QOS;
            return *this->pubQoS_;
        }
        else if (type == "subscription")
        {
            if (this->subQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_SUB_QOS;
            return *this->subQoS_;
        }
        else
            throw TopicQoSException::QOS_EXC_INVALID_TOPIC_TYPE;
    }

    /**
     * @brief Get the QoS profile by the type DEVICE_TYPE_XXX under TopicDeviceInfo.msg.
     * @param[in] type The type of the QoS profile.
     * @return The QoS profile.
     * @note The function will throw TopicQoSException if the input type is invalid or the QoS profile is invalid.
     */
    const rmw_qos_profile_t& operator[](const uint8_t& type) const
    {
        if (type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_PUBLISHER)
        {
            if (this->pubQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_PUB_QOS;
            return *this->pubQoS_;
        }
        else if (type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_SUBSCRIPTION)
        {
            if (this->subQoS_ == nullptr)
                throw TopicQoSException::QOS_EXC_INVALID_SUB_QOS;
            return *this->subQoS_;
        }
        else
            throw TopicQoSException::QOS_EXC_INVALID_TOPIC_TYPE;
    }

    /**
     * @brief Check TopicQoS is valid.
     * @return True if the TopicQoS is valid; false otherwise.
     */
    operator bool() const
    {
        return this->topicName_ != "" && this->pubQoS_ != nullptr && this->subQoS_ != nullptr;
    }
};

}


namespace std
{
    template <>
    struct less<vehicle_interfaces::TopicQoS>
    {
        bool operator()(const vehicle_interfaces::TopicQoS& lhs, const vehicle_interfaces::TopicQoS& rhs) const
        {
            return lhs.getTopicName() < rhs.getTopicName();
        }
    };
}


namespace vehicle_interfaces
{

/**
 * ============================================================================
 * QoSServer functions and classes.
 * ============================================================================
 */

/**
 * @brief The QoSServer class.
 * @details The class is used to manage the QoS profiles for the topics.
 */
class QoSServer : public rclcpp::Node
{
private:
    // Service settings.
    const std::string serviceName_;// The name of the QoSServer service.
    const std::string qosRegServiceName_;// The name of the QoS registration service.
    const std::string qosReqServiceName_;// The name of the QoS request service.
    const std::string topicDevInfoRegServiceName_;// The name of the topic device information registration service.
    const fs::path qosDirPath_;// The directory path of the QoS configuration files.


    /**
     * QoS profile service management.
     */
    // QoS registration and request services.
    rclcpp::Service<vehicle_interfaces::srv::QosReg>::SharedPtr qosRegService_;// The service to receive the QoS profile registration.
    std::mutex qosRegServiceMutex_;// The mutex to lock the QoS registration callback function.
    rclcpp::Service<vehicle_interfaces::srv::QosReq>::SharedPtr qosReqService_;// The service to receive the QoS profile request.
    std::mutex qosReqServiceMutex_;// The mutex to lock the QoS request callback function.

    /**
     * Topic device management.
     */
    // Topic device registration service.
    rclcpp::Service<vehicle_interfaces::srv::TopicDeviceInfoReg>::SharedPtr topicDevInfoRegService_;// The service to receive the topic device requisition from the QoSNode.
    std::mutex topicDevInfoRegServiceMutex_;// The mutex to lock the callack function.

    // Topic device information publisher.
    // The QoSNode will subscribe the topic to receive the registered topic device information, then determine whether send the registration request.
    rclcpp::Publisher<vehicle_interfaces::msg::IDTable>::SharedPtr topicDevInfoRegPub_;// The publisher to publish the registered topic device information.
    vehicle_interfaces::Timer* topicDevInfoRegPubTimer_;// The timer to publish the registered topic device information.
    const int topicDevInfoRegPubCnt_;// The number of published times when the topicDevInfoRegPubTimer_ is triggered.

    // Topic device information.
    std::map<std::string, vehicle_interfaces::msg::TopicDeviceInfo> topicDevInfoMap_;// The map to store the topic device information by the node name.
    std::mutex topicDevInfoMapMutex_;// The mutex to lock the topic device information map.

    
    // QoS management.
    std::map<std::string, TopicQoS> topicQoSMap_;// The map to store the QoS profiles by the topic name.
    std::mutex topicQoSMapMutex_;// The mutex to lock the QoS profile map.
    std::map<std::string, TopicQoS> topicQoSMapTmp_;// The map to store the temporary QoS profiles by the topic name.
    std::mutex topicQoSMapTmpMutex_;// The mutex to lock the temporary QoS profile map.
    std::atomic<uint64_t> qid_;// The unique ID for whole QoS profiles.
    std::map<std::string, std::pair<bool, bool> > topicQoSUpdateMap_;// The map to store the topic QoS update status: <topicName, <pub, sub>>.
    std::mutex topicQoSUpdateMapMutex_;// The mutex to lock the topic QoS update status map.

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
     * QoS profile registration and requisition service.
     */

    /**
     * @brief QoS registration service callback function.
     * @details The function is called when the QoS registration service receives a request.
     * @note The function is using topicQoSMapMutex_ and topicQoSMapTmpMutex_.
     */
    void _qosRegServiceCbFunc(const std::shared_ptr<vehicle_interfaces::srv::QosReg::Request> request, 
                                std::shared_ptr<vehicle_interfaces::srv::QosReg::Response> response)
    {
        std::lock_guard<std::mutex> qosRegServiceLock(this->qosRegServiceMutex_);
        // Invalid request.
        if (request->topic_name == "" && !request->save_qmap && !request->clear_profiles)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Request: %s [%s] failed: invalid request.", request->topic_name.c_str(), request->qos_type.c_str());
            response->response = false;
            response->qid = this->qid_.load();
            return;
        }

        // Invalid topic type.
        if (request->qos_type != "publisher" && request->qos_type != "subscription" && request->qos_type != "both")
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Request: %s [%s] failed: invalid qos_type.", request->topic_name.c_str(), request->qos_type.c_str());
            response->response = false;
            response->qid = this->qid_.load();
            return;
        }

        response->response = true;

        if (request->save_qmap)// Save temporary topic QoS profile.
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Save qmap request.");
            auto result = this->setTopicQoSMap();
            response->response = result.result;
            if (!response->response)
            {
                response->reason = result.reason;
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Save qmap failed.");
            }
        }
        else if (request->clear_profiles)// Recover whole temporary QoS profiles.
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Recover tmp qmap.");
            this->recoverTmpQoSProfile();
        }
        else if (request->remove_profile)// Remove single temporary QoS profile.
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Request: remove %s.", 
                request->topic_name.c_str());
            this->removeTmpQoSProfile(request->topic_name);
        }
        else// Set temporary QoS profile.
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Request: set %s (%-2d/%-2d/%-2d) [%s].", 
                request->topic_name.c_str(), request->qos_profile.history, request->qos_profile.depth, request->qos_profile.reliability, request->qos_type.c_str());
            response->response = this->setTmpQoSProfile(request->topic_name, request->qos_type, CvtMsgToRMWQoS(request->qos_profile));
            if (!response->response)
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosRegServiceCbFunc] Request: set %s [%s] failed.", request->topic_name.c_str(), request->qos_type.c_str());
        }
        response->qid = this->qid_.load();
        return;
    }

    /**
     * @brief Response the QoS profile request.
     * @details The function responses the QoS profile by the topic name and the type "publisher" or "subscription".
     * @note The function is using topicQoSMapMutex_.
     */
    void _qosReqServiceCbFunc(const std::shared_ptr<vehicle_interfaces::srv::QosReq::Request> request, 
                                std::shared_ptr<vehicle_interfaces::srv::QosReq::Response> response)
    {
        std::lock_guard<std::mutex> qosReqServiceLock(this->qosReqServiceMutex_);
        // Invalid request.
        if (request->topic_name == "")
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosReqServiceCbFunc] Request: %s [%s] failed: invalid request.", request->topic_name.c_str(), request->qos_type.c_str());
            response->response = false;
            response->qid = this->qid_.load();
            response->reason = TopicQoSExceptionMsg[TopicQoSException::QOS_EXC_INVALID_TOPIC_NAME];
            return;
        }

        // Invalid topic type.
        if (request->qos_type != "publisher" && request->qos_type != "subscription" && request->qos_type != "both")
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosReqServiceCbFunc] Request: %s [%s] failed: invalid qos_type.", request->topic_name.c_str(), request->qos_type.c_str());
            response->response = false;
            response->qid = this->qid_.load();
            response->reason = TopicQoSExceptionMsg[TopicQoSException::QOS_EXC_INVALID_TOPIC_TYPE];
            return;
        }

        response->response = true;

        // Safe copy of topicQoSMap_.
        auto topicQoSMapCopy = this->_safeCall(&this->topicQoSMap_, this->topicQoSMapMutex_);
        try
        {
            if (request->topic_name == "all")
            {
                for (const auto& [topicName, tQoS] : topicQoSMapCopy)
                {
                    if (tQoS.isPubQoSValid() && (request->qos_type == "publisher" || request->qos_type == "both"))
                    {
                        response->topic_name_vec.push_back(topicName);
                        response->qos_type_vec.push_back("publisher");
                        response->qos_profile_vec.push_back(CvtRMWQoSToMsg(tQoS["publisher"]));
                    }
                    if (tQoS.isSubQoSValid() && (request->qos_type == "subscription" || request->qos_type == "both"))
                    {
                        response->topic_name_vec.push_back(topicName);
                        response->qos_type_vec.push_back("subscription");
                        response->qos_profile_vec.push_back(CvtRMWQoSToMsg(tQoS["subscription"]));
                    }
                }
            }
            else
            {
                if (topicQoSMapCopy.find(request->topic_name) == topicQoSMapCopy.end())
                    throw TopicQoSException::QOS_EXC_INVALID_TOPIC_NAME;
                auto msg = vehicle_interfaces::msg::QosProfile();
                TopicQoS qos = topicQoSMapCopy[request->topic_name];
                if (request->qos_type == "publisher" || request->qos_type == "both")
                {
                    msg = CvtRMWQoSToMsg(qos["publisher"]);
                    response->topic_name_vec.push_back(request->topic_name);
                    response->qos_type_vec.push_back("publisher");
                    response->qos_profile_vec.push_back(msg);
                }
                if (request->qos_type == "subscription" || request->qos_type == "both")
                {
                    msg = CvtRMWQoSToMsg(qos["subscription"]);
                    response->topic_name_vec.push_back(request->topic_name);
                    response->qos_type_vec.push_back("subscription");
                    response->qos_profile_vec.push_back(msg);
                }

            }
            response->qid = this->qid_.load();
        }
        catch (const TopicQoSException& e)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::_qosReqServiceCbFunc] Request: %s [%s] failed: %s.", 
                request->topic_name.c_str(), 
                request->qos_type.c_str(), 
                TopicQoSExceptionMsg[e]);
            response->response = false;
            response->qid = this->qid_.load();
            response->reason = TopicQoSExceptionMsg[e];
            return;
        }
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_qosReqServiceCbFunc] Response: qid: %-4d %s [%s] (found: %s).", 
            response->qid, request->topic_name.c_str(), request->qos_type.c_str(), response->response ? "true" : "false");
    }

    /**
     * Topic device management.
     */

    /**
     * @brief Topic device information registration service callback function.
     * @details The function is called when the QoSNode sends the topic device information registration request.
     * @note The function is using topicDevInfoMapMutex_.
     */
    void _topicDevInfoRegServiceCbFunc(const std::shared_ptr<vehicle_interfaces::srv::TopicDeviceInfoReg::Request> request, 
                                        std::shared_ptr<vehicle_interfaces::srv::TopicDeviceInfoReg::Response> response)
    {
        std::lock_guard<std::mutex> topicDevInfoRegServiceLock(this->topicDevInfoRegServiceMutex_);
        std::lock_guard<std::mutex> topicDevInfoMapLock(this->topicDevInfoMapMutex_);
        this->topicDevInfoMap_[request->request.node_name] = request->request;
        RCLCPP_INFO(this->get_logger(), "[QoSServer::_topicDevInfoRegServiceCbFunc] %-30s: %-20s[%-12s] registered.", 
            request->request.node_name.c_str(), 
            request->request.topic_name.c_str(), 
            request->request.device_type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_PUBLISHER ? "publisher" : 
            (request->request.device_type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_SUBSCRIPTION ? "subscription" : "unknown"));
        response->response = true;
    }

    /**
     * @brief Timer callback function for registered topic device information publishing.
     * @details The function is called when the topicDevInfoRegPubTimer_ is triggered.
     * @note The function is using topicDevInfoMapMutex_.
     */
    void _topicDevInfoRegPubTimerCbFunc()
    {
        static int cnt = this->topicDevInfoRegPubCnt_;
        if (cnt-- <= 0)
        {
            this->topicDevInfoRegPubTimer_->stop();
            cnt = this->topicDevInfoRegPubCnt_;
            return;
        }
        auto tmp = this->_safeCall(&this->topicDevInfoMap_, this->topicDevInfoMapMutex_);
        vehicle_interfaces::msg::IDTable msg;
        for (const auto& [nodeName, tInfo] : tmp)
            msg.idtable.push_back(nodeName);
        this->topicDevInfoRegPub_->publish(msg);
    }


    /**
     * InteractiveNode management.
     */

    /**
     * @brief Send request to the InteractiveNode.
     * @details The function sends the request to the InteractiveNode.
     * @param[in] nodeName The name of the InteractiveNode.
     * @param[in] request The request to send.
     * @return True if the request is sent successfully; false otherwise.
     */
    bool _sendInteractiveNodeRequest(const std::string& nodeName, const std::shared_ptr<vehicle_interfaces::srv::InteractiveNode::Request> request)
    {
        auto node = std::make_shared<rclcpp::Node>(this->serviceName_ + "_client");
        auto client = node->create_client<vehicle_interfaces::srv::InteractiveNode>(nodeName);
        auto result = client->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
                return true;
            else
                RCLCPP_WARN(this->get_logger(), "[QoSNode::_sendInteractiveNodeRequest] Failed to send request to %s: %s.", nodeName.c_str(), response->reason.c_str());
        }
        return false;
    }

    /**
     * @brief Send target alive signal to all InteractiveNodes.
     * @details The function sends the target alive signal to all InteractiveNodes.
     * @param[in] devQue The deque of the topic device information.
     * @param[in] status The target alive status.
     * @return True if all target alive signal are sent successfully; false otherwise.
     * @note The function will stop if any target alive signal is sent failed.
     */
    bool _sendAllInteracticeNodeTargetAliveStatus(const std::deque<vehicle_interfaces::msg::TopicDeviceInfo> &devQue, uint8_t status)
    {
        for (const auto& tInfo : devQue)
        {
            auto request = std::make_shared<vehicle_interfaces::srv::InteractiveNode::Request>();
            request->device_id = this->serviceName_;
            request->interactive_node.target_alive = status;
            if (this->_sendInteractiveNodeRequest(tInfo.node_name, request))
                RCLCPP_INFO(this->get_logger(), "[QoSServer::_sendAllInteracticeNodeTargetAliveStatus] Send target alive signal [%d] to %s done.", status, tInfo.node_name.c_str());
            else
            {
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_sendAllInteracticeNodeTargetAliveStatus] Send target alive signal [%d] to %s failed.", status, tInfo.node_name.c_str());
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Update QoS profile for the topic devices.
     * @details The function updates the QoS profile for the topic devices.
     * @return True if the QoS profile is updated successfully; false otherwise.
     * @note The functin will called InteractiveNode directly to update the QoS profile for the publisher and subscriber.
     * @note The function is using topicDevInfoMapMutex_, topicQoSMapMutex_ and topicQoSUpdateMapMutex_.
     */
    bool _updateInteractiveNodeQoS()
    {
        // Get topic device map.
        auto topicDevInfoMapCopy = this->_safeCall(&this->topicDevInfoMap_, this->topicDevInfoMapMutex_);
        // Get Current QoS map.
        auto topicQoSMapCopy = this->_safeCall(&this->topicQoSMap_, this->topicQoSMapMutex_);
        // Get update map.
        auto topicQoSUpdateMapCopy = this->_safeCall(&this->topicQoSUpdateMap_, this->topicQoSUpdateMapMutex_);

        // Group topic devices by QoS profile.
        std::map<TopicQoS, std::deque<vehicle_interfaces::msg::TopicDeviceInfo> > qosDevMap;
        for (const auto& [nodeName, tInfo] : topicDevInfoMapCopy)// Registered topic devices.
        {
            if (topicQoSUpdateMapCopy.find(tInfo.topic_name) == topicQoSUpdateMapCopy.end())// Topic no need to update.
                continue;
            qosDevMap[topicQoSMapCopy[tInfo.topic_name]].push_back(tInfo);// Add the topic device under the QoS profile.
        }

        RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Update QoS profile for the topic devices.");
        vehicle_interfaces::HierarchicalPrint hprint;
        for (const auto& [tQoS, tInfoQue] : qosDevMap)
        {
            hprint.push(0, "[Topic: %s]", tQoS.getTopicName().c_str());
            for (const auto& tInfo : tInfoQue)
                hprint.push(1, "%s [%s]", tInfo.node_name.c_str(), tInfo.device_type == vehicle_interfaces::msg::TopicDeviceInfo::DEVICE_TYPE_PUBLISHER ? "publisher" : "subscription");
        }
        hprint.print();
        hprint.clear();

        // Start to update QoS profile for the topic devices.
        for (const auto& [qos, devQue] : qosDevMap)// Each QoS profile and its devices.
        {
            // Send target alive disable signal to the InteractiveNode to close the publisher and subscriber.
            if (!this->_sendAllInteracticeNodeTargetAliveStatus(devQue, vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE))
            {
                // Send target alive disable signal failed, recover all nodes.
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Send target alive disable signal to all nodes of topic %s failed.", qos.getTopicName().c_str());
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Recover all nodes of topic %s.", qos.getTopicName().c_str());
                this->_sendAllInteracticeNodeTargetAliveStatus(devQue, vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE);
                continue;// Ignore rest of the process and continue to the next QoS profile.
            }
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Send target alive disable signal to all nodes of topic %s done.", qos.getTopicName().c_str());

            //Check whether all publishers and subscribers are closed.
            std::chrono::duration<double, std::milli> checkTopicNodeTimeout(500);// timeout 500ms.
            auto checkTopicNodeStart = std::chrono::high_resolution_clock::now();
            bool allClosed = false;
            while (std::chrono::high_resolution_clock::now() - checkTopicNodeStart < checkTopicNodeTimeout)
            {
                if (this->get_publishers_info_by_topic(qos.getTopicName()).size() <= 0 && 
                    this->get_subscriptions_info_by_topic(qos.getTopicName()).size() <= 0)
                {
                    allClosed = true;
                    break;
                }
                std::this_thread::sleep_for(10ms);
            }
            // If not all nodes are closed, send all nodes the target alive enable signal to open the publisher and subscriber, then continue to the next QoS profile.
            if (!allClosed)
            {
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Check topic %s timeout. Not all nodes are closed.", qos.getTopicName().c_str());
                this->_sendAllInteracticeNodeTargetAliveStatus(devQue, vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE);
                continue;// Ignore rest of the process and continue to the next QoS profile.
            }
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Check topic %s done. All nodes are closed.", qos.getTopicName().c_str());

            // Send node command to the InteractiveNode to update QoS profile for the publisher and subscriber.
            for (const auto& tInfo : devQue)
            {
                auto request = std::make_shared<vehicle_interfaces::srv::InteractiveNode::Request>();
                request->device_id = this->serviceName_;
                request->node_command_name = "qos_update";
                request->node_command_args = CvtRMWQoSToInteractiveNodeCommandArgs(qos[tInfo.device_type]);
                if (!this->_sendInteractiveNodeRequest(tInfo.node_name, request))
                    RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Send %s command to %s failed.", request->node_command_name.c_str(), tInfo.node_name.c_str());
                else
                    RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Send %s command to %s done.", request->node_command_name.c_str(), tInfo.node_name.c_str());
            }

            // Send target alive enable signal to the InteractiveNode to open the publisher and subscriber.
            if (!this->_sendAllInteracticeNodeTargetAliveStatus(devQue, vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE))
            {
                RCLCPP_WARN(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Send target alive enable signal to all nodes of topic %s failed.", qos.getTopicName().c_str());
                continue;// Ignore rest of the process and continue to the next QoS profile.
            }
            RCLCPP_INFO(this->get_logger(), "[QoSServer::_updateInteractiveNodeQoS] Send target alive enable signal to all nodes of topic %s done.", qos.getTopicName().c_str());
        }
    }


    /**
     * Temporary QoS profile management.
     */

    /**
     * @brief Check temporary QoS profile update status.
     * @details The function checks the temporary QoS profile with the current QoS profile.
     * @param[out] updateMap The map to store the update status: <topicName, <pub, sub>>.
     * @return True if the temporary QoS profile is different from the current QoS profile; false otherwise.
     * @note The function is using topicQoSMapMutex_ and topicQoSMapTmpMutex_.
     * @note The function will throw TopicQoSException if the temporary QoS profile is invalid.
     */
    bool _topicQoSUpdateCheck(std::map<std::string, std::pair<bool, bool> >& updateMap)// <topicName, <pub, sub>>
    {
        updateMap.clear();
        auto topicQoSMapCopy = this->_safeCall(&this->topicQoSMap_, this->topicQoSMapMutex_);
        auto topicQoSMapTmpCopy = this->_safeCall(&this->topicQoSMapTmp_, this->topicQoSMapTmpMutex_);

        bool needUpdateF = false;
        needUpdateF |= topicQoSMapCopy.size() != topicQoSMapTmpCopy.size();// True if different sizes
        for (auto& [topicName, topicProp] : topicQoSMapTmpCopy)
        {
            if (!topicProp)// QoSTmp has invalid QoS profile.
                throw topicName + " has invalid QoS profile.";

            if (topicQoSMapCopy.find(topicName) == topicQoSMapCopy.end())// QoSTmp has new QoS profile.
            {
                updateMap[topicName] = { true, true };
                continue;
            }
            auto cmp = topicProp % topicQoSMapCopy[topicName];
            switch (cmp)
            {
                case TopicQoSCompareResult::QOS_CMP_SAME:// Same QoS profile won't added to updateMap.
                    // updateMap[topicName] = { false, false };
                    break;
                case TopicQoSCompareResult::QOS_CMP_PUB_DIFF:
                    updateMap[topicName] = { true, false };
                    break;
                case TopicQoSCompareResult::QOS_CMP_SUB_DIFF:
                    updateMap[topicName] = { false, true };
                    break;
                case TopicQoSCompareResult::QOS_CMP_BOTH_DIFF:
                    updateMap[topicName] = { true, true };
                    break;
                case TopicQoSCompareResult::QOS_CMP_TOPIC_DIFF:// It is weird if this happens.
                    updateMap[topicName] = { true, true };
                    break;
            }
        }
        needUpdateF |= updateMap.size() > 0;
        return needUpdateF;
    }

public:
    QoSServer(const std::string nodeName, const std::string& qosServiceName, const std::string& qosFilePath) : 
        rclcpp::Node(nodeName), 
        serviceName_(qosServiceName), 
        qosRegServiceName_(qosServiceName + "_Reg"), 
        qosReqServiceName_(qosServiceName + "_Req"), 
        topicDevInfoRegServiceName_(qosServiceName), 
        qosDirPath_(qosFilePath), 
        topicDevInfoRegPubTimer_(nullptr), 
        topicDevInfoRegPubCnt_(10), 
        qid_(0)
    {
        {// Check QoS directory
            char buf[512];
            sprintf(buf, "mkdir -p %s", this->qosDirPath_.parent_path().generic_string().c_str());
            system(buf);
        }

        {// Load QoS profiles from the file.
            if (this->loadQmapFromJSON(true))
            {
                RCLCPP_INFO(this->get_logger(), "[QoSServer] Load QoS profiles from %s.", this->qosDirPath_.generic_string().c_str());
                auto tmp = this->_safeCall(&this->topicQoSMap_, this->topicQoSMapMutex_);
                printf("Qmap found, size: %d, qid: %d\n", tmp.size(), this->qid_.load());
                for (const auto& [topicName, topicQoS] : tmp)
                {
                    if (topicQoS.isPubQoSValid())
                    {
                        auto v = topicQoS["publisher"];
                        printf("%s [publisher] (%02d/%02d/%02d)\n", topicName.c_str(), v.history, v.depth, v.reliability);
                    }
                    if (topicQoS.isSubQoSValid())
                    {
                        auto v = topicQoS["subscription"];
                        printf("%s [subscription] (%02d/%02d/%02d)\n", topicName.c_str(), v.history, v.depth, v.reliability);
                    }
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[QoSServer] Load QoS profiles failed: %s.", this->qosDirPath_.generic_string().c_str());
            }
        }

        // QoS registration and request services.
        this->qosRegService_ = this->create_service<vehicle_interfaces::srv::QosReg>(this->qosRegServiceName_, std::bind(&QoSServer::_qosRegServiceCbFunc, this, std::placeholders::_1, std::placeholders::_2));
        this->qosReqService_ = this->create_service<vehicle_interfaces::srv::QosReq>(this->qosReqServiceName_, std::bind(&QoSServer::_qosReqServiceCbFunc, this, std::placeholders::_1, std::placeholders::_2));

        // Topic device information registration service.
        this->topicDevInfoRegService_ = this->create_service<vehicle_interfaces::srv::TopicDeviceInfoReg>(qosServiceName, std::bind(&QoSServer::_topicDevInfoRegServiceCbFunc, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher to publish the registered topic device information.
        this->topicDevInfoRegPub_ = this->create_publisher<vehicle_interfaces::msg::IDTable>(qosServiceName, 10);
        this->topicDevInfoRegPubTimer_ = new vehicle_interfaces::Timer(1000, std::bind(&QoSServer::_topicDevInfoRegPubTimerCbFunc, this));
        this->topicDevInfoRegPubTimer_->start();

        RCLCPP_INFO(this->get_logger(), "[QoSServer] Constructed.");
    }

    ~QoSServer()
    {
        if (this->topicDevInfoRegPubTimer_ != nullptr)
        {
            this->topicDevInfoRegPubTimer_->stop();
            delete this->topicDevInfoRegPubTimer_;
        }
    }

    /**
     * @brief Set the QoS profile to the temporary QoS profile map.
     * @details The function sets the QoS profile to the temporary QoS profile map by the topic name and the node type.
     * @param[in] topicName The name of the topic.
     * @param[in] qosType The type of the qos profile: "publisher", "subscription", or "both".
     * @param[in] prof The QoS profile.
     * @return True if the QoS profile is set; false otherwise.
     * @note The function is using topicQoSMapTmpMutex_.
     */
    bool setTmpQoSProfile(const std::string& topicName, const std::string& qosType, const rmw_qos_profile_t& prof)
    {
        // Invalid topic name or qos type.
        if (topicName == "" || (qosType != "publisher" && qosType != "subscription" && qosType != "both"))
            return false;
        std::lock_guard<std::mutex> topicQoSMapTmpLock(this->topicQoSMapTmpMutex_);
        if (this->topicQoSMapTmp_.find(topicName) == this->topicQoSMapTmp_.end())
            this->topicQoSMapTmp_[topicName] = TopicQoS(topicName);
        if (qosType == "publisher")
            this->topicQoSMapTmp_[topicName].setPubQoS(prof);
        else if (qosType == "subscription")
            this->topicQoSMapTmp_[topicName].setSubQoS(prof);
        else if (qosType == "both")
            this->topicQoSMapTmp_[topicName].setQoS(prof);
        return true;
    }

    /**
     * @brief Remove the temporary QoS profile by the topic name, including both publisher and subscriber QoS profiles.
     * @details The function removes the temporary QoS profile by the topic name, including both publisher and subscriber QoS profiles.
     * @param[in] topicName The name of the topic.
     * @note The function is using topicQoSMapTmpMutex_.
     */
    void removeTmpQoSProfile(const std::string& topicName)
    {
        std::lock_guard<std::mutex> topicQoSMapTmpLock(this->topicQoSMapTmpMutex_);
        this->topicQoSMapTmp_.erase(topicName);
    }

    /**
     * @brief Clear the temporary QoS profile map.
     * @details The function clears the temporary QoS profile map.
     * @note The function is using topicQoSMapTmpMutex_.
     */
    void clearTmpQoSProfile()
    {
        std::lock_guard<std::mutex> topicQoSMapTmpLock(this->topicQoSMapTmpMutex_);
        this->topicQoSMapTmp_.clear();
    }

    /**
     * @brief Recover the temporary QoS profile map.
     * @details The function recovers the temporary QoS profile map from the QoS profile map.
     * @note The function is using topicQoSMapMutex_ and topicQoSMapTmpMutex_.
     */
    void recoverTmpQoSProfile()
    {
        std::lock_guard<std::mutex> topicQoSMapLock(this->topicQoSMapMutex_);
        std::lock_guard<std::mutex> topicQoSMapTmpLock(this->topicQoSMapTmpMutex_);
        this->topicQoSMapTmp_ = this->topicQoSMap_;
    }

    /**
     * @brief Set the temporary QoS profile map to the QoS profile map and dump the QoS profile map to the file.
     * @details The function sets the temporary QoS profile map to the QoS profile map and dumps the QoS profile map to the file.
     * @return True if the QoS profile map is set; false otherwise.
     * @note The function is using topicQoSMapMutex_, topicQoSMapTmpMutex_, topicQoSUpdateMapMutex_ and topicDevInfoMapMutex_.
     */
    vehicle_interfaces::ReasonResult<bool> setTopicQoSMap()
    {
        std::map<std::string, std::pair<bool, bool> > updateMap;
        bool needUpdate = false;
        try
        {
            needUpdate = this->_topicQoSUpdateCheck(updateMap);
        }
        catch (const TopicQoSException& e)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::setTopicQoSMap] Temporary QoS check failed: %s.", TopicQoSExceptionMsg[e]);
            return { false, TopicQoSExceptionMsg[e] };
        }
        catch (const std::string& e)
        {
            RCLCPP_WARN(this->get_logger(), "[QoSServer::setTopicQoSMap] Temporary QoS check failed: %s.", e.c_str());
            return { false, e };
        }

        if (needUpdate)
        {
            RCLCPP_INFO(this->get_logger(), "[QoSServer::setTopicQoSMap] QoS need updated!");
            for (const auto& [topicName, update] : updateMap)
                RCLCPP_INFO(this->get_logger(), "[QoSServer::setTopicQoSMap] %-30s: %-9s %-12s.", topicName.c_str(), update.first ? "publisher" : "", update.second ? "subscription" : "");
            // Lock both topicQoSMap_ and topicQoSMapTmp_.
            std::unique_lock<std::mutex> topicQoSMapLock(this->topicQoSMapMutex_, std::defer_lock);
            std::unique_lock<std::mutex> topicQoSMapTmpLock(this->topicQoSMapTmpMutex_, std::defer_lock);
            topicQoSMapLock.lock();
            topicQoSMapTmpLock.lock();
            this->topicQoSMap_ = this->topicQoSMapTmp_;
            this->qid_ += 1;
            this->_safeSave(&this->topicQoSUpdateMap_, updateMap, this->topicQoSUpdateMapMutex_);
            // Unlock both topicQoSMap_ and topicQoSMapTmp_ then dump the file.
            topicQoSMapTmpLock.unlock();
            topicQoSMapLock.unlock();
            this->dumpQmapToJSON();
            this->_updateInteractiveNodeQoS();
        }
        return true;
    }

    /**
     * @brief Load QoS profiles to the temporary QoS profile map from the file.
     * @details The function loads the QoS profiles to the temporary QoS profile map from the file.
     * @param[in] forceWrite Set true to force write both temporary and current QoS profile maps to the file.
     * @return True if the QoS profile is loaded; false otherwise.
     * @note If load failed, the temporary QoS profile map will not be modified.
     * @note The function is using topicQoSMapTmpMutex_.
     */
    bool loadQmapFromJSON(bool forceWrite = false)
    {
        try
        {
            nlohmann::json json;
            json.update(nlohmann::json::parse(std::ifstream(this->qosDirPath_)));
            std::map<std::string, TopicQoS> tmpMap;

            for (const auto& [topicName, topicQoS] : json.items())
            {
                for (const auto& [type, qos] : topicQoS.items())
                {
                    rmw_qos_profile_t tmpProf;
#if ROS_DISTRO < 2// Foxy and older
                    tmpProf.history = (rmw_qos_history_policy_t)prof["history"];
                    tmpProf.depth = prof["depth"];
                    tmpProf.reliability = (rmw_qos_reliability_policy_t)prof["reliability"];
                    tmpProf.durability = (rmw_qos_durability_policy_t)prof["durability"];
                    tmpProf.deadline = CvtMsgToRMWTime(prof["deadline_ms"]);
                    tmpProf.lifespan = CvtMsgToRMWTime(prof["lifespan_ms"]);
                    tmpProf.liveliness = (rmw_qos_liveliness_policy_t)prof["liveliness"];
                    tmpProf.liveliness_lease_duration = CvtMsgToRMWTime(prof["liveliness_lease_duration_ms"]);
#else
                    tmpProf.history = (rmw_qos_history_policy_e)qos["history"];
                    tmpProf.depth = qos["depth"];
                    tmpProf.reliability = (rmw_qos_reliability_policy_e)qos["reliability"];
                    tmpProf.durability = (rmw_qos_durability_policy_e)qos["durability"];
                    tmpProf.deadline = CvtMsgToRMWTime(qos["deadline_ms"]);
                    tmpProf.lifespan = CvtMsgToRMWTime(qos["lifespan_ms"]);
                    tmpProf.liveliness = (rmw_qos_liveliness_policy_e)qos["liveliness"];
                    tmpProf.liveliness_lease_duration = CvtMsgToRMWTime(qos["liveliness_lease_duration_ms"]);
#endif
                    tmpMap[topicName].setTopicName(topicName);
                    if (type == "publisher")
                        tmpMap[topicName].setPubQoS(tmpProf);
                    else if (type == "subscription")
                        tmpMap[topicName].setSubQoS(tmpProf);
                }
            }
            std::lock_guard<std::mutex> topicQoSMapTmpLock(this->topicQoSMapTmpMutex_);
            this->topicQoSMapTmp_ = tmpMap;
            if (forceWrite)
            {
                std::lock_guard<std::mutex> topicQoSMapLock(this->topicQoSMapMutex_);
                this->topicQoSMap_ = tmpMap;
            }
        }
        catch (...)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief Dump the QoS profiles map to the file.
     * @details The function dumps the QoS profile map to the file.
     * @return True if the QoS profile map is dumped; false otherwise.
     * @note The function is using topicQoSMapMutex_.
     */
    bool dumpQmapToJSON()
    {
        auto topicQoSMapCopy = this->_safeCall(&this->topicQoSMap_, this->topicQoSMapMutex_);
        try
        {
            nlohmann::json json;
            for (auto& [topicName, topicProp] : topicQoSMapCopy)
            {
                if (topicProp.getTopicName() == "")
                    continue;
                if (topicProp.isPubQoSValid())
                {
                    auto& prof = topicProp["publisher"];
                    json[topicProp.getTopicName()]["publisher"]["history"] = (int8_t)prof.history;
                    json[topicProp.getTopicName()]["publisher"]["depth"] = prof.depth;
                    json[topicProp.getTopicName()]["publisher"]["reliability"] = (int8_t)prof.reliability;
                    json[topicProp.getTopicName()]["publisher"]["durability"] = (int8_t)prof.durability;
                    json[topicProp.getTopicName()]["publisher"]["deadline_ms"] = CvtRMWTimeToMsg(prof.deadline);
                    json[topicProp.getTopicName()]["publisher"]["lifespan_ms"] = CvtRMWTimeToMsg(prof.lifespan);
                    json[topicProp.getTopicName()]["publisher"]["liveliness"] = (int8_t)prof.liveliness;
                    json[topicProp.getTopicName()]["publisher"]["liveliness_lease_duration_ms"] = CvtRMWTimeToMsg(prof.liveliness_lease_duration);
                }
                if (topicProp.isSubQoSValid())
                {
                    auto& prof = topicProp["subscription"];
                    json[topicProp.getTopicName()]["subscription"]["history"] = (int8_t)prof.history;
                    json[topicProp.getTopicName()]["subscription"]["depth"] = prof.depth;
                    json[topicProp.getTopicName()]["subscription"]["reliability"] = (int8_t)prof.reliability;
                    json[topicProp.getTopicName()]["subscription"]["durability"] = (int8_t)prof.durability;
                    json[topicProp.getTopicName()]["subscription"]["deadline_ms"] = CvtRMWTimeToMsg(prof.deadline);
                    json[topicProp.getTopicName()]["subscription"]["lifespan_ms"] = CvtRMWTimeToMsg(prof.lifespan);
                    json[topicProp.getTopicName()]["subscription"]["liveliness"] = (int8_t)prof.liveliness;
                    json[topicProp.getTopicName()]["subscription"]["liveliness_lease_duration_ms"] = CvtRMWTimeToMsg(prof.liveliness_lease_duration);
                }
            }
            std::ofstream outFile(this->qosDirPath_);
            outFile << json;
        }
        catch (...)
        {
            return false;
        }
        return true;
    }
};

}