#pragma once
#include "interactive_node.h"

/**
 * TODO: subscriptions, mutex notes.
 */

namespace vehicle_interfaces
{

enum InteractiveTopicType
{
    INTERACTIVE_TOPIC_UNKNOWN = 0,
    INTERACTIVE_TOPIC_PUBLISHER,
    INTERACTIVE_TOPIC_SUBSCRIPTION
};



/**
 * Interactive Methods.
 */

/**
 * @brief InteractiveTopicNode inherits from InteractiveNode, and is the base class of InteractivePublisher and InteractiveSubscription.
 * @details The class provides the following functionalities:
 * 1. Set and get Quality of Service.
 * 2. Get topic name and topic type.
 */
class InteractiveTopicNode : public InteractiveNode
{
private:
    const std::string topicName_;// Name of the topic.
    const InteractiveTopicType topicType_;// Type of the topic.
    std::shared_ptr<rclcpp::QoS> qos_;// Quality of Service.
    std::mutex qosLock_;// Lock qos_.

public:
    InteractiveTopicNode(const std::string& nodeName, 
                            const std::string& topicName, 
                            const rclcpp::QoS& qos, 
                            InteractiveTopicType type, 
                            InteractiveNodeInitProp prop) : 
        InteractiveNode(nodeName, prop), 
        rclcpp::Node(nodeName), 
        topicName_(topicName), 
        topicType_(type) { this->setQoS(qos); }

    /**
     * @brief Set Quality of Service.
     * @param[in] qos Quality of Service.
     * @note This function is using qosLock_.
     */
    void setQoS(const rclcpp::QoS& qos)
    {
        std::lock_guard<std::mutex> lock(this->qosLock_);
        this->qos_ = std::make_shared<rclcpp::QoS>(qos);
    }

    /**
     * @brief Get Quality of Service.
     * @return Quality of Service.
     * @note This function is using qosLock_.
     */
    rclcpp::QoS getQoS()
    {
        std::lock_guard<std::mutex> lock(this->qosLock_);
        return *this->qos_;
    }

    std::string getTopicName() const
    {
        return this->topicName_;
    }

    InteractiveTopicType getTopicType() const
    {
        return this->topicType_;
    }
};







/**
 * MultiInteractive Methods.
 */

/**
 * @brief MultiInteractiveTopicNode inherits from MultiInteractiveNode, and is the base class of MultiInteractivePublisher and MultiInteractiveSubscription.
 * @details The class provides the following functionalities:
 * 1. Set and get Quality of Service.
 * 2. Get topic name and topic type.
 * 3. Check all master devices' target alive and target activity status.
 * 4. Set target alive enable and disable event handlers. Which event will be raised was determined by checkAllTargetAlive().
 * 5. Set target activity enable and disable event handlers. Which event will be raised was determined by checkAllTargetActivity().
 */
class MultiInteractiveTopicNode : public MultiInteractiveNode
{
private:
    const std::string topicName_;// Name of the topic.
    const InteractiveTopicType topicType_;// Type of the topic.
    std::shared_ptr<rclcpp::QoS> qos_;// Quality of Service.
    std::mutex qosLock_;// Lock qos_.

    std::function<void(void)> targetAliveEnableCbFunc_;// Target activity enable callback function.
    std::mutex targetAliveEnableCbFuncLock_;// Lock targetAliveEnableCbFunc_.
    std::atomic<bool> targetAliveEnableCbFuncF_;// Flag indicating whether the target activity enable callback function is set.
    std::function<void(void)> targetAliveDisableCbFunc_;// Target activity disable callback function.
    std::mutex targetAliveDisableCbFuncLock_;// Lock targetAliveDisableCbFunc_.
    std::atomic<bool> targetAliveDisableCbFuncF_;// Flag indicating whether the target activity disable callback function is set.

    std::function<void(void)> targetActivityEnableCbFunc_;// Target activity enable callback function.
    std::mutex targetActivityEnableCbFuncLock_;// Lock targetActivityEnableCbFunc_.
    std::atomic<bool> targetActivityEnableCbFuncF_;// Flag indicating whether the target activity enable callback function is set.
    std::function<void(void)> targetActivityDisableCbFunc_;// Target activity disable callback function.
    std::mutex targetActivityDisableCbFuncLock_;// Lock targetActivityDisableCbFunc_.
    std::atomic<bool> targetActivityDisableCbFuncF_;// Flag indicating whether the target activity disable callback function is set.

private:
    /**
     * @brief Target alive event handler.
     * @details The function is called when the interactive service server receives a target alive request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetAlive Requested target alive status.
     * @return For MultiInteractivePublisher, the function returns false.
     * @note This function is using targetAliveMapLock_, targetAliveEnableCbFuncLock_ and targetAliveDisableCbFuncLock_.
     */
    bool _targetAliveEventHandler(const std::string deviceID, const uint8_t targetAlive)
    {
        this->modifyTargetAlive(deviceID, targetAlive);
        uint8_t ret = this->checkAllTargetAlive();
        this->setTargetAlive(ret);

        std::lock_guard<std::mutex> targetAliveEnableCbFuncLocker(this->targetAliveEnableCbFuncLock_);
        std::lock_guard<std::mutex> targetAliveDisableCbFuncLocker(this->targetAliveDisableCbFuncLock_);

        if (this->targetAliveEnableCbFuncF_ && ret == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
            this->targetAliveEnableCbFunc_();
        else if (this->targetAliveDisableCbFuncF_ && ret == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE)
            this->targetAliveDisableCbFunc_();
        return false;
    }

    /**
     * @brief Target activity event handler.
     * @details The function is called when the interactive service server receives a target activity request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetActivity Requested target activity status.
     * @return For MultiInteractivePublisher, the function returns false.
     * @note This function is using targetActivityMapLock_, targetActivityEnableCbFuncLock_ and targetActivityDisableCbFuncLock_.
     */
    bool _targetActivityEventHandler(const std::string deviceID, const uint8_t targetActivity)
    {
        this->modifyTargetActivity(deviceID, targetActivity);
        uint8_t ret = this->checkAllTargetActivity();
        this->setTargetActivity(ret);

        std::lock_guard<std::mutex> targetActivityEnableCbFuncLocker(this->targetActivityEnableCbFuncLock_);
        std::lock_guard<std::mutex> targetActivityDisableCbFuncLocker(this->targetActivityDisableCbFuncLock_);

        if (this->targetActivityEnableCbFuncF_ && ret == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE)
            this->targetActivityEnableCbFunc_();
        else if (this->targetActivityDisableCbFuncF_ && ret == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_DISABLE)
            this->targetActivityDisableCbFunc_();
        return false;
    }

public:
    MultiInteractiveTopicNode(const std::string& nodeName, 
                                const std::string& topicName, 
                                const rclcpp::QoS& qos, 
                                InteractiveTopicType type, 
                                InteractiveNodeInitProp prop) : 
        MultiInteractiveNode(nodeName, prop), 
        rclcpp::Node(nodeName), 
        topicName_(topicName), 
        topicType_(type), 
        targetAliveEnableCbFuncF_(false), 
        targetAliveDisableCbFuncF_(false), 
        targetActivityEnableCbFuncF_(false), 
        targetActivityDisableCbFuncF_(false)
    {
        this->setQoS(qos);
        this->setTargetAliveEventHandler(std::bind(&MultiInteractiveTopicNode::_targetAliveEventHandler, this, std::placeholders::_1, std::placeholders::_2));
        this->setTargetActivityEventHandler(std::bind(&MultiInteractiveTopicNode::_targetActivityEventHandler, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief Set target alive enable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using targetAliveEnableCbFuncLock_.
     */
    void setTargetAliveEnableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(this->targetAliveEnableCbFuncLock_);
        if (this->targetAliveEnableCbFuncF_ && !overwrite)
            return;
        this->targetAliveEnableCbFunc_ = func;
        this->targetAliveEnableCbFuncF_ = true;
    }

    /**
     * @brief Set target alive disable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using targetAliveDisableCbFuncLock_.
     */
    void setTargetAliveDisableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(this->targetAliveDisableCbFuncLock_);
        if (this->targetAliveDisableCbFuncF_ && !overwrite)
            return;
        this->targetAliveDisableCbFunc_ = func;
        this->targetAliveDisableCbFuncF_ = true;
    }

    /**
     * @brief Set target activity enable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using targetActivityEnableCbFuncLock_.
     */
    void setTargetActivityEnableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(this->targetActivityEnableCbFuncLock_);
        if (this->targetActivityEnableCbFuncF_ && !overwrite)
            return;
        this->targetActivityEnableCbFunc_ = func;
        this->targetActivityEnableCbFuncF_ = true;
    }

    /**
     * @brief Set target activity disable event handler.
     * @param[in] func Callback function.
     * @param[in] overwrite Whether to overwrite the existing callback function.
     * @note This function is using targetActivityDisableCbFuncLock_.
     */
    void setTargetActivityDisableEventHandler(const std::function<void(void)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(this->targetActivityDisableCbFuncLock_);
        if (this->targetActivityDisableCbFuncF_ && !overwrite)
            return;
        this->targetActivityDisableCbFunc_ = func;
        this->targetActivityDisableCbFuncF_ = true;
    }

    /**
     * @brief Set Quality of Service.
     * @param[in] qos Quality of Service.
     * @note This function is using qosLock_.
     */
    void setQoS(const rclcpp::QoS& qos)
    {
        std::lock_guard<std::mutex> lock(this->qosLock_);
        this->qos_ = std::make_shared<rclcpp::QoS>(qos);
    }

    /**
     * @brief Get Quality of Service.
     * @return Quality of Service.
     * @note This function is using qosLock_.
     */
    rclcpp::QoS getQoS()
    {
        std::lock_guard<std::mutex> lock(this->qosLock_);
        return *this->qos_;
    }

    std::string getTopicName() const
    {
        return this->topicName_;
    }

    InteractiveTopicType getTopicType() const
    {
        return this->topicType_;
    }
};





}
