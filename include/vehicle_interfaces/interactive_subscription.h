#pragma once
#include "vehicle_interfaces/interactive_topic.h"

namespace vehicle_interfaces
{

/**
 * @brief InteractiveSubscription inherits from InteractiveTopicNode.
 * @note InteractiveSubscription class can be controlled by any master device which has the privilege to control the subscription.
 * @note The class provides a controlable subscription with following functionalities:
 * @note 1. Create and destroy subscription by requesting target alive event with enable and disable status.
 */
template <typename T>
class InteractiveSubscription : public InteractiveTopicNode
{
private:
    std::shared_ptr<rclcpp::Subscription<T> > sub_;// Subscription.
    std::mutex subLock_;// Lock sub_.
    std::atomic<bool> subscriptionF_;// Flag indicating whether the subscription is created.

    std::function<void(const std::shared_ptr<T>)> subCbFunc_;// Subscription callback function.

private:
    /**
     * @brief Target alive event handler.
     * @details The function is called when the interactive service server receives a target alive request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetAlive Requested target alive status.
     * @return Wehther the event is handled successfully. If return true, the target alive status will be updated. Otherwise, the target alive status will remain unchanged.
     * @note This function is using subLock_.
     */
    bool _targetAliveEventHandler(const std::string deviceID, const uint8_t targetAlive)
    {
        if (targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
        {
            std::lock_guard<std::mutex> lock(this->subLock_);
            if (this->sub_.use_count() <= 0)
                this->sub_ = this->create_subscription<T>(this->getTopicName(), this->getQoS(), this->subCbFunc_);
            this->subscriptionF_ = true;
        }
        else if (targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE)
        {
            std::lock_guard<std::mutex> lock(this->subLock_);
            if (this->sub_.use_count() > 0)
                this->sub_.reset();
            this->subscriptionF_ = false;
        }
        else
            return false;
        return true;
    }

public:
    /**
     * @brief Constructor of InteractiveSubscription class.
     * @param[in] nodeName Name of the node.
     * @param[in] topicName Name of the topic.
     * @param[in] qos Quality of Service.
     * @param[in] callback Subscription callback function.
     * @param[in] targetAlive Default target alive status.
     * @param[in] targetActivity Default target activity status.
     * @note The subscription node will be created and named as nodeName.
     * @note The subscription will be created and named as topicName.
     * @note The InteractiveNode service server will be created and named as nodeName.
     * @note The InteractiveNodeReq service server will be created and named as nodeName + "_Req".
     */
    InteractiveSubscription(const std::string& nodeName, 
                            const std::string& topicName, 
                            const rclcpp::QoS& qos, 
                            std::function<void(const std::shared_ptr<T>)> callback, 
                            InteractiveNodeInitProp prop = InteractiveNodeInitProp()) : 
        InteractiveTopicNode(nodeName, topicName, qos, INTERACTIVE_TOPIC_SUBSCRIPTION, prop), 
        rclcpp::Node(nodeName), 
        subCbFunc_(callback), 
        subscriptionF_(false)
    {
        if (prop.targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
        {
            this->sub_ = this->create_subscription<T>(this->getTopicName(), this->getQoS(), this->subCbFunc_);
            this->subscriptionF_ = true;
        }

        // Set target alive and target activity event handlers.
        this->setTargetAliveEventHandler(std::bind(&InteractiveSubscription::_targetAliveEventHandler, this, std::placeholders::_1, std::placeholders::_2));
    }
};



/**
 * @brief MultiInteractiveSubscription inherits from MultiInteractiveTopicNode.
 * @note MultiInteractiveSubscription class can be controlled by any master device which has the privilege to control the subscription.
 * @note The class provides a controlable subscription with following functionalities:
 * @note 1. Create and destroy subscription by requesting target alive event with enable and disable status.
 * @note The target event will be raised by checking all master devices' target alive status.
 */
template <typename T>
class MultiInteractiveSubscription : public MultiInteractiveTopicNode
{
private:
    std::shared_ptr<rclcpp::Subscription<T> > sub_;// Subscription.
    std::mutex subLock_;// Lock sub_.
    std::atomic<bool> subscriptionF_;// Flag indicating whether the subscription is created.

    std::function<void(const std::shared_ptr<T>)> subCbFunc_;// Subscription callback function.

private:
    /**
     * @brief Target alive enable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to enable.
     * @note This function is using subLock_.
     */
    void _targetAliveEnableCbFunc()
    {
        std::lock_guard<std::mutex> lock(this->subLock_);
        if (this->sub_.use_count() <= 0)
            this->sub_ = this->create_subscription<T>(this->getTopicName(), this->getQoS(), this->subCbFunc_);
        this->subscriptionF_ = true;
    }

    /**
     * @brief Target alive disable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to disable.
     * @note This function is using subLock_.
     */
    void _targetAliveDisableCbFunc()
    {
        std::lock_guard<std::mutex> lock(this->subLock_);
        if (this->sub_.use_count() > 0)
            this->sub_.reset();
        this->subscriptionF_ = false;
    }

public:
    /**
     * @brief Constructor of MultiInteractiveSubscription class.
     * @param[in] nodeName Name of the node.
     * @param[in] topicName Name of the topic.
     * @param[in] qos Quality of Service.
     * @param[in] callback Subscription callback function.
     * @param[in] targetAlive Default target alive status.
     * @param[in] targetActivity Default target activity status.
     * @note The subscription node will be created and named as nodeName.
     * @note The subscription will be created and named as topicName.
     * @note The InteractiveNode service server will be created and named as nodeName.
     * @note The InteractiveNodeReq service server will be created and named as nodeName + "_Req".
     */
    MultiInteractiveSubscription(const std::string& nodeName, 
                                    const std::string& topicName, 
                                    const rclcpp::QoS& qos, 
                                    std::function<void(const std::shared_ptr<T>)> callback, 
                                    InteractiveNodeInitProp prop = InteractiveNodeInitProp()) : 
        MultiInteractiveTopicNode(nodeName, topicName, qos, INTERACTIVE_TOPIC_SUBSCRIPTION, prop), 
        rclcpp::Node(nodeName), 
        subCbFunc_(callback), 
        subscriptionF_(false)
    {
        if (prop.targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
        {
            this->sub_ = this->create_subscription<T>(this->getTopicName(), this->getQoS(), this->subCbFunc_);
            this->subscriptionF_ = true;
        }

        // Set target alive event handlers.
        this->setTargetAliveEnableEventHandler(std::bind(&MultiInteractiveSubscription::_targetAliveEnableCbFunc, this));
        this->setTargetAliveDisableEventHandler(std::bind(&MultiInteractiveSubscription::_targetAliveDisableCbFunc, this));
    }
};

}