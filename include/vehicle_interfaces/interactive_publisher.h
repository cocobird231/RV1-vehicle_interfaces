#pragma once
#include "vehicle_interfaces/interactive_topic.h"

namespace vehicle_interfaces
{

/**
 * @brief InteractivePublisher inherits from InteractiveTopicNode. 
 * @note InteractivePublisher class can be controlled by any master device which has the privilege to control the publisher.
 * @note The class provides a controlable publisher with following functionalities:
 * @note 1. Create and destroy publisher by requesting target alive event with enable and disable status.
 * @note 2. Enable and disable publishing by requesting target activity event with enable and disable status.
 */
template <typename T>
class InteractivePublisher : public InteractiveTopicNode
{
private:
    std::shared_ptr<rclcpp::Publisher<T> > pub_;// Publisher.
    std::mutex pubLock_;// Lock pub_.
    std::atomic<bool> publisherF_;// Flag indicating whether the publisher is created.
    std::atomic<bool> publishF_;// Flag indicating whether the publishing is enabled.

private:
    /**
     * @brief Target alive event handler.
     * @details The function is called when the interactive service server receives a target alive request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetAlive Requested target alive status.
     * @return Wehther the event is handled successfully. If return true, the target alive status will be updated. Otherwise, the target alive status will remain unchanged.
     * @note This function is using pubLock_.
     */
    bool _targetAliveEventHandler(const std::string deviceID, const uint8_t targetAlive)
    {
        if (targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
        {
            std::lock_guard<std::mutex> lock(this->pubLock_);
            if (this->pub_.use_count() <= 0)
                this->pub_ = this->create_publisher<T>(this->getTopicName(), this->getQoS());
            this->publisherF_ = true;
        }
        else if (targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE)
        {
            std::lock_guard<std::mutex> lock(this->pubLock_);
            if (this->pub_.use_count() > 0)
                this->pub_.reset();
            this->publisherF_ = false;
        }
        else
            return false;
        return true;
    }

    /**
     * @brief Target activity event handler.
     * @details The function is called when the interactive service server receives a target activity request.
     * @param[in] deviceID Requested device ID.
     * @param[in] targetActivity Requested target activity status.
     * @return Wehther the event is handled successfully. If return true, the target activity status will be updated. Otherwise, the target activity status will remain unchanged.
     */
    bool _targetActivityEventHandler(const std::string deviceID, const uint8_t targetActivity)
    {
        if (targetActivity == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE)
        {
            this->publishF_ = true;
        }
        else if (targetActivity == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_DISABLE)
        {
            this->publishF_ = false;
        }
        else
            return false;
        return true;
    }

public:
    /**
     * @brief Constructor of InteractivePublisher class.
     * @param[in] topicName Name of the topic.
     * @param[in] targetAlive Default target alive status.
     * @param[in] targetActivity Default target activity status.
     * @note The publisher node will be created and named as nodeName.
     * @note The publisher will be created and named as topicName.
     * @note The InteractiveNode service server will be created and named as nodeName.
     * @note The InteractiveNodeReq service server will be created and named as nodeName + "_Req".
     */
    InteractivePublisher(const std::string& nodeName, 
                            const std::string& topicName, 
                            const rclcpp::QoS& qos, 
                            InteractiveNodeInitProp prop = InteractiveNodeInitProp()) : 
        InteractiveTopicNode(nodeName, topicName, qos, INTERACTIVE_TOPIC_PUBLISHER, prop), 
        rclcpp::Node(nodeName), 
        publisherF_(false), 
        publishF_(false)
    {
        if (prop.targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
        {
            this->pub_ = this->create_publisher<T>(this->getTopicName(), this->getQoS());
            this->publisherF_ = true;
        }
        if (prop.targetActivity == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE)
            this->publishF_ = true;

        // Set target alive and target activity event handlers.
        this->setTargetAliveEventHandler(std::bind(&InteractivePublisher::_targetAliveEventHandler, this, std::placeholders::_1, std::placeholders::_2));
        this->setTargetActivityEventHandler(std::bind(&InteractivePublisher::_targetActivityEventHandler, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief Publish message.
     * @param[in] msg Message to be published.
     * @note The message will be published only if the target alive and target activity status are "enable".
     * @note This function is using pubLock_.
     */
    void publish(const T& msg)
    {
        std::lock_guard<std::mutex> lock(this->pubLock_);
        if (this->publisherF_ && this->publishF_)
            this->pub_->publish(msg);
    }
};



/**
 * @brief MultiInteractivePublisher inherits from MultiInteractiveTopicNode.
 * @note MultiInteractivePublisher class can be controlled by any master device which has the privilege to control the publisher.
 * @note The class provides a controlable publisher with following functionalities:
 * @note 1. Create and destroy publisher by requesting target alive event with enable and disable status.
 * @note 2. Enable and disable publishing by requesting target activity event with enable and disable status.
 * @note The target event will be raised by checking all master devices' target alive and target activity status.
 */
template <typename T>
class MultiInteractivePublisher : public MultiInteractiveTopicNode
{
private:
    std::shared_ptr<rclcpp::Publisher<T> > pub_;// Publisher.
    std::mutex pubLock_;// Lock pub_.
    std::atomic<bool> publisherF_;// Flag indicating whether the publisher is created.
    std::atomic<bool> publishF_;// Flag indicating whether the publishing is enabled.

private:
    /**
     * @brief Target alive enable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to enable.
     * @note This function is using pubLock_.
     */
    void _targetAliveEnableCbFunc()
    {
        std::lock_guard<std::mutex> lock(this->pubLock_);
        if (this->pub_.use_count() <= 0)
            this->pub_ = this->create_publisher<T>(this->getTopicName(), this->getQoS());
        this->publisherF_ = true;
    }

    /**
     * @brief Target alive disable callback function.
     * @details The function is called when the interactive service server receives a target alive request, and all target alive status are set to disable.
     * @note This function is using pubLock_.
     */
    void _targetAliveDisableCbFunc()
    {
        std::lock_guard<std::mutex> lock(this->pubLock_);
        if (this->pub_.use_count() > 0)
            this->pub_.reset();
        this->publisherF_ = false;
    }

    /**
     * @brief Target activity enable callback function.
     * @details The function is called when the interactive service server receives a target activity request, and all target activity status are set to enable.
     */
    void _targetActivityEnableCbFunc()
    {
        this->publishF_ = true;
    }

    /**
     * @brief Target activity disable callback function.
     * @details The function is called when the interactive service server receives a target activity request, and all target activity status are set to disable.
     */
    void _targetActivityDisableCbFunc()
    {
        this->publishF_ = false;
    }

public:
    /**
     * @brief Constructor of MultiInteractivePublisher class.
     * @param[in] nodeName Name of the node.
     * @param[in] topicName Name of the topic.
     * @param[in] qos Quality of Service.
     * @param[in] targetAlive Default target alive status.
     * @param[in] targetActivity Default target activity status.
     * @note The publisher node will be created and named as nodeName.
     * @note The publisher will be created and named as topicName.
     * @note The InteractiveNode service server will be created and named as nodeName.
     * @note The InteractiveNodeReq service server will be created and named as nodeName + "_Req".
     */
    MultiInteractivePublisher(const std::string& nodeName, 
                                const std::string& topicName, 
                                const rclcpp::QoS& qos, 
                                InteractiveNodeInitProp prop = InteractiveNodeInitProp()) : 
        MultiInteractiveTopicNode(nodeName, topicName, qos, INTERACTIVE_TOPIC_PUBLISHER, prop), 
        rclcpp::Node(nodeName), 
        publisherF_(false), 
        publishF_(false)
    {
        if (prop.targetAlive == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
        {
            this->pub_ = this->create_publisher<T>(this->getTopicName(), this->getQoS());
            this->publisherF_ = true;
        }
        if (prop.targetActivity == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE)
            this->publishF_ = true;

        // Set target alive and target activity event handlers.
        this->setTargetAliveEnableEventHandler(std::bind(&MultiInteractivePublisher::_targetAliveEnableCbFunc, this));
        this->setTargetAliveDisableEventHandler(std::bind(&MultiInteractivePublisher::_targetAliveDisableCbFunc, this));
        this->setTargetActivityEnableEventHandler(std::bind(&MultiInteractivePublisher::_targetActivityEnableCbFunc, this));
        this->setTargetActivityDisableEventHandler(std::bind(&MultiInteractivePublisher::_targetActivityDisableCbFunc, this));
    }

    /**
     * @brief Publish message.
     * @param[in] msg Message to be published.
     * @note The message will be published only if the target alive and target activity status are "enable".
     * @note This function is using pubLock_.
     */
    void publish(const T& msg)
    {
        std::lock_guard<std::mutex> lock(this->pubLock_);
        if (this->publisherF_ && this->publishF_)
            this->pub_->publish(msg);
    }
};

}
