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

#include "vehicle_interfaces/msg/interactive_node.hpp"
#include "vehicle_interfaces/msg/interactive_node_master_privilege.hpp"
#include "vehicle_interfaces/srv/interactive_node.hpp"
#include "vehicle_interfaces/srv/interactive_node_req.hpp"

namespace vehicle_interfaces
{

enum InteractiveNodeMasterPrivilegeTargetAlive
{
    TARGET_ALIVE_NONE = 0, 
    TARGET_ALIVE_ENABLE_ONLY, 
    TARGET_ALIVE_DISABLE_ONLY, 
    TARGET_ALIVE_ALL
};

enum InteractiveNodeMasterPrivilegeTargetActivity
{
    TARGET_ACTIVITY_NONE = 0, 
    TARGET_ACTIVITY_ENABLE_ONLY, 
    TARGET_ACTIVITY_DISABLE_ONLY, 
    TARGET_ACTIVITY_ALL
};

/**
 * @brief Structure for master device privilege.
 * @details This structure stores the privilege for the master device, including the target alive privilege, target activity privilege, and node command privilege.
 */
struct InteractiveNodeMasterPrivilege
{
    std::string masterID;
    InteractiveNodeMasterPrivilegeTargetAlive targetAlive;// Target alive privilege.
    InteractiveNodeMasterPrivilegeTargetActivity targetActivity;// Target activity privilege.
    std::set<std::string> nodeCommandSet;// Node command privilege.
    bool requestInteractiveNode;// Whether the master device can request the interactive node.
};

/**
 * @brief Check whether the master device has the privilege to change the target alive status.
 * @param[in] targetAliveSignal Target alive signal.
 * @param[in] masterPrivi Privilege for the master device.
 * @return True if the master device has the privilege, false otherwise.
 */
bool CheckMasterTargetAlivePrivilege(uint8_t targetAliveSignal, InteractiveNodeMasterPrivilege masterPrivi)
{
    switch (masterPrivi.targetAlive)
    {
        case InteractiveNodeMasterPrivilegeTargetAlive::TARGET_ALIVE_ENABLE_ONLY:
            if (targetAliveSignal == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE)
                return true;
            break;
        case InteractiveNodeMasterPrivilegeTargetAlive::TARGET_ALIVE_DISABLE_ONLY:
            if (targetAliveSignal == vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE)
                return true;
            break;
        case InteractiveNodeMasterPrivilegeTargetAlive::TARGET_ALIVE_ALL:
            return true;
            break;
        default:
            return false;
    }
    return false;
}

/**
 * @brief Check whether the master device has the privilege to change the target activity status.
 * @param[in] targetActivitySignal Target activity signal.
 * @param[in] masterPrivi Privilege for the master device.
 * @return True if the master device has the privilege, false otherwise.
 */
bool CheckMasterTargetActivityPrivilege(uint8_t targetActivitySignal, InteractiveNodeMasterPrivilege masterPrivi)
{
    switch (masterPrivi.targetActivity)
    {
        case InteractiveNodeMasterPrivilegeTargetActivity::TARGET_ACTIVITY_ENABLE_ONLY:
            if (targetActivitySignal == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE)
                return true;
            break;
        case InteractiveNodeMasterPrivilegeTargetActivity::TARGET_ACTIVITY_DISABLE_ONLY:
            if (targetActivitySignal == vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_DISABLE)
                return true;
            break;
        case InteractiveNodeMasterPrivilegeTargetActivity::TARGET_ACTIVITY_ALL:
            return true;
            break;
        default:
            return false;
    }
    return false;
}

/**
 * @brief Check whether the master device has the privilege to run the node command.
 * @param[in] commandName Command name.
 * @param[in] masterPrivi Privilege for the master device.
 * @return True if the master device has the privilege, false otherwise.
 */
bool CheckMasterNodeCommandPrivilege(const std::string& commandName, InteractiveNodeMasterPrivilege masterPrivi)
{
    if (masterPrivi.nodeCommandSet.find(commandName) != masterPrivi.nodeCommandSet.end())
        return true;
    return false;
}

bool CheckMasterRequestInteractiveNodePrivilege(InteractiveNodeMasterPrivilege masterPrivi)
{
    return masterPrivi.requestInteractiveNode;
}



/**
 * Interactive node initialization property.
 * @details This structure stores the initialization properties for the interactive node.
 */
struct InteractiveNodeInitProp
{
    uint8_t targetAlive;
    uint8_t targetActivity;
    bool noInteract;
    bool useWhiteList;
    bool useReqServer;
    InteractiveNodeInitProp()
    {
        targetAlive = vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE;
        targetActivity = vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE;
        noInteract = false;
        useWhiteList = false;
        useReqServer = true;
    }
};



/**
 * Interactive node class.
 * @details This class provides a service server for interactive node, which can be used to change the status of the target object and send commands to the target object.
 * @note The InteractiveNode class is designed to be used as a base class for other classes, and the derived class should implement the callback functions for target alive event, target activity event, and node command.
 */
class InteractiveNode : virtual public rclcpp::Node
{
private:
    const std::string nodeName_;
    rclcpp::Service<vehicle_interfaces::srv::InteractiveNode>::SharedPtr interServer_;// Service server for interactive node.
    std::mutex interServerLock_;// Lock interServer_ service callback function.

    std::function<bool(const std::string, const uint8_t)> targetAliveCbFunc_;// Callback function for target alive event.
    std::mutex targetAliveCbFuncLock_;// Lock targetAliveCbFunc_ when setting or calling the callback function.
    std::atomic<bool> targetAliveCbFuncF_;// Flag indicating whether the callback function for target alive event is set.
    std::atomic<uint8_t> targetAlive_;// Store recent target alive status.

    std::function<bool(const std::string, const uint8_t)> targetActivityCbFunc_;// Callback function for target activity event.
    std::mutex targetActivityCbFuncLock_;// Lock targetActivityCbFunc_ when setting or calling the callback function.
    std::atomic<bool> targetActivityCbFuncF_;// Flag indicating whether the callback function for target activity event is set.
    std::atomic<uint8_t> targetActivity_;// Store recent target activity status.

    std::map<std::string, std::function<void(InteractiveNode*, const std::string, const std::vector<std::string>)> > nodeCommandFuncMap_;// Stored callback functions for each command.
    std::mutex nodeCommandFuncMapLock_;// Lock nodeCommandFuncMap_.

    std::atomic<bool> noInteractF_;// Flag indicating whether the interactive method is disabled.
    std::atomic<bool> reqServerF_;// Flag indicating whether to use interactive node request service server.

protected:
    std::map<std::string, InteractiveNodeMasterPrivilege> whiteListMap_;// White list for master devices, including privileges for each master device.
    std::mutex whiteListMapLock_;// Lock whiteListMap_.
    std::atomic<bool> whiteListMapF_;// Flag indicating whether to use white list.

    rclcpp::Service<vehicle_interfaces::srv::InteractiveNodeReq>::SharedPtr interReqServer_;// Service server for interactive node request.
    std::mutex interReqServerLock_;// Lock interReqServer_ service callback function.

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
     * @brief Callback function for InteractiveNode service server.
     * @note This function is using whiteListMapLock_.
     * @note This function is using nodeCommandFuncMapLock_, targetAliveCbFuncLock_ and targetActivityCbFuncLock_.
     */
    void _interactiveNodeServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::InteractiveNode::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::InteractiveNode::Response> response)
    {
        std::lock_guard<std::mutex> interServerLocker(this->interServerLock_);
        response->response = true;
        if (this->noInteractF_)// If the interactive method is disabled, the function does nothing.
        {
            response->response = false;
            response->reason = "No interaction";
            return;
        }

        auto whiteListMapCopy = this->_safeCall(&this->whiteListMap_, this->whiteListMapLock_);
        if (this->whiteListMapF_ && whiteListMapCopy.find(request->device_id) == whiteListMapCopy.end())// If the white list is used but the device ID is not found in the white list, the function does nothing.
        {
            response->response = false;
            response->reason = "No privilege";
            return;
        }

        std::unique_lock<std::mutex> targetAliveCbFuncLocker(this->targetAliveCbFuncLock_, std::defer_lock);
        std::unique_lock<std::mutex> targetActivityCbFuncLocker(this->targetActivityCbFuncLock_, std::defer_lock);

        targetAliveCbFuncLocker.lock();
        if (this->targetAliveCbFuncF_ && request->interactive_node.target_alive != vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_NONE)
        {
            if (this->whiteListMapF_ && !CheckMasterTargetAlivePrivilege(request->interactive_node.target_alive, whiteListMapCopy[request->device_id]))
            {
                response->response = false;
                response->reason = "[Target alive privilege denied]";
            }
            else
                this->targetAlive_ = this->targetAliveCbFunc_(request->device_id, request->interactive_node.target_alive) ? request->interactive_node.target_alive : this->targetAlive_.load();
        }
        targetAliveCbFuncLocker.unlock();

        targetActivityCbFuncLocker.lock();
        if (this->targetActivityCbFuncF_ && request->interactive_node.target_activity != vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_NONE)
        {
            if (this->whiteListMapF_ && !CheckMasterTargetActivityPrivilege(request->interactive_node.target_activity, whiteListMapCopy[request->device_id]))
            {
                response->response = false;
                response->reason = "[Target activity privilege denied]";
            }
            else
                this->targetActivity_ = this->targetActivityCbFunc_(request->device_id, request->interactive_node.target_activity) ? request->interactive_node.target_activity : this->targetActivity_.load();
        }
        targetActivityCbFuncLocker.unlock();

        if (request->node_command_name != "")
        {
            if (this->whiteListMapF_ && !CheckMasterNodeCommandPrivilege(request->node_command_name, whiteListMapCopy[request->device_id]))
            {
                response->response = false;
                response->reason = "[Node command privilege denied]";
            }
            else
            {
                std::lock_guard<std::mutex> lock(this->nodeCommandFuncMapLock_);
                if (this->nodeCommandFuncMap_.find(request->node_command_name) != this->nodeCommandFuncMap_.end())
                    this->nodeCommandFuncMap_[request->node_command_name](this, request->device_id, request->node_command_args);
                else
                {
                    response->response = false;
                    response->reason = "[Node command not found]";
                }
            }
        }
    }

    /**
     * @brief Callback function for InteractiveNodeReq service server.
     * @note The callback function should be implemented in the derived class.
     * @note This function is using whiteListMapLock_.
     */
    virtual void _interactiveNodeReqServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::InteractiveNodeReq::Request> request, 
                                                    std::shared_ptr<vehicle_interfaces::srv::InteractiveNodeReq::Response> response)
    {
        std::lock_guard<std::mutex> interReqServerLocker(this->interReqServerLock_);
        auto whiteListMapCopy = this->_safeCall(&this->whiteListMap_, this->whiteListMapLock_);
        if (this->whiteListMapF_ && whiteListMapCopy.find(request->device_id) == whiteListMapCopy.end())// If the white list is used but the device ID is not found in the white list, the function does nothing.
        {
            response->response = false;
            return;
        }
        if (this->whiteListMapF_ ? whiteListMapCopy[request->device_id].requestInteractiveNode : true)
        {
            auto msg = this->getInteractiveNodeMsg();
            response->response = true;
            response->status = msg;
            return;
        }
        response->response = false;
    }

protected:
    /**
     * @brief Constructor of InteractiveNode class.
     * @param[in] nodeName Name of the node.
     * @param[in] prop Initialization property.
     * @param[in] isMultiF True if InteractiveNode is inherited by MultiInteractiveNode, false otherwise.
     * @note The node name will be nodeName, and the service server name will be nodeName.
     */
    InteractiveNode(const std::string& nodeName, InteractiveNodeInitProp prop, bool isMultiF) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        targetAliveCbFuncF_(false), 
        targetAlive_(prop.targetAlive), 
        targetActivityCbFuncF_(false), 
        targetActivity_(prop.targetActivity), 
        noInteractF_(prop.noInteract), 
        whiteListMapF_(prop.useReqServer), 
        reqServerF_(prop.useReqServer)
    {
        this->interServer_ = this->create_service<vehicle_interfaces::srv::InteractiveNode>(this->nodeName_, 
            std::bind(&InteractiveNode::_interactiveNodeServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
        
        if (this->reqServerF_ && !isMultiF)// If this node is inherited by MultiInteractiveNode, the request server should be created in the MultiInteractiveNode class.
            this->interReqServer_ = this->create_service<vehicle_interfaces::srv::InteractiveNodeReq>(this->nodeName_ + "_Req", 
                std::bind(&InteractiveNode::_interactiveNodeReqServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
    }

public:
    /**
     * @brief Constructor of InteractiveNode class.
     * @param[in] nodeName Name of the node.
     * @param[in] prop Initialization property.
     * @note The node name will be nodeName, and the service server name will be nodeName.
     * @note If useReqServer of prop set to true,  InteractiveNodeReq service server will be created and named as nodeName + "_Req".
     */
    InteractiveNode(const std::string& nodeName, InteractiveNodeInitProp prop) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        targetAliveCbFuncF_(false), 
        targetAlive_(prop.targetAlive), 
        targetActivityCbFuncF_(false), 
        targetActivity_(prop.targetActivity), 
        noInteractF_(prop.noInteract), 
        whiteListMapF_(prop.useReqServer), 
        reqServerF_(prop.useReqServer)
    {
        this->interServer_ = this->create_service<vehicle_interfaces::srv::InteractiveNode>(this->nodeName_, 
            std::bind(&InteractiveNode::_interactiveNodeServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
        
        if (this->reqServerF_)
            this->interReqServer_ = this->create_service<vehicle_interfaces::srv::InteractiveNodeReq>(this->nodeName_ + "_Req", 
                std::bind(&InteractiveNode::_interactiveNodeReqServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
    }


    /**
     * @brief Set callback function for target alive event.
     * @param[in] func Function called when target alive event occurs.
     * @param[in] overwrite True to overwrite the callback function, false to do nothing if the callback function is already set.
     * @note The device ID and target alive status will be passed to the func, and the func should return whether the process is successful.
     * @note If return true, the target alive status will be updated to the new status. Otherwise, the target alive status will remain unchanged.
     * @note This function is using targetAliveCbFuncLock_.
     */
    void setTargetAliveEventHandler(const std::function<bool(const std::string, const uint8_t)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(this->targetAliveCbFuncLock_);
        if (this->targetAliveCbFuncF_ && !overwrite)// If the callback function is already set and overwrite is false, the function does nothing.
            return;
        this->targetAliveCbFunc_ = func;
        this->targetAliveCbFuncF_ = true;
    }

    /**
     * @brief Set callback function for target activity event.
     * @param[in] func Function called when target activity event occurs.
     * @param[in] overwrite True to overwrite the callback function, false to do nothing if the callback function is already set.
     * @note The device ID and target activity status will be passed to the func, and the func should return whether the process is successful.
     * @note If return true, the target activity status will be updated to the new status. Otherwise, the target activity status will remain unchanged.
     * @note This function is using targetActivityCbFuncLock_.
     */
    void setTargetActivityEventHandler(const std::function<bool(const std::string, const uint8_t)>& func, bool overwrite = false)
    {
        std::lock_guard<std::mutex> lock(this->targetActivityCbFuncLock_);
        if (this->targetActivityCbFuncF_ && !overwrite)// If the callback function is already set and overwrite is false, the function does nothing.
            return;
        this->targetActivityCbFunc_ = func;
        this->targetActivityCbFuncF_ = true;
    }

    /**
     * @brief Add master privilege to the white list.
     * @param[in] masterPrivi Privilege for the master device.
     * @note The function will overwrite the privilege if the device ID is already in the white list.
     * @note This function is using whiteListMapLock_.
     */
    virtual void addMasterPrivilege(const InteractiveNodeMasterPrivilege& masterPrivi)
    {
        std::lock_guard<std::mutex> lock(this->whiteListMapLock_);
        this->whiteListMap_[masterPrivi.masterID] = masterPrivi;
    }

    /**
     * @brief Update master device privilege for node command.
     * @param[in] masterID Device ID of the master device.
     * @param[in] command Node command.
     * @param[in] add True to add the command to the privilege, false to remove the command from the privilege.
     * @note If the deviceID is found, the function will add or remove the command from the privilege.
     * @note This function is using whiteListMapLock_.
     */
    void updateMasterDeviceCommand(const std::string& masterID, const std::string& command, bool add)
    {
        std::lock_guard<std::mutex> lock(this->whiteListMapLock_);
        if (this->whiteListMap_.find(masterID) != this->whiteListMap_.end())
        {
            if (add)
                this->whiteListMap_[masterID].nodeCommandSet.insert(command);
            else
                this->whiteListMap_[masterID].nodeCommandSet.erase(command);
        }
    }

    /**
     * @brief Update master device privilege for node command set.
     * @param[in] masterID Device ID of the master device.
     * @param[in] commandSet Set of node commands.
     * @note The function will overwrite the command set if the deviceID is already in the white list.
     * @note This function is using whiteListMapLock_.
     */
    void updateMasterDeviceCommandSet(const std::string& masterID, const std::set<std::string>& commandSet)
    {
        std::lock_guard<std::mutex> lock(this->whiteListMapLock_);
        if (this->whiteListMap_.find(masterID) != this->whiteListMap_.end())
            this->whiteListMap_[masterID].nodeCommandSet = commandSet;
    }

    /**
     * @brief Add callback function for node command.
     * @param[in] command Command string.
     * @param[in] func Function called when the command event occurs. The arguments are interactive node pointer, device ID, and command arguments.
     * @note The device ID will be passed to the function.
     * @note This function is using nodeCommandFuncMapLock_.
     */
    void addNodeCommandEventHandler(const std::string& command, const std::function<void(InteractiveNode *node, const std::string, const std::vector<std::string>)>& func)
    {
        std::lock_guard<std::mutex> lock(this->nodeCommandFuncMapLock_);
        this->nodeCommandFuncMap_[command] = func;
    }

    /**
     * @brief Set whether enable the node interaction.
     */
    void noInteraction(bool flag)
    {
        this->noInteractF_ = !flag;
    }

    /**
     * @brief Set whether to use white list.
     */
    void useWhiteList(bool flag)
    {
        this->whiteListMapF_ = flag;
    }

    /**
     * @brief Set target alive status manually.
     */
    void setTargetAlive(uint8_t targetAlive)
    {
        this->targetAlive_ = targetAlive;
    }

    /**
     * @brief Set target activity status manually.
     */
    void setTargetActivity(uint8_t targetActivity)
    {
        this->targetActivity_ = targetActivity;
    }

    /**
     * @brief Get node name.
     */
    std::string getNodeName() const
    {
        return this->nodeName_;
    }

    /**
     * @brief Get target alive status.
     */
    uint8_t getTargetAlive()
    {
        return this->targetAlive_;
    }

    /**
     * @brief Get target activity status.
     */
    uint8_t getTargetActivity()
    {
        return this->targetActivity_;
    }

    /**
     * @brief Get interactive node message.
     * @return Interactive node message.
     * @note This function is using whiteListMapLock_.
     * @note The callback function should be implemented in the derived class.
     */
    virtual vehicle_interfaces::msg::InteractiveNode getInteractiveNodeMsg()
    {
        vehicle_interfaces::msg::InteractiveNode ret;
        ret.node_name = this->nodeName_;// The node name has suffix "_interactive".
        ret.target_alive = this->targetAlive_;
        ret.target_activity = this->targetActivity_;
        ret.no_interaction = this->noInteractF_;
        ret.use_white_list = this->whiteListMapF_;
        auto whiteListMapCopy = this->_safeCall(&this->whiteListMap_, this->whiteListMapLock_);
        for (const auto& [master, privi] : whiteListMapCopy)
        {
            vehicle_interfaces::msg::InteractiveNodeMasterPrivilege masterPrivi;
            masterPrivi.master_id = privi.masterID;
            masterPrivi.target_alive_privilege = privi.targetAlive;
            masterPrivi.target_activity_privilege = privi.targetActivity;
            for (const auto& command : privi.nodeCommandSet)
                masterPrivi.node_command.push_back(command);
            masterPrivi.request_interactive_node_privilege = privi.requestInteractiveNode;
            ret.white_list.push_back(masterPrivi);
        }
        return ret;
    }
};



/**
 * Multi-interactive node class.
 * @details This class derives from the InteractiveNode class and provides additional functions to store the target alive status and target activity status for each master device.
 */
class MultiInteractiveNode : public InteractiveNode
{
private:
    std::map<std::string, uint8_t> targetAliveMap_;// Store target alive status for each master device.
    std::mutex targetAliveMapLock_;// Lock targetAliveMap_.

    std::map<std::string, uint8_t> targetActivityMap_;// Store target activity status for each master device.
    std::mutex targetActivityMapLock_;// Lock targetActivityMap_.

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
     * @brief Callback function for InteractiveNodeReq service server.
     * @note This function is implemented in the derived class.
     * @note This function is using whiteListMapLock_, targetAliveMapLock_ and targetActivityMapLock_.
     */
    void _interactiveNodeReqServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::InteractiveNodeReq::Request> request, 
                                            std::shared_ptr<vehicle_interfaces::srv::InteractiveNodeReq::Response> response) override
    {
        std::lock_guard<std::mutex> interReqServerLocker(this->interReqServerLock_);
        auto whiteListMapCopy = this->_safeCall(&this->whiteListMap_, this->whiteListMapLock_);
        if (this->whiteListMapF_ && whiteListMapCopy.find(request->device_id) == whiteListMapCopy.end())// If the white list is used but the device ID is not found in the white list, the function does nothing.
        {
            response->response = false;
            return;
        }
        if (this->whiteListMapF_ ? whiteListMapCopy[request->device_id].requestInteractiveNode : true)
        {
            auto msg = this->getInteractiveNodeMsg();
            response->response = true;
            response->status = msg;
            return;
        }
        response->response = false;
    }

public:
    /**
     * @brief Constructor of MultiInteractiveNode class.
     * @param[in] nodeName Name of the node.
     * @param[in] prop Initialization property.
     * @note The node name will be nodeName, and the service server name will be nodeName.
     * @note If useReqServer of prop set to true,  InteractiveNodeReq service server will be created and named as nodeName + "_Req".
     */
    MultiInteractiveNode(const std::string& nodeName, InteractiveNodeInitProp prop) : 
        InteractiveNode(nodeName, prop, true), 
        rclcpp::Node(nodeName)
    {
        if (prop.useReqServer)
            this->interReqServer_ = this->create_service<vehicle_interfaces::srv::InteractiveNodeReq>(this->getNodeName() + "_Req", 
                std::bind(&MultiInteractiveNode::_interactiveNodeReqServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
    }

    /**
     * @brief Add master privilege to the white list.
     * @param[in] masterPrivi Privilege for the master device.
     * @note The function will overwrite the privilege if the device ID is already in the white list.
     * @note The function will add TARGET_ALIVE_NONE and TARGET_ACTIVITY_NONE to the status map for the master device by default.
     * @note If user wants to change the status of the master device, call addTargetAlive() or addTargetActivity() for target_alive and target_activity status respectively.
     * @note This function is using whiteListMapLock_, targetAliveMapLock_ and targetActivityMapLock_.
     */
    void addMasterPrivilege(const InteractiveNodeMasterPrivilege& masterPrivi) override
    {
        std::lock_guard<std::mutex> lock(this->whiteListMapLock_);
        this->whiteListMap_[masterPrivi.masterID] = masterPrivi;
        this->addTargetAlive(masterPrivi.masterID, vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_NONE);
        this->addTargetActivity(masterPrivi.masterID, vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_NONE);
    }

    /**
     * @brief Add target status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetAlive Target alive status.
     * @param[in] targetActivity Target activity status.
     * @note This function is using targetAliveMapLock_ and targetActivityMapLock_.
     */
    void addMasterTargetStatus(const std::string& master, uint8_t targetAlive, uint8_t targetActivity)
    {
        this->addTargetAlive(master, targetAlive);
        this->addTargetActivity(master, targetActivity);
    }

    /**
     * @brief Add target alive status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetAlive Target alive status.
     * @note This function is using targetAliveMapLock_.
     */
    void addTargetAlive(const std::string& master, uint8_t targetAlive)
    {
        std::lock_guard<std::mutex> lock(this->targetAliveMapLock_);
        this->targetAliveMap_[master] = targetAlive;
    }

    /**
     * @brief Modify target alive status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetAlive Target alive status.
     * @note If the master is not found in the map, the function does nothing.
     * @note This function is using targetAliveMapLock_.
     */
    void modifyTargetAlive(const std::string& master, uint8_t targetAlive)
    {
        std::lock_guard<std::mutex> lock(this->targetAliveMapLock_);
        if (this->targetAliveMap_.find(master) != this->targetAliveMap_.end())
            this->targetAliveMap_[master] = targetAlive;
    }

    /**
     * @brief Add target activity status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetActivity Target activity status.
     * @note This function is using targetActivityMapLock_.
     */
    void addTargetActivity(const std::string& master, uint8_t targetActivity)
    {
        std::lock_guard<std::mutex> lock(this->targetActivityMapLock_);
        this->targetActivityMap_[master] = targetActivity;
    }

    /**
     * @brief Modify target activity status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[in] targetActivity Target activity status.
     * @note If the master is not found in the map, the function does nothing.
     * @note This function is using targetActivityMapLock_.
     */
    void modifyTargetActivity(const std::string& master, uint8_t targetActivity)
    {
        std::lock_guard<std::mutex> lock(this->targetActivityMapLock_);
        if (this->targetActivityMap_.find(master) != this->targetActivityMap_.end())
            this->targetActivityMap_[master] = targetActivity;
    }

    /**
     * @brief Get target alive status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[out] outTargetAlive Output target alive status.
     * @return True if the master is found in the map. Otherwise, the function returns false.
     * @note This function is using targetAliveMapLock_.
     */
    bool getTargetAlive(const std::string& master, uint8_t& outTargetAlive)
    {
        std::lock_guard<std::mutex> lock(this->targetAliveMapLock_);
        if (this->targetAliveMap_.find(master) != this->targetAliveMap_.end())
        {
            outTargetAlive = this->targetAliveMap_[master];
            return true;
        }
        return false;
    }

    /**
     * @brief Get target alive status map.
     * @return Target alive status map.
     * @note This function is using targetAliveMapLock_.
     */
    std::map<std::string, uint8_t> getAllTargetAlive()
    {
        std::lock_guard<std::mutex> lock(this->targetAliveMapLock_);
        return this->targetAliveMap_;
    }

    /**
     * @brief Get target activity status for a master device.
     * @param[in] master Device ID of the master device.
     * @param[out] outTargetActivity Output target activity status.
     * @return True if the master is found in the map. Otherwise, the function returns false.
     * @note This function is using targetActivityMapLock_.
     */
    bool getTargetActivity(const std::string& master, uint8_t& outTargetActivity)
    {
        std::lock_guard<std::mutex> lock(this->targetActivityMapLock_);
        if (this->targetActivityMap_.find(master) != this->targetActivityMap_.end())
        {
            outTargetActivity = this->targetActivityMap_[master];
            return true;
        }
        return false;
    }

    /**
     * @brief Get target activity status map.
     * @return Target activity status map.
     * @note This function is using targetActivityMapLock_.
     */
    std::map<std::string, uint8_t> getAllTargetActivity()
    {
        std::lock_guard<std::mutex> lock(this->targetActivityMapLock_);
        return this->targetActivityMap_;
    }

    /**
     * @brief Check all target alive status.
     * @return Target alive status.
     * @note If there are multiple target alive status, the function returns the following value:
     * @note - If there is at least one "enable" status and no "disable" status, the function returns "enable".
     * @note - If there is at least one "disable" status, the function returns "disable".
     * @note - Otherwise, the function returns "none".
     * @note This function is using targetAliveMapLock_.
     */
    uint8_t checkAllTargetAlive()
    {
        std::lock_guard<std::mutex> lock(this->targetAliveMapLock_);
        size_t targetAliveEnableCount = 0;
        size_t targetAliveDisableCount = 0;
        size_t targetAliveUnknownCount = 0;
        for (auto& i : this->targetAliveMap_)
        {
            switch (i.second)
            {
                case vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE:
                    targetAliveEnableCount++;
                    break;
                case vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE:
                    targetAliveDisableCount++;
                    break;
                default:
                    targetAliveUnknownCount++;
                    break;
            }
        }
        return (targetAliveEnableCount > 0 && targetAliveDisableCount == 0) ? vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_ENABLE : 
                (targetAliveDisableCount > 0) ? vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_DISABLE : 
                vehicle_interfaces::msg::InteractiveNode::TARGET_ALIVE_NONE;
    }

    /**
     * @brief Check all target activity status.
     * @return Target activity status.
     * @note If there are multiple target activity status, the function returns the following value:
     * @note - If there is at least one "enable" status and no "disable" status, the function returns "enable".
     * @note - If there is at least one "disable" status, the function returns "disable".
     * @note - Otherwise, the function returns "none".
     * @note This function is using targetActivityMapLock_.
     */
    uint8_t checkAllTargetActivity()
    {
        std::lock_guard<std::mutex> lock(this->targetActivityMapLock_);
        size_t targetActivityEnableCount = 0;
        size_t targetActivityDisableCount = 0;
        size_t targetActivityUnknownCount = 0;
        for (auto& i : this->targetActivityMap_)
        {
            switch (i.second)
            {
                case vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE:
                    targetActivityEnableCount++;
                    break;
                case vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_DISABLE:
                    targetActivityDisableCount++;
                    break;
                default:
                    targetActivityUnknownCount++;
                    break;
            }
        }
        return (targetActivityEnableCount > 0 && targetActivityDisableCount == 0) ? vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_ENABLE : 
                (targetActivityDisableCount > 0) ? vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_DISABLE : 
                vehicle_interfaces::msg::InteractiveNode::TARGET_ACTIVITY_NONE;
    }

    vehicle_interfaces::msg::InteractiveNode getInteractiveNodeMsg() override
    {
        auto msg = InteractiveNode::getInteractiveNodeMsg();
        msg.is_multi = true;
        for (const auto& [master, alive] : this->getAllTargetAlive())
        {
            msg.target_alive_master_vec.push_back(master);
            msg.target_alive_vec.push_back(alive);
        }

        for (const auto& [master, activity] : this->getAllTargetActivity())
        {
            msg.target_activity_master_vec.push_back(master);
            msg.target_activity_vec.push_back(activity);
        }
        return msg;
    }
};

}
