#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <map>

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/msg/surround_emergency.hpp"
#include "vehicle_interfaces/srv/safety_reg.hpp"
#include "vehicle_interfaces/srv/safety_req.hpp"

#ifndef ROS_DISTRO// 0: eloquent, 1: foxy, 2: humble
#define ROS_DISTRO 1
#endif

using namespace std::chrono_literals;

namespace vehicle_interfaces
{

/**
 * Directions for surround emergency usage.
 */
enum EmergencyScoreDirection { FORWARD = 0, 
                                BACKWARD, 
                                LEFT, 
                                RIGHT, 
                                FORWARD_LEFT, 
                                FORWARD_RIGHT, 
                                BACKWARD_LEFT, 
                                BACKWARD_RIGHT };

/**
 * SafetyNode
 */
class SafetyNode : virtual public rclcpp::Node
{
private:
    std::shared_ptr<rclcpp::Node> regClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::SafetyReg>::SharedPtr regClient_;

    std::shared_ptr<rclcpp::Node> reqClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::SafetyReq>::SharedPtr reqClient_;

    // Node enable
    std::atomic<bool> nodeEnableF_;

private:
    inline bool _regSurroundEmergency(vehicle_interfaces::srv::SafetyReg::Request::SharedPtr request)
    {
        auto result = this->regClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 20ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 20ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
                return true;
        }
        return false;
    }

    inline bool _reqSurroundEmergency(vehicle_interfaces::srv::SafetyReq::Request::SharedPtr request, std::vector<std::string>& deviceIDs, std::vector<vehicle_interfaces::msg::SurroundEmergency>& emergencies)
    {
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 20ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 20ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
            {
                deviceIDs = response->device_id_vec;
                emergencies = response->emergency_scores;
                return true;
            }
        }
        return false;
    }

public:
    SafetyNode(const std::string& nodeName, const std::string& safetyServiceName) : 
        rclcpp::Node(nodeName), 
        nodeEnableF_(false)
    {
        if (safetyServiceName == "")
        {
            RCLCPP_WARN(this->get_logger(), "[SafetyNode] Ignored.");
            return;
        }
        this->regClientNode_ = rclcpp::Node::make_shared(nodeName + "_safetyreg_client");
        this->regClient_ = this->regClientNode_->create_client<vehicle_interfaces::srv::SafetyReg>(safetyServiceName + "_Reg");

        this->reqClientNode_ = rclcpp::Node::make_shared(nodeName + "_safetyreq_client");
        this->reqClient_ = this->reqClientNode_->create_client<vehicle_interfaces::srv::SafetyReq>(safetyServiceName + "_Req");

        this->nodeEnableF_ = true;
        RCLCPP_INFO(this->get_logger(), "[SafetyNode] Constructed.");
    }

    /**
     * @brief Register emergency score to SafetyServer with specific direction.
     * 
     * @param[in] deviceID the name to register to SafetyServer, e.g. the node name.
     * @param[in] emP the emergency score.
     * @param[in] direction the direction where emergency occurs.
     * @param[in] lifetime_ms the lifetime of `emP` score.
     * @return true if request success. Otherwise, false.
     */
    bool setEmergency(std::string deviceID, float emP, EmergencyScoreDirection direction, float lifetime_ms = 500.0)
    {
        if (!this->nodeEnableF_)
            return false;
        vehicle_interfaces::msg::SurroundEmergency score;
        score.emergency_percentages[direction] = emP;
        score.lifetime_ms = lifetime_ms;
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReg::Request>();
        request->device_id_vec.push_back(deviceID);
        request->emergency_scores.push_back(score);
        return this->_regSurroundEmergency(request);
    }

    /**
     * @brief Register emergency scores to SafetyServer.
     * 
     * @param[in] deviceID the name to register to SafetyServer, e.g. the node name.
     * @param[in] emPs the 8-direction emergency scores.
     * @param[in] lifetime_ms the lifetime of `emP` score.
     * @return true if request success. Otherwise, false.
     */
    bool setEmergency(std::string deviceID, std::array<float, 8> emPs, float lifetime_ms = 500.0)
    {
        if (!this->nodeEnableF_)
            return false;
        vehicle_interfaces::msg::SurroundEmergency score;
        score.emergency_percentages = emPs;
        score.lifetime_ms = lifetime_ms;
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReg::Request>();
        request->device_id_vec.push_back(deviceID);
        request->emergency_scores.push_back(score);
        return this->_regSurroundEmergency(request);
    }

    /**
     * @brief Register emergency scores to SafetyServer.
     * 
     * @param[in] emergencies the map of node name and SurroundEmergency.
     * @return true if request success. Otherwise, false.
     */
    bool setEmergencies(std::map<std::string, vehicle_interfaces::msg::SurroundEmergency> emergencies)
    {
        if (!this->nodeEnableF_)
            return false;
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReg::Request>();
        for (const auto& [nodeName, emergency] : emergencies)
        {
            request->device_id_vec.push_back(nodeName);
            request->emergency_scores.push_back(emergency);
        }
        return this->_regSurroundEmergency(request);
    }

    /**
     * @brief Request emergency score from SafetyServer with specific device ID and direction.
     * 
     * @param[in] deviceID either node name or "nearest". Do not pass "all" to prevent unexpected result.
     * @param[out] outEmP output emergency value of specific direction.
     * @param[in] direction Specify the direction according to EmergencyScoreDirection.
     * @return true if request success. Otherwise, false.
     */
    bool getEmergency(std::string deviceID, float& outEmP, EmergencyScoreDirection direction)
    {
        if (!this->nodeEnableF_)
            return false;
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReq::Request>();
        request->device_id = deviceID;
        std::vector<std::string> resIDs;
        std::vector<vehicle_interfaces::msg::SurroundEmergency> resEmPs;
        try
        {
            if (this->_reqSurroundEmergency(request, resIDs, resEmPs) && resIDs.back() == deviceID)
            {
                outEmP = resEmPs.back().emergency_percentages[direction];
                return true;
            }
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyNode::getEmergency] Request %s[%d] error.", deviceID.c_str(), direction);
        }
        return false;
    }

    /**
     * @brief Request emergency scores from SafetyServer with specific device ID.
     * 
     * @param[in] deviceID either node name or "nearest". Do not pass "all" to prevent unexpected result.
     * @param[out] outEmPs output 8-direction emergencies.
     * @return true if request success. Otherwise, false.
     */
    bool getEmergency(std::string deviceID, std::array<float, 8>& outEmPs)
    {
        if (!this->nodeEnableF_)
            return false;
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReq::Request>();
        request->device_id = deviceID;
        std::vector<std::string> resIDs;
        std::vector<vehicle_interfaces::msg::SurroundEmergency> resEmPs;
        try
        {
            if (this->_reqSurroundEmergency(request, resIDs, resEmPs) && resIDs.back() == deviceID)
            {
                outEmPs = resEmPs.back().emergency_percentages;
                return true;
            }
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyNode::getEmergency] Request %s[all] error.", deviceID.c_str());
        }
        return false;
    }

    /**
     * @brief Request all emergency scores from SafetyServer.
     * 
     * @param[out] emergencies output full emergencies information formed by map which node name and SurroundEmergency were related.
     * @return true if request success. Otherwise, false.
     */
    bool getEmergencies(std::map<std::string, vehicle_interfaces::msg::SurroundEmergency>& emergencies)
    {
        if (!this->nodeEnableF_)
            return false;
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReq::Request>();
        request->device_id = "all";
        std::vector<std::string> resIDs;
        std::vector<vehicle_interfaces::msg::SurroundEmergency> resEmPs;
        try
        {
            if (this->_reqSurroundEmergency(request, resIDs, resEmPs))
            {
                int sz = resIDs.size();
                for (int i = 0; i < sz; i++)
                {
                    emergencies[resIDs.back()] = resEmPs.back();
                    resIDs.pop_back();
                    resEmPs.pop_back();
                }
                return true;
            }
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyNode::getEmergency] Request all[all] error.");
        }
        return false;
    }
};


/**
 * SafetyServer class enables multiple devices to register or request the 8-direction emergency scores (F, B, L, R, FL, FR, BL, BR) of vehicle.
 */
class SafetyServer : public rclcpp::Node
{
private:
    typedef struct _DeviceSurroundEmergency
    {
        vehicle_interfaces::msg::SurroundEmergency emPs;// 8-direction emergency scores and lifetime.
        std::chrono::high_resolution_clock::time_point ts;// Timestamp.
    } DeviceSurroundEmergency;

    enum DeviceSurroundEmergencyCombineStrategy { NEAREST };

    rclcpp::Service<vehicle_interfaces::srv::SafetyReg>::SharedPtr regServer_;
    rclcpp::Service<vehicle_interfaces::srv::SafetyReq>::SharedPtr reqServer_;

    std::map<std::string, DeviceSurroundEmergency> devEmMap_;
    std::mutex devEmMapLock_;

private:
    inline bool _checkDeviceSurroundEmergencyLifetime(const DeviceSurroundEmergency& devEm) const
    {
        if (std::chrono::high_resolution_clock::now() - devEm.ts > std::chrono::duration<float, std::milli>(devEm.emPs.lifetime_ms))
            return false;
        return true;
    }

    DeviceSurroundEmergency _combineDeviceSurroundEmergencies(const DeviceSurroundEmergency& src1, const DeviceSurroundEmergency& src2, DeviceSurroundEmergencyCombineStrategy strategy)
    {
        DeviceSurroundEmergency ret;
        try
        {
            for (int emPIdx = 0; emPIdx < 8; emPIdx++)
            {
                switch (strategy)
                {
                    case NEAREST:
                            ret.emPs.emergency_percentages[emPIdx] = src1.emPs.emergency_percentages[emPIdx] > src2.emPs.emergency_percentages[emPIdx] ? 
                                                                        src1.emPs.emergency_percentages[emPIdx] : src2.emPs.emergency_percentages[emPIdx];
                        break;

                    default:
                        break;
                }
            }
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyServer::_combineDeviceSurroundEmergencies] Caught unknown exception.");
        }
        return ret;
    }

    void _regServerCallback(const std::shared_ptr<vehicle_interfaces::srv::SafetyReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::SafetyReg::Response> response)
    {
        std::lock_guard<std::mutex> devEmMapLocker(this->devEmMapLock_);
        bool resF = false;
        try
        {
            const std::vector<std::string>& ids = request->device_id_vec;
            const std::vector<vehicle_interfaces::msg::SurroundEmergency>& emPs = request->emergency_scores;
            if (ids.size() != emPs.size()) throw "SizeError: device_id_vec and emergency_scores size not fit";
            for (int i = 0; i < ids.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "[SafetyServer::_regServerCallback] Request device: [%s].", ids[i].c_str());
                DeviceSurroundEmergency tmp;
                tmp.emPs = emPs[i];
                tmp.ts = std::chrono::high_resolution_clock::now();
                this->devEmMap_[ids[i]] = tmp;
            }
            resF = true;
        }
        catch (const std::string& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyServer::_regServerCallback] %s", e.c_str());
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyServer::_regServerCallback] %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyServer::_regServerCallback] Caught unknown exception.");
        }
        response->response = resF;
    }

    void _reqServerCallback(const std::shared_ptr<vehicle_interfaces::srv::SafetyReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::SafetyReq::Response> response)
    {
        std::lock_guard<std::mutex> devEmMapLocker(this->devEmMapLock_);

        // Response
        bool resF = true;
        std::vector<std::string> resDeviceVec;
        std::vector<vehicle_interfaces::msg::SurroundEmergency> resEmergeVec;

        // The keys for DeviceSurroundEmergency removal
        std::vector<std::string> rmList;

        try
        {
            if (request->device_id == "all")
            {
                for (const auto& [devID, devEm] : this->devEmMap_)
                {
                    if (this->_checkDeviceSurroundEmergencyLifetime(devEm))
                    {
                        resDeviceVec.push_back(devID);
                        resEmergeVec.push_back(devEm.emPs);
                    }
                    else
                    {
                        rmList.push_back(devID);
                        RCLCPP_INFO(this->get_logger(), "[SafetyServer::_reqServerCallback] Add to removal: %s", devID.c_str());
                    }
                }
            }
            else if (request->device_id == "nearest")
            {
                DeviceSurroundEmergency tmp;
                tmp.emPs.lifetime_ms = -1;// Not used.
                for (const auto& [devID, devEm] : this->devEmMap_)
                {
                    if (this->_checkDeviceSurroundEmergencyLifetime(devEm))
                    {
                        tmp = this->_combineDeviceSurroundEmergencies(tmp, devEm, DeviceSurroundEmergencyCombineStrategy::NEAREST);
                    }
                    else
                    {
                        rmList.push_back(devID);
                        RCLCPP_INFO(this->get_logger(), "[SafetyServer::_reqServerCallback] Add to removal: %s", devID.c_str());
                    }
                }
                resDeviceVec.push_back("nearest");
                resEmergeVec.push_back(tmp.emPs);
            }
            else// nodeName
            {
                if (this->devEmMap_.find(request->device_id) == this->devEmMap_.end())
                    throw "KeyError: device_id not found";
                if (this->_checkDeviceSurroundEmergencyLifetime(this->devEmMap_[request->device_id]))
                {
                    resDeviceVec.push_back(request->device_id);
                    resEmergeVec.push_back(this->devEmMap_[request->device_id].emPs);
                }
                else
                {
                    rmList.push_back(request->device_id);
                    RCLCPP_INFO(this->get_logger(), "[SafetyServer::_reqServerCallback] Add to removal: %s", request->device_id.c_str());
                    resF = false;
                }
            }

            for (auto& key : rmList)
            {
                this->devEmMap_.erase(key);
                RCLCPP_INFO(this->get_logger(), "[SafetyServer::_reqServerCallback] Removed element: %s", key.c_str());
            }
        }
        catch (const std::string& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyServer::_reqServerCallback] %s: %s", e.c_str(), request->device_id.c_str());
            resF = false;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyServer::_reqServerCallback] %s", e.what());
            resF = false;
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[SafetyServer::_reqServerCallback] Caught unknown exception.");
            resF = false;
        }
        response->response = resF;
        response->device_id_vec = resDeviceVec;
        response->emergency_scores = resEmergeVec;
    }

public:
    SafetyServer(const std::string& nodeName, const std::string& serviceName) : rclcpp::Node(nodeName)
    {
        this->regServer_ = this->create_service<vehicle_interfaces::srv::SafetyReg>(serviceName + "_Reg", 
            std::bind(&SafetyServer::_regServerCallback, this, std::placeholders::_1, std::placeholders::_2));

        this->reqServer_ = this->create_service<vehicle_interfaces::srv::SafetyReq>(serviceName + "_Req", 
            std::bind(&SafetyServer::_reqServerCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "[SafetyReqServer] Constructed.");
    }
};

}
