#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/srv/safety_reg.hpp"
#include "vehicle_interfaces/srv/safety_req.hpp"

#ifndef ROS_DISTRO// 0: eloquent, 1: foxy, 2: humble
#define ROS_DISTRO 1
#endif

using namespace std::chrono_literals;

class SafetyNode : virtual public rclcpp::Node
{
private:
    std::shared_ptr<rclcpp::Node> regClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::SafetyReg>::SharedPtr regClient_;

    std::shared_ptr<rclcpp::Node> reqClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::SafetyReq>::SharedPtr reqClient_;

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


    void connToService(rclcpp::ClientBase::SharedPtr client)
    {
        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
    }

public:
    SafetyNode(std::string nodeName, std::string safetyServiceName) : Node(nodeName)
    {
        this->regClientNode_ = rclcpp::Node::make_shared(nodeName + "_safetyreg_client");
        this->regClient_ = this->regClientNode_->create_client<vehicle_interfaces::srv::SafetyReg>(safetyServiceName + "_Reg");
        printf("[SafetyNode] Connecting to safetyreg server: %s\n", safetyServiceName.c_str());
        this->connToService(this->regClient_);

        this->reqClientNode_ = rclcpp::Node::make_shared(nodeName + "_safetyreq_client");
        this->reqClient_ = this->reqClientNode_->create_client<vehicle_interfaces::srv::SafetyReq>(safetyServiceName + "_Req");
        printf("[SafetyNode] Connecting to safetyreq server: %s\n", safetyServiceName.c_str());
        this->connToService(this->reqClient_);

        std::this_thread::sleep_for(200ms);
    }

    bool setEmergency(std::string devID, float emP)// Return true if success
    {
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReg::Request>();
        request->device_id = devID;
        request->emergency_percentage = emP;
        auto result = this->regClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 10ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 10ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
                return true;
        }
        return false;
    }

    bool getEmergency(std::string devID, float& outEm)// Return true if success
    {
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReq::Request>();
        request->device_id = devID;
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 10ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 10ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
            {
                outEm = response->emergency_percentages.back();
                return true;
            }
        }
        return false;
    }

    bool getEmergencies(std::map<std::string, float>& outDevEmMap)// Return true if success
    {
        auto request = std::make_shared<vehicle_interfaces::srv::SafetyReq::Request>();
        request->device_id = "all";
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 10ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 10ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
            {
                for (int i = 0; i < response->device_ids.size(); i++)
                    outDevEmMap[response->device_ids[i]] = response->emergency_percentages[i];
                return true;
            }
        }
        return false;
    }
};


class SafetyServer : public rclcpp::Node
{
private:
    rclcpp::Service<vehicle_interfaces::srv::SafetyReg>::SharedPtr regServer_;
    rclcpp::Service<vehicle_interfaces::srv::SafetyReq>::SharedPtr reqServer_;
    std::map<std::string, float> deviceEmergencyMap_;
    std::mutex deviceEmergencyMapLock_;

private:
    void _regServerCallback(const std::shared_ptr<vehicle_interfaces::srv::SafetyReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::SafetyReg::Response> response)
    {
        std::unique_lock<std::mutex> deviceEmergencyMapLocker(this->deviceEmergencyMapLock_, std::defer_lock);
        bool resF = false;
        try
        {
            RCLCPP_INFO(this->get_logger(), "request device: [%s][%f]", request->device_id.c_str(), request->emergency_percentage);
            deviceEmergencyMapLocker.lock();
            this->deviceEmergencyMap_[request->device_id] = request->emergency_percentage;
            deviceEmergencyMapLocker.unlock();
            resF = true;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        catch (...)
        {
            std::cerr << 'Unknown exceptions\n';
        }
        response->response = resF;
    }

    void _reqServerCallback(const std::shared_ptr<vehicle_interfaces::srv::SafetyReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::SafetyReq::Response> response)
    {
        std::unique_lock<std::mutex> deviceEmergencyMapLocker(this->deviceEmergencyMapLock_, std::defer_lock);
        bool resF = false;
        std::vector<std::string> retDeviceVec;
        std::vector<float> retEmergeVec;
        try
        {
            if (request->device_id != "all")
            {
                deviceEmergencyMapLocker.lock();
                if (this->deviceEmergencyMap_.find(request->device_id) == this->deviceEmergencyMap_.end())
                    throw "request device unregistered";
                retDeviceVec.push_back(request->device_id);
                retEmergeVec.push_back(this->deviceEmergencyMap_[request->device_id]);
                deviceEmergencyMapLocker.unlock();
            }
            else
            {
                deviceEmergencyMapLocker.lock();
                for (const auto& i : this->deviceEmergencyMap_)
                {
                    retDeviceVec.push_back(i.first);
                    retEmergeVec.push_back(i.second);
                }
                deviceEmergencyMapLocker.unlock();
            }
            resF = true;
        }
        catch (const std::string& e)
        {
            RCLCPP_INFO(this->get_logger(), "%s: %s", e.c_str(), request->device_id.c_str());
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        catch (...)
        {
            std::cerr << 'Unknown exceptions\n';
        }
        response->response = resF;
        response->device_ids = retDeviceVec;
        response->emergency_percentages = retEmergeVec;
    }

public:
    SafetyServer(std::string nodeName, std::string serviceName) : Node(nodeName)
    {
        this->regServer_ = this->create_service<vehicle_interfaces::srv::SafetyReg>(serviceName + "_Reg", 
            std::bind(&SafetyServer::_regServerCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "[SafetyRegServer] Constructed");

        this->reqServer_ = this->create_service<vehicle_interfaces::srv::SafetyReq>(serviceName + "_Req", 
            std::bind(&SafetyServer::_reqServerCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "[SafetyReqServer] Constructed");
    }
};