#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <regex>

#include <string>
#include <vector>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/dev_info.hpp"
#include "vehicle_interfaces/srv/dev_info_reg.hpp"
#include "vehicle_interfaces/srv/dev_info_req.hpp"

#include <fstream>
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;

namespace vehicle_interfaces
{

/**
 * CompDevInfoStrategy
 * OR: compare two DevInfo objects using OR operator, ignoring empty elements.
 * AND: compare two DevInfo objects using AND operator, considering empty elements.
 * *_IGNORE_*: compare without specific element using specific operator.
*/
enum CompDevInfoStrategy { OR, OR_IGNORE_HOSTNAME, AND, AND_IGNORE_HOSTNAME };

inline bool CompDevInfo(const vehicle_interfaces::msg::DevInfo d1, const vehicle_interfaces::msg::DevInfo d2, CompDevInfoStrategy strategy = CompDevInfoStrategy::OR_IGNORE_HOSTNAME)
{
    switch (strategy)
    {
    case CompDevInfoStrategy::OR:
        return (d1.node_name == d2.node_name && d1.node_name.length() > 0 && d2.node_name.length() > 0) ||
                (d1.hostname == d2.hostname && d1.hostname.length() > 0 && d2.hostname.length() > 0) || 
                (d1.mac_addr == d2.mac_addr && d1.mac_addr.length() > 0 && d2.mac_addr.length() > 0) || 
                (d1.ipv4_addr == d2.ipv4_addr && d1.ipv4_addr.length() > 0 && d2.ipv4_addr.length() > 0) || 
                (d1.ipv6_addr == d2.ipv6_addr && d1.ipv6_addr.length() > 0 && d2.ipv6_addr.length() > 0);
        break;
    
    case CompDevInfoStrategy::OR_IGNORE_HOSTNAME:
        return (d1.node_name == d2.node_name && d1.node_name.length() > 0 && d2.node_name.length() > 0) ||
                (d1.mac_addr == d2.mac_addr && d1.mac_addr.length() > 0 && d2.mac_addr.length() > 0) || 
                (d1.ipv4_addr == d2.ipv4_addr && d1.ipv4_addr.length() > 0 && d2.ipv4_addr.length() > 0) || 
                (d1.ipv6_addr == d2.ipv6_addr && d1.ipv6_addr.length() > 0 && d2.ipv6_addr.length() > 0);
        break;
    
    case CompDevInfoStrategy::AND:
        return (d1.node_name == d2.node_name) &&
                (d1.hostname == d2.hostname) && 
                (d1.mac_addr == d2.mac_addr) && 
                (d1.ipv4_addr == d2.ipv4_addr) && 
                (d1.ipv6_addr == d2.ipv6_addr);
        break;
    
    case CompDevInfoStrategy::AND_IGNORE_HOSTNAME:
        return (d1.node_name == d2.node_name) &&
                (d1.mac_addr == d2.mac_addr) && 
                (d1.ipv4_addr == d2.ipv4_addr) && 
                (d1.ipv6_addr == d2.ipv6_addr);
        break;
    
    default:
        break;
    }
    return false;
}

bool LoadDevInfoFromJSON(const fs::path& filePath, vehicle_interfaces::msg::DevInfo& dInfo)
{
    try
    {
        nlohmann::json json;
        json.update(nlohmann::json::parse(std::ifstream(filePath)));
        dInfo.node_name = json["node_name"];
        dInfo.hostname = json["hostname"];
        dInfo.mac_addr = json["mac_addr"];
        dInfo.ipv4_addr = json["ipv4_addr"];
        dInfo.ipv6_addr = json["ipv6_addr"];
        return true;
    }
    catch(...)
    {
        return false;
    }
}

bool DumpDevInfoToJSON(const fs::path& filePath, const vehicle_interfaces::msg::DevInfo& dInfo)
{
    try
    {
        nlohmann::json json;
        json["node_name"] = dInfo.node_name;
        json["hostname"] = dInfo.hostname;
        json["mac_addr"] = dInfo.mac_addr;
        json["ipv4_addr"] = dInfo.ipv4_addr;
        json["ipv6_addr"] = dInfo.ipv6_addr;

        std::ofstream outFile(filePath);
        outFile << json;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

class DevInfoNode : virtual public rclcpp::Node
{
private:
    // QoS Service Definition
    std::shared_ptr<rclcpp::Node> regClientNode_;// DevInfoReg service node
    rclcpp::Client<vehicle_interfaces::srv::DevInfoReg>::SharedPtr regClient_;// DevInfoReg service client
    std::thread regClientTh_;
    bool regClientStopF_;
    std::mutex regClientLock_;

    // Device info
    std::string nodeName_;
    std::string ifName_;
    std::string hostname_;
    std::string ipv4Addr_;
    std::string ipv6Addr_;
    std::string macAddr_;
    bool reqEnableF_;

    // Node enable
    std::atomic<bool> nodeEnableF_;

private:
    void _waitService()
    {
        try
        {
            while (!this->_getHostname() && !this->regClientStopF_)
                std::this_thread::sleep_for(1000ms);
            while (!this->_getIPv4Addr() && !this->regClientStopF_)
                std::this_thread::sleep_for(1000ms);
            while (!this->_getMACAddr() && !this->regClientStopF_)
                std::this_thread::sleep_for(1000ms);

            vehicle_interfaces::ConnToService(this->regClient_, this->regClientStopF_, std::chrono::milliseconds(5000), -1);
            this->reqEnableF_ = true;

            while (!this->regDevInfo() && !this->regClientStopF_)
                std::this_thread::sleep_for(1000ms);
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[DevInfoNode::_waitService] Caught unexpected errors.");
        }
    }

    bool _getHostname()
    {
        char buf[128];
        bool retF = false;

        // Get hostname
        FILE* fp = popen("hostname | awk '{print \"^\"$0\"!\"}'", "r");
        if (fp != NULL)
        {
            while (fgets(buf, 128, fp) != NULL)
            {
                std::string recvStr(buf);
                recvStr = recvStr.substr(recvStr.find('^') + 1, recvStr.rfind('!') - 1);
                if (recvStr.length() > 0)
                {
                    this->hostname_ = recvStr;
                    retF = true;
                }
            }
            pclose(fp);
        }
        return retF;
    }

    bool _getIPv4Addr()
    {
        char buf[128];
        char cmdBuf[128];
        bool retF = false;

        // Get IP address
        sprintf(cmdBuf, "ip addr show dev %s | grep -Po \"(?<=inet )((\\d{1,3}\\.){3}\\d{1,3})\" | awk '{print \"^\"$0\"!\"}'", this->ifName_.c_str());
        FILE* fp = popen(cmdBuf, "r");
        if (fp != NULL)
        {
            while (fgets(buf, 128, fp) != NULL)
            {
                std::string recvStr(buf);
                recvStr = recvStr.substr(recvStr.find('^') + 1, recvStr.rfind('!') - 1);
                if (recvStr.length() > 0)
                {
                    std::smatch match;
                    std::regex_match(recvStr, match, std::regex("(\\d{1,3}\\.){3}\\d{1,3}"));
                    if (match.size() > 0)
                    {
                        this->ipv4Addr_ = recvStr;
                        retF = true;
                    }
                }
            }
            pclose(fp);
        }
        return retF;
    }

    bool _getMACAddr()
    {
        char buf[128];
        char cmdBuf[128];
        bool retF = false;

        // Get MAC address
        sprintf(cmdBuf, "ip addr show dev %s | grep -Po \"(?<=link/ether )(([A-Za-z0-9]{2}:){5}[A-Za-z0-9]{2})(?= brd)\" | awk '{print \"^\"$0\"!\"}'", this->ifName_.c_str());
        FILE* fp = popen(cmdBuf, "r");
        if (fp != NULL)
        {
            while (fgets(buf, 128, fp) != NULL)
            {
                std::string recvStr(buf);
                recvStr = recvStr.substr(recvStr.find('^') + 1, recvStr.rfind('!') - 1);
                if (recvStr.length() > 0)
                {
                    std::smatch match;
                    std::regex_match(recvStr, match, std::regex("([A-Za-z0-9]{2}:){5}[A-Za-z0-9]{2}"));
                    if (match.size() > 0)
                    {
                        this->macAddr_ = recvStr;
                        retF = true;
                    }
                }
            }
            pclose(fp);
        }
        return retF;
    }

public:
    DevInfoNode(const std::string& nodeName, const std::string& devInfoServiceName, const std::string& devInterface) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        ifName_(devInterface), 
        nodeEnableF_(false), 
        regClientStopF_(false), 
        reqEnableF_(false)
    {
        if (devInfoServiceName == "")
            return;
        
        this->regClientNode_ = rclcpp::Node::make_shared(nodeName + "_devinforeg_client");
        this->regClient_ = this->regClientNode_->create_client<vehicle_interfaces::srv::DevInfoReg>(devInfoServiceName + "_Reg");
        this->nodeEnableF_ = true;

        this->regClientTh_ = std::thread(&DevInfoNode::_waitService, this);
    }

    ~DevInfoNode()
    {
        this->reqEnableF_ = false;
        this->regClientStopF_ = true;
        this->regClientTh_.join();
    }

    bool regDevInfo()
    {
        if (!this->nodeEnableF_ && !this->reqEnableF_)
            return false;
        
        std::lock_guard<std::mutex> locker(this->regClientLock_);
        
        if ((this->ipv4Addr_.length() <= 0 && this->ipv6Addr_.length() <= 0) || this->macAddr_.length() <= 0 || this->hostname_.length() <= 0)
            return false;
        
        // Add namespace into nodeName
        std::string nodeName = this->nodeName_;
        if (strlen(this->get_namespace()) > 0)// Namespace exists
        {
            if (nodeName.find(this->get_namespace()) == std::string::npos)
            {
                if (nodeName[0] == '/')
                    nodeName = std::string(this->get_namespace()) + nodeName;
                else
                    nodeName = std::string(this->get_namespace()) + "/" + nodeName;
            }
        }

        auto msg = vehicle_interfaces::msg::DevInfo();
        msg.node_name = nodeName;
        msg.hostname = this->hostname_;
        msg.mac_addr = this->macAddr_;
        msg.ipv4_addr = this->ipv4Addr_;
        msg.ipv6_addr = this->ipv6Addr_;

        auto request = std::make_shared<vehicle_interfaces::srv::DevInfoReg::Request>();
        request->dev_info = msg;
        auto result = this->regClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            return res->response;
        }
        return false;
    }
};

class DevInfoServer : public rclcpp::Node
{
private:
    rclcpp::Service<vehicle_interfaces::srv::DevInfoReg>::SharedPtr regServer_;
    rclcpp::Service<vehicle_interfaces::srv::DevInfoReq>::SharedPtr reqServer_;

    std::map<std::string, vehicle_interfaces::msg::DevInfo> devInfoList_;// {nodeName, DevInfo}
    std::mutex devInfoListLock_;
    
    fs::path devInfoDirPath_;

    // Controllable parameters

    enum DevInfoStoreStrategy { CONFLICT_OVERWRITE, CONFLICT_STRICT_OVERWRITE, CONFLICT_IGNORE } storeStrategy_;

private:
    void _regServerCallback(const std::shared_ptr<vehicle_interfaces::srv::DevInfoReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::DevInfoReg::Response> response)
    {
        std::lock_guard<std::mutex> locker(this->devInfoListLock_);
        bool resF = false;
        try
        {
            RCLCPP_INFO(this->get_logger(), "[DevInfoServer::_regServerCallback] Register: %s[%s]\t%s\t%s.", 
                        request->dev_info.node_name.c_str(), 
                        request->dev_info.hostname.c_str(), 
                        request->dev_info.ipv4_addr.c_str(), 
                        request->dev_info.mac_addr.c_str());// node[host] ipv4/mac
            
            std::vector<std::string> conflictVec;
            for (const auto& [n, dInfo] : this->devInfoList_)
            {
                if (CompDevInfo(dInfo, request->dev_info, CompDevInfoStrategy::OR_IGNORE_HOSTNAME))
                {
                    RCLCPP_WARN(this->get_logger(), "[DevInfoServer::_regServerCallback] Conflict: %s[%s]\t%s\t%s.", 
                                dInfo.node_name.c_str(), 
                                dInfo.hostname.c_str(), 
                                dInfo.ipv4_addr.c_str(), 
                                dInfo.mac_addr.c_str());// node[host] ipv4/mac
                    conflictVec.push_back(n);
                }
            }

            if (this->storeStrategy_ != DevInfoStoreStrategy::CONFLICT_IGNORE)
            {
                for (auto& conflictKey : conflictVec)
                {
                    if (CompDevInfo(this->devInfoList_[conflictKey], request->dev_info, CompDevInfoStrategy::AND_IGNORE_HOSTNAME))// Same
                        continue;
                    
                    if (conflictKey == request->dev_info.node_name)// Overwrite
                        continue;
                    
                    if (this->storeStrategy_ == DevInfoStoreStrategy::CONFLICT_STRICT_OVERWRITE)// Strict overwrite
                    {
                        char cmdBuf[128];

                        // Remove conflict file
                        std::string fn = conflictKey;
                        vehicle_interfaces::replace_all(fn, "/", "_");
                        sprintf(cmdBuf, "rm -rf %s", (this->devInfoDirPath_ / (fn + ".json")).generic_string().c_str());
                        system(cmdBuf);
                        DumpDevInfoToJSON(this->devInfoDirPath_ / (fn + ".json.conflict"), this->devInfoList_[conflictKey]);
                        RCLCPP_WARN(this->get_logger(), "[DevInfoServer::_regServerCallback] Move conflict file %s to %s.", 
                                    (this->devInfoDirPath_ / (fn + ".json")).generic_string().c_str(), 
                                    (this->devInfoDirPath_ / (fn + ".json.conflict")).generic_string().c_str());
                        
                        // Remove conflict DevInfo
                        this->devInfoList_.erase(conflictKey);
                        RCLCPP_WARN(this->get_logger(), "[DevInfoServer::_regServerCallback] Remove conflict element.");
                        continue;
                    }
                }
                this->devInfoList_[request->dev_info.node_name] = request->dev_info;

                // Dump file
                std::string fn = request->dev_info.node_name;
                vehicle_interfaces::replace_all(fn, "/", "_");
                DumpDevInfoToJSON(this->devInfoDirPath_ / (fn + ".json"), request->dev_info);
            }
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

    void _reqServerCallback(const std::shared_ptr<vehicle_interfaces::srv::DevInfoReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::DevInfoReq::Response> response)
    {
        std::lock_guard<std::mutex> locker(this->devInfoListLock_);
        bool resF = false;
        std::string tag;
        try
        {
            if (request->dev_info.node_name == "all")
            {
                response->dev_info_vec.clear();
                for (const auto& [n, dInfo] : this->devInfoList_)
                    response->dev_info_vec.push_back(dInfo);
                resF = true;
            }
            else
            {
                for (const auto& [n, dInfo] : this->devInfoList_)
                {
                    if (CompDevInfo(dInfo, request->dev_info, OR_IGNORE_HOSTNAME))
                    {
                        tag = n;
                        resF = true;
                        break;
                    }
                }
                if (resF)
                    response->dev_info_vec = { this->devInfoList_[tag] };
            }
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

    void _loadDevInfoFile()
    {
        std::lock_guard<std::mutex> locker(this->devInfoListLock_);
        for (auto& fp : fs::directory_iterator(this->devInfoDirPath_))
        {
            if (fp.path().extension() == ".conflict")
                continue;
            auto msg = vehicle_interfaces::msg::DevInfo();
            if (LoadDevInfoFromJSON(fp, msg))
                this->devInfoList_[msg.node_name] = msg;
        }
    }

public:
    DevInfoServer(const std::string& nodeName, const std::string& serviceName, const std::string& devInfoDirPath, const DevInfoStoreStrategy strategy = DevInfoStoreStrategy::CONFLICT_STRICT_OVERWRITE) : 
        rclcpp::Node(nodeName), 
        devInfoDirPath_(devInfoDirPath), 
        storeStrategy_(strategy)
    {
        {// Check DevInfo directory
            char buf[512];
            sprintf(buf, "mkdir -p %s", this->devInfoDirPath_.generic_string().c_str());
            system(buf);
        }

        this->_loadDevInfoFile();

        this->regServer_ = this->create_service<vehicle_interfaces::srv::DevInfoReg>(serviceName + "_Reg", 
            std::bind(&DevInfoServer::_regServerCallback, this, std::placeholders::_1, std::placeholders::_2));

        this->reqServer_ = this->create_service<vehicle_interfaces::srv::DevInfoReq>(serviceName + "_Req", 
            std::bind(&DevInfoServer::_reqServerCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "[DevInfoServer] Constructed.");
        
        this->printDevInfoList();
    }

    void printDevInfoList()
    {
        std::lock_guard<std::mutex> locker(this->devInfoListLock_);
        for (const auto& [n, dInfo] : this->devInfoList_)
        {
            printf("%s[%s]\t%s/%s\n", 
                    dInfo.node_name.c_str(), 
                    dInfo.hostname.c_str(), 
                    dInfo.ipv4_addr.c_str(), 
                    dInfo.mac_addr.c_str());
        }
    }
};

}
