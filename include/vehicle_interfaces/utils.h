#pragma once

#include <iostream>
#include <memory>
#include <cmath>

#include <mutex>
#include <atomic>
#include <functional>
#include <thread>
#include <chrono>

#include <string>
#include <vector>
#include <deque>
#include <map>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

// Ref: https://stackoverflow.com/a/55475023
#ifndef __has_include
  static_assert(false, "__has_include not supported");
#else
#  if __cplusplus >= 201703L && __has_include(<filesystem>)
#    include <filesystem>
     namespace fs = std::filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
     namespace fs = std::experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
     namespace fs = boost::filesystem;
#  endif
#endif

namespace vehicle_interfaces
{

struct thread_deleter
{
    void operator()(std::thread* t)
    {
        if (t->joinable())
            t->join();
        delete t;
    }
};

using unique_thread = std::unique_ptr<std::thread, thread_deleter>;

template<typename F, typename... args>
unique_thread make_unique_thread(F&& f, args&&... a)
{
    return unique_thread(new std::thread(std::forward<F>(f), std::forward<args>(a)...));
}

using shared_thread = std::shared_ptr<std::thread>;

template<typename F, typename... args>
shared_thread make_shared_thread(F&& f, args&&... a)
{
    return shared_thread(new std::thread(std::forward<F>(f), std::forward<args>(a)...), thread_deleter());
}



class HierarchicalPrint
{
private:
    uint16_t maxHierarchy_;
    std::deque<std::pair<uint16_t, std::string> > que_;

public:
    HierarchicalPrint() : maxHierarchy_(0) {}

    void push(const uint16_t hierarchy, const std::string& str)
    {
        this->que_.push_back({ hierarchy, str });
        this->maxHierarchy_ = std::max(this->maxHierarchy_, hierarchy);
    }

    void push(const uint16_t hierarchy, const char* fmt, ...)
    {
        char printBuf[1024];
        va_list args;
        va_start(args, fmt);
        vsprintf(printBuf, fmt, args);
        va_end(args);
        this->que_.push_back({ hierarchy, printBuf });
        this->maxHierarchy_ = std::max(this->maxHierarchy_, hierarchy);
    }

    void print()
    {
        std::vector<uint16_t> level(this->maxHierarchy_ + 1, 0);

        // Search the hierarchy level from the end of the queue.
        for (std::deque<std::pair<uint16_t, std::string> >::reverse_iterator it{this->que_.rbegin()}; it != this->que_.rend(); ++it)
        {
            const auto& [hierarchy, str] = *it;
            bool changeF = level[hierarchy] == 0;// If the hierarchy level appears for the first time, change the symbol.
            level[hierarchy] = 1;// Mark the hierarchy level as appeared.
            std::string hierarchyStr = "";// Hierarcy prefix string.
            if (hierarchy == 0)// Root.
                hierarchyStr += "---";
            else
            {
                for (int j = 0; j < hierarchy; j++)// Find parent hierarchy level.
                {
                    if (j != 0 && level[j] == 1)// If the hierarchy level appears, add the symbol.
                        hierarchyStr += "|  ";
                    else
                        hierarchyStr += "   ";
                }
                if (changeF)// If the hierarchy level appears for the first time, change the symbol.
                    hierarchyStr += "\\__";
                else
                    hierarchyStr += "|--";
            }
            for (int j = hierarchy + 1; j < level.size(); j++)// Reset the hierarchy level after the current hierarchy level.
                level[j] = 0;
            (*it).second = hierarchyStr + str;// Add the hierarchy prefix string to the message.
        }
        for (const auto& i : this->que_)// Print the message with the hierarchy prefix string.
            printf("%s\n", i.second.c_str());
    }

    void clear() { this->que_.clear(); }
};

template<typename T>
struct ReasonResult
{
    T result;
    std::string reason;
    ReasonResult() {}
    ReasonResult(const T& res) : result(res), reason("") {}
    ReasonResult(const T& res, const std::string& reason) : result(res), reason(reason) {}
};

template<typename T>
struct DescriptiveValue
{
    T value = 0;
    std::string str = "";
    DescriptiveValue() {}
    DescriptiveValue(T value, std::string str) : value(value), str(str) {}
    bool operator==(const DescriptiveValue& tp)
    {
        return this->value == tp.value && this->str == tp.str;
    }
};

template<typename msgT, typename srvT>
class MsgDataSender : public rclcpp::Node
{
private:
    const std::string nodeName_;
    const std::string senderName_;

    std::shared_ptr<rclcpp::Publisher<msgT> > pub_;// Publisher for active mode.
    std::shared_ptr<rclcpp::Service<srvT> > srv_;// Service for passive mode.

    std::function<void(const std::shared_ptr<typename srvT::Request>, std::shared_ptr<typename srvT::Response>)> srvCbFunc_;// Passive mode callback function.
    bool isPassiveModeF_;
    bool srvCbFuncF_;

private:
    void _srvCbFunc(const std::shared_ptr<typename srvT::Request> req, std::shared_ptr<typename srvT::Response> res)
    {
        if (this->srvCbFuncF_ && this->isPassiveModeF_)
            this->srvCbFunc_(req, res);
        else
            RCLCPP_WARN(this->get_logger(), "[MsgDataSender::_srvCbFunc] No callback function for service: %s", this->senderName_.c_str());
    }

public:
    MsgDataSender(const std::string nodeName, const std::string senderName, rclcpp::QoS qos, bool passiveMode) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        senderName_(senderName), 
        isPassiveModeF_(passiveMode), 
        srvCbFuncF_(false)
    {
        if (passiveMode)
            this->srv_ = this->create_service<srvT>(senderName, std::bind(&MsgDataSender::_srvCbFunc, this, std::placeholders::_1, std::placeholders::_2));
        else
            this->pub_ = this->create_publisher<msgT>(senderName, qos);
    }

    void setSrvCbFunc(std::function<void(const std::shared_ptr<typename srvT::Request>, std::shared_ptr<typename srvT::Response>)> srvCbFunc)
    {
        this->srvCbFunc_ = srvCbFunc;
        this->srvCbFuncF_ = true;
    }

    void publish(const msgT& msg)
    {
        if (!this->isPassiveModeF_)
            this->pub_->publish(msg);
        else
            RCLCPP_WARN(this->get_logger(), "[MsgDataSender::publish] This is passive mode. Cannot publish message to service: %s", this->senderName_.c_str());
    }
};

template<typename msgT, typename srvT>
class MsgDataReceiver : public rclcpp::Node
{
private:
    const std::string nodeName_;
    const std::string receiverName_;

    std::shared_ptr<rclcpp::Subscription<msgT> > sub_;// Subscriber for active mode.
    std::shared_ptr<rclcpp::Client<srvT> > cli_;// Client for passive mode.
    rclcpp::Node::SharedPtr cliNode_;// Client node for passive mode.

    std::function<void(const std::shared_ptr<msgT>)> subCbFunc_;// Active mode callback function.
    bool isPassiveModeF_;
    bool subCbFuncF_;

private:
    void _subCbFunc(const std::shared_ptr<msgT> msg)
    {
        if (this->subCbFuncF_ && !this->isPassiveModeF_)
            this->subCbFunc_(msg);
        else
            RCLCPP_WARN(this->get_logger(), "[MsgDataReceiver::_subCbFunc] No callback function for subscription: %s", this->receiverName_.c_str());
    }

public:
    MsgDataReceiver(const std::string nodeName, const std::string receiverName, rclcpp::QoS qos, bool passiveMode) : 
        rclcpp::Node(nodeName), 
        nodeName_(nodeName), 
        receiverName_(receiverName), 
        isPassiveModeF_(passiveMode), 
        subCbFuncF_(false)
    {
        if (passiveMode)
        {
            this->cli_ = this->create_client<srvT>(receiverName);
            this->cliNode_ = std::make_shared<rclcpp::Node>(nodeName + "_cli_node");
        }
        else
            this->sub_ = this->create_subscription<msgT>(receiverName, qos, std::bind(&MsgDataReceiver::_subCbFunc, this, std::placeholders::_1));
    }

    void setSubCbFunc(std::function<void(const std::shared_ptr<msgT>)> subCbFunc)
    {
        this->subCbFunc_ = subCbFunc;
        this->subCbFuncF_ = true;
    }

    bool request(const std::shared_ptr<typename srvT::Request> req, double timeout_ms = 500)
    {
        if (this->isPassiveModeF_)
        {
            auto result = this->cli_->async_send_request(req);
#if ROS_DISTRO == 0
            if (rclcpp::spin_until_future_complete(this->cliNode_, result, std::chrono::duration<double, std::milli>(timeout_ms)) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
            if (rclcpp::spin_until_future_complete(this->cliNode_, result, std::chrono::duration<double, std::milli>(timeout_ms)) == rclcpp::FutureReturnCode::SUCCESS)
#endif
            {
                return true;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[MsgDataReceiver::request] Request service timeout: %s", this->receiverName_.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[MsgDataReceiver::request] This is active mode. Cannot request service: %s", this->receiverName_.c_str());
            return false;
        }
    }
};



void SpinNode(std::shared_ptr<rclcpp::Node> node, std::string threadName)
{
	std::cerr << threadName << " start..." << std::endl;
	rclcpp::spin(node);
	std::cerr << threadName << " exit." << std::endl;
	rclcpp::shutdown();
}

void SpinExecutor(std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec, std::string threadName, double delay_ms)
{
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(delay_ms));
    std::cerr << threadName << " start..." << std::endl;
    exec->spin();
    std::cerr << threadName << " exit." << std::endl;
}

void SpinExecutor2(std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec, std::string threadName, double delay_ms, bool& isEnded)
{
    isEnded = false;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(delay_ms));
    std::cerr << threadName << " start..." << std::endl;
    exec->spin();
    std::cerr << threadName << " exit." << std::endl;
    isEnded = true;
}

bool ConnToService(rclcpp::ClientBase::SharedPtr client, bool& stopF, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000), int retry = 5)
{
    printf("[ConnToService] Connect to service: %s (%d)\n", client->get_service_name(), retry);
    if (retry > 0)
    {
        while (!client->wait_for_service(timeout) && retry-- > 0 && !stopF)
        {
            if (!rclcpp::ok())
            {
                printf("[ConnToService (%s)] Interrupted while waiting for the service. Exiting.\n", client->get_service_name());
                return false;
            }
            printf("[ConnToService (%s)] Service not available, waiting again... (%d)\n", client->get_service_name(), retry);
        }
        if (retry < 0 || stopF)
        {
            printf("[ConnToService (%s)] Connect to service failed.\n", client->get_service_name());
            return false;
        }
        printf("[ConnToService (%s)] Service connected.\n", client->get_service_name());
        return true;
    }
    else
    {
        while (!client->wait_for_service(timeout) && !stopF)
        {
            if (!rclcpp::ok())
            {
                printf("[ConnToService (%s)] Interrupted while waiting for the service. Exiting.\n", client->get_service_name());
                return false;
            }
            printf("[ConnToService (%s)] Service not available, waiting again...\n", client->get_service_name());
        }
        if (stopF)
        {
            printf("[ConnToService (%s)] Connect to service failed.\n", client->get_service_name());
            return false;
        }
        printf("[ConnToService (%s)] Service connected.\n", client->get_service_name());
        return true;
    }
}

std::vector<std::string> split(const std::string& str, const std::string& delimiter)
{
    std::vector<std::string> splitStrings;
    int encodingStep = 0;
    for (size_t i = 0; i < str.length(); i++)
    {
        bool isDelimiter = false;
        for (auto& j : delimiter)
            if (str[i] == j)
            {
                isDelimiter = true;
                break;
            }
        if (!isDelimiter)// Is the spliting character
        {
            encodingStep++;
            if (i == str.length() - 1)
                splitStrings.push_back(str.substr(str.length() - encodingStep, encodingStep));
        }
        else// Is delimiter
        {
            if (encodingStep > 0)// Have characters need to split
                splitStrings.push_back(str.substr(i - encodingStep, encodingStep));
            encodingStep = 0;
        }
    }
    return splitStrings;
}

/**
 * This code is referenced from: https://en.cppreference.com/w/cpp/string/basic_string/replace
*/
std::size_t replace_all(std::string& inout, std::string what, std::string with)
{
    std::size_t count{};
    for (std::string::size_type pos{};
            inout.npos != (pos = inout.find(what.data(), pos, what.length()));
            pos += with.length(), ++count)
    {
        inout.replace(pos, what.length(), with.data(), with.length());
    }
    return count;
}

fs::path GetHomePath()
{
    const static uint16_t BUFF_SIZE = 256;
    char buf[BUFF_SIZE];
    FILE* fp = popen("echo $HOME", "r");
    std::string retStr = "";
    if (fp != NULL)
    {
        while (fgets(buf, BUFF_SIZE, fp) != NULL)
            retStr += buf;
        pclose(fp);
    }
    if (retStr.back() == '\n')
        retStr.pop_back();
    printf("Found home path: %s\n", retStr.c_str());
    return retStr;
}

fs::path GetCurrentPath()
{
    const static uint16_t BUFF_SIZE = 256;
    char buf[BUFF_SIZE];
    FILE* fp = popen("echo $PWD", "r");
    std::string retStr = "";
    if (fp != NULL)
    {
        while (fgets(buf, BUFF_SIZE, fp) != NULL)
            retStr += buf;
        pclose(fp);
    }
    if (retStr.back() == '\n')
        retStr.pop_back();
    printf("Found current path: %s\n", retStr.c_str());
    return retStr;
}

double LinearMapping1d(double value, double from_start, double from_end, double to_start, double to_end)
{
    double a = (to_end - to_start) / (from_end - from_start);
    return a * (value - from_start) + to_start;
}

double GammaCorrection(double value, double gamma, double min_bound = 0, double max_bound = 1)
{
    if (value >= min_bound && value <= max_bound && max_bound > min_bound)
    {
        if (min_bound == 0 && max_bound == 1)
            return std::pow(value, gamma);
        double ratio = (value - min_bound) / (max_bound - min_bound);
        return std::pow(ratio, gamma) * (max_bound - min_bound) + min_bound;
    }
    throw "Boundary value error.";
}

}