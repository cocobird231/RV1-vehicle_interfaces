#pragma once
#include <iostream>
#include <memory>

#include <mutex>
#include <atomic>
#include <functional>
#include <thread>

#include <string>
#include <vector>
#include <deque>
#include <map>

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

class Timer
{
private:
    std::chrono::high_resolution_clock::duration interval_;
    std::chrono::high_resolution_clock::time_point st_;

    std::atomic<bool> activateF_;
    std::atomic<bool> exitF_;
    std::atomic<bool> funcCallableF_;
    std::function<void()> func_;

    std::thread timerTH_;
    std::thread callbackTH_;

private:
    void _timer_fixedRate()
    {
        while (!this->exitF_)
        {
            try
            {
                if (!this->exitF_ && this->activateF_)
                {
                    auto tickTimePoint = this->st_ + this->interval_;
                    this->st_ = tickTimePoint;

                    while (!this->exitF_ && this->activateF_ && (std::chrono::high_resolution_clock::now() < tickTimePoint))
                        std::this_thread::yield();
                    if (!this->exitF_ && this->activateF_ && this->funcCallableF_)
                    {
                        if (this->callbackTH_.joinable())
                            this->callbackTH_.join();
                        this->callbackTH_ = std::thread(&Timer::_tick, this);
                    }
                }
                std::this_thread::yield();
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        if (this->callbackTH_.joinable())
            this->callbackTH_.join();
    }

    void _tick()
    {
        this->funcCallableF_ = false;
        this->func_();
        this->funcCallableF_ = true;
    }

public:
    Timer(double interval_ms, const std::function<void()>& callback) : activateF_(false), exitF_(false), funcCallableF_(true)
    {
        this->interval_ = std::chrono::high_resolution_clock::duration(std::chrono::nanoseconds(static_cast<uint64_t>(interval_ms * 1000000)));
        this->func_ = callback;
        this->st_ = std::chrono::high_resolution_clock::now();
        this->timerTH_ = std::thread(&Timer::_timer_fixedRate, this);
    }

    ~Timer()
    {
        this->destroy();
    }

    void start()
    {
        this->st_ = std::chrono::high_resolution_clock::now();
        this->activateF_ = true;
    }

    void stop() { this->activateF_ = false; }

    void setInterval(double interval_ms)
    {
        bool preState = this->activateF_;
        this->stop();
        this->interval_ = std::chrono::high_resolution_clock::duration(std::chrono::nanoseconds(static_cast<uint64_t>(interval_ms * 1000000)));
        if (preState)
            this->start();
    }

    void destroy()
    {
        this->activateF_ = false;
        this->exitF_ = true;
        if (this->timerTH_.joinable())
            this->timerTH_.join();
    }
};

template<typename T>
struct ReasonResult
{
    T result;
    std::string reason = "";
    ReasonResult() {}
    ReasonResult(const T& res) : result(res) {}
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

void SpinNode(std::shared_ptr<rclcpp::Node> node, std::string threadName)
{
	std::cerr << threadName << " start..." << std::endl;
	rclcpp::spin(node);
	std::cerr << threadName << " exit." << std::endl;
	rclcpp::shutdown();
}

bool ConnToService(rclcpp::ClientBase::SharedPtr client, bool& stopF, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000), int retry = 5)
{
    printf("Connect to service: %s (%d)\n", client->get_service_name(), retry);
    if (retry > 0)
    {
        while (!client->wait_for_service(timeout) && retry-- > 0 && !stopF)
        {
            if (!rclcpp::ok())
            {
                printf("[ConnToService] Interrupted while waiting for the service. Exiting.\n");
                return false;
            }
            printf("[ConnToService] Service not available, waiting again... (%d)\n", retry);
        }
        if (retry < 0 || stopF)
        {
            printf("[ConnToService] Connect to service failed.");
            return false;
        }
        printf("[ConnToService] Service connected.");
        return true;
    }
    else
    {
        while (!client->wait_for_service(timeout) && !stopF)
        {
            if (!rclcpp::ok())
            {
                printf("[ConnToService] Interrupted while waiting for the service. Exiting.\n");
                return false;
            }
            printf("[ConnToService] Service not available, waiting again...\n");
        }
        if (stopF)
        {
            printf("[ConnToService] Connect to service failed.");
            return false;
        }
        printf("[ConnToService] Service connected.");
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

}