#pragma once
#include <string>
#include <vector>
#include <deque>
#include <map>


namespace vehicle_interfaces
{

template<typename T>
struct ReasonResult
{
    T result;
    std::string reason = "";
    ReasonResult() {}
    ReasonResult(const T& res) : result(res) {}
    ReasonResult(const T& res, const std::string& reason) : result(res), reason(reason) {}
};

}