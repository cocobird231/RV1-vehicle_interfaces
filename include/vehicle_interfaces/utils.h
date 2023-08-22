#pragma once
#include <string>
#include <vector>
#include <deque>
#include <map>

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

/*
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