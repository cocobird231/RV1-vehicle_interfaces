#pragma once
#include "rclcpp/rclcpp.hpp"

namespace vehicle_interfaces
{

/* The GenericParams class defines the generic settings for ros2 node and extended services, e.g., 
 * time sync service, safety service and qos service, etc..
 * The configure file can be stored under launch directory in each package.
 */
class GenericParams : public rclcpp::Node
{
public:
    std::string nodeName = "node";
    std::string ns = "";
    uint8_t id = 0;
    std::string qosService = "qos_0";
    std::string qosDirPath = "qos";
    std::string safetyService = "safety_0";
    std::string timesyncService = "timesync_0";
    double timesyncPeriod_ms = 10000.0;
    double timesyncAccuracy_ms = 2.0;

private:
    void _getParams()
    {
        this->get_parameter("nodeName", this->nodeName);
        this->get_parameter("ns", this->ns);
        this->get_parameter("id", this->id);
        this->get_parameter("qosService", this->qosService);
        this->get_parameter("qosDirPath", this->qosDirPath);
        this->get_parameter("safetyService", this->safetyService);
        this->get_parameter("timesyncService", this->timesyncService);
        this->get_parameter("timesyncPeriod_ms", this->timesyncPeriod_ms);
        this->get_parameter("timesyncAccuracy_ms", this->timesyncAccuracy_ms);

        // Change nodeName into "<nodeName>_<id>_node" format
        // this->nodeName += "_" + std::to_string(this->id) + "_node";
    }

public:
    GenericParams(std::string nodeName) : Node(nodeName)
    {
        this->declare_parameter<std::string>("nodeName", this->nodeName);
        this->declare_parameter<std::string>("ns", this->ns);
        this->declare_parameter<int>("id", this->id);
        this->declare_parameter<std::string>("qosService", this->qosService);
        this->declare_parameter<std::string>("qosDirPath", this->qosDirPath);
        this->declare_parameter<std::string>("safetyService", this->safetyService);
        this->declare_parameter<std::string>("timesyncService", this->timesyncService);
        this->declare_parameter<double>("timesyncPeriod_ms", this->timesyncPeriod_ms);
        this->declare_parameter<double>("timesyncAccuracy_ms", this->timesyncAccuracy_ms);
        this->_getParams();
    }
};

}// namespace vehicle_interfaces
