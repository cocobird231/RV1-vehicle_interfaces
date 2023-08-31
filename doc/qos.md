# qos.h Document

*`Updated: 2023/08/30`*
*`Version: v1.1-dev`*

## MACRO
```cpp
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
#include "vehicle_interfaces/msg/qos_profile.hpp"
#include "vehicle_interfaces/msg/qos_update.hpp"
#include "vehicle_interfaces/srv/qos_reg.hpp"
#include "vehicle_interfaces/srv/qos_req.hpp"

#include <fstream>
#include "nlohmann/json.hpp"

#ifndef ROS_DISTRO// 0: eloquent, 1: foxy, 2: humble
#define ROS_DISTRO 2
#endif
```

## QoSUpdateNode
### Member Functions


## QoSServer

### Member Functions

## Functions
### Time Struct Transform
- CvtMsgToRMWTime
- CvtRMWTimeToMsg
- CompRMWTime

### QoS Struct Transform
- CvtRMWQoSToMsg
- CvtMsgToRMWQoS
- DumpRMWQoSToJSON
- LoadRMWQoSFromJSON
- CvtRMWQoSToRclQoS
- getQoSProfEnumName

## Examples

### Publisher
```cpp
// Examples for QoS updated publisher
class SamplePublisher : public vehicle_interfaces::VehicleServiceNode
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

private:
    void _timerCallback() {} // Publish timer

    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Get qmap size: %d", qmap.size());
        for (const auto& [k, v] : qmap)
        {
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher::_qosCallback] Get qmap[%s]", k.c_str());
            if (k == TOPIC_NAME || k == (std::string)this->get_namespace() + "/" + TOPIC_NAME)
            {
                // Stop timer
                this->timer_->cancel();
                while (!this->timer_->is_canceled())
                    std::this_thread::sleep_for(50ms);
                
                // Destruct timer
                this->timer_.reset();// Call destructor

                // Destruct publisher
                this->pub_.reset();// Call destructor

                // Create new publisher
                this->pub_ = this->create_publisher<std_msgs::msg::String>(TOPIC_NAME, *v);

                // Create new timer
                this->timer_ = this->create_wall_timer(50ms, std::bind(&SamplePublisher::_timerCallback, this));
            }
        }
    }

public:
    SamplePublisher(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::VehicleServiceNode(gParams), 
        rclcpp::Node(NODE_NAME)
    {
        // Add callback function to get latest update QoS map
        this->addQoSCallbackFunc(std::bind(&SamplePublisher::_qosCallback, this, std::placeholders::_1));

        // Add topic name to qos update track list. The return should be {TOPIC_NAME : rclcpp::QoS}.
        vehicle_interfaces::QoSPair qpair = this->addQoSTracking(TOPIC_NAME);
        if (qpair.first == "")
            RCLCPP_ERROR(this->get_logger(), "[SamplePublisher] Failed to add topic to track list: %s", TOPIC_NAME);
        else
        {
            RCLCPP_INFO(this->get_logger(), "[SamplePublisher] QoS profile [%s]:\nDepth: %d\nReliability: %d", 
                qpair.first.c_str(), qpair.second->get_rmw_qos_profile().depth, qpair.second->get_rmw_qos_profile().reliability);
        }
        
        // Same as official examples
        this->pub_ = this->create_publisher<std_msgs::msg::String>(TOPIC_NAME, *qpair.second);
        this->timer_ = this->create_wall_timer(20ms, std::bind(&SamplePublisher::_timerCallback, this));
    }
};
```