#pragma once
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/utils.h"

/** msg */
#include "vehicle_interfaces/msg/chassis.hpp"
#include "vehicle_interfaces/msg/steering_wheel.hpp"
/** msg_content */
#include "vehicle_interfaces/msg/chassis_info.hpp"
#include "vehicle_interfaces/msg/control_chassis.hpp"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/msg/control_server_status.hpp"
#include "vehicle_interfaces/msg/control_steering_wheel.hpp"
#include "vehicle_interfaces/msg/data_server_status.hpp"
#include "vehicle_interfaces/msg/mapping_data.hpp"
#include "vehicle_interfaces/msg/motor_value_range.hpp"
/** msg_geo */
#include "vehicle_interfaces/msg/bbox2d.hpp"
#include "vehicle_interfaces/msg/point2d.hpp"
#include "vehicle_interfaces/msg/point2f.hpp"
#include "vehicle_interfaces/msg/size2f.hpp"

namespace vehicle_interfaces
{



/**
 * ================================================================
 * Namespace msg_show.
 * ================================================================
 */

namespace msg_show
{

/** msg_geo */
class Point2d
{
public:
    static HierarchicalPrint hprint(const vehicle_interfaces::msg::Point2d& msg)
    {
        HierarchicalPrint ret;
        ret.push(0, "<Point2d>");
        ret.push(1, "x: " + std::to_string(msg.x));
        ret.push(1, "y: " + std::to_string(msg.y));
        return ret;
    }

    static std::string to_string(const vehicle_interfaces::msg::Point2d& msg)
    {
        return "(" + std::to_string(msg.x) + ", " + std::to_string(msg.y) + ")";
    }
};

/** msg_content */
class MappingData
{
public:
    static HierarchicalPrint hprint(const vehicle_interfaces::msg::MappingData& msg)
    {
        int maxArrPrint = 5;
        HierarchicalPrint ret;
        ret.push(0, "<MappingData>");
        ret.push(1, "input_vec (" + std::to_string(msg.input_vec.size()) + ")");
        for (size_t i = 0; i < msg.input_vec.size(); i++)
        {
            if (i >= maxArrPrint)
            {
                ret.push(2, "...");
                break;
            }
            ret.push(2, "[" + std::to_string(i) + "]: " + std::to_string(msg.input_vec[i]));
        }

        ret.push(1, "output_vec (" + std::to_string(msg.output_vec.size()) + ")");
        for (size_t i = 0; i < msg.output_vec.size(); i++)
        {
            if (i >= maxArrPrint)
            {
                ret.push(2, "...");
                break;
            }
            ret.push(2, "[" + std::to_string(i) + "]: " + std::to_string(msg.output_vec[i]));
        }
        return ret;
    }
};

class MotorValueRange
{
public:
    static HierarchicalPrint hprint(const vehicle_interfaces::msg::MotorValueRange& msg)
    {
        HierarchicalPrint ret;
        ret.push(0, "<MotorValueRange>");
        ret.push(1, "min: " + std::to_string(msg.min));
        ret.push(1, "max: " + std::to_string(msg.max));
        ret.push(1, "init: " + std::to_string(msg.init));
        return ret;
    }
};

class ControllerInfo
{
public:
    static std::string getMsgTypeStr(uint8_t type)
    {
        switch (type)
        {
            case vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL:
                return "MSG_TYPE_STEERING_WHEEL";
            case vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS:
                return "MSG_TYPE_CHASSIS";
            default:
                return "Unknown";
        }
    }

    static std::string getControllerModeStr(uint8_t mode)
    {
        switch (mode)
        {
            case vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_TOPIC:
                return "CONTROLLER_MODE_TOPIC";
            case vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_SERVICE:
                return "CONTROLLER_MODE_SERVICE";
            default:
                return "Unknown";
        }
    }

    static std::string getPubTypeStr(uint8_t type)
    {
        switch (type)
        {
            case vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_NONE:
                return "PUB_TYPE_NONE";
            case vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER_CLIENT:
                return "PUB_TYPE_CONTROLLER_CLIENT";
            case vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER_SERVER:
                return "PUB_TYPE_CONTROLLER_SERVER";
            case vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_BOTH:
                return "PUB_TYPE_BOTH";
            default:
                return "Unknown";
        }
    }

    static HierarchicalPrint hprint(const vehicle_interfaces::msg::ControllerInfo& msg)
    {
        HierarchicalPrint ret;
        ret.push(0, "<ControllerInfo>");
        ret.push(1, "msg_type: " + getMsgTypeStr(msg.msg_type));
        ret.push(1, "controller_mode: " + getControllerModeStr(msg.controller_mode));
        ret.push(1, "node_name: " + msg.node_name);
        ret.push(1, "service_name: " + msg.service_name);
        ret.push(1, "timeout_ms: " + std::to_string(msg.timeout_ms) + " ms");
        ret.push(1, "period_ms: " + std::to_string(msg.period_ms) + " ms");
        ret.push(1, "privilege: " + std::to_string(msg.privilege));
        ret.push(1, "pub_type: " + getPubTypeStr(msg.pub_type));
        return ret;
    }
};

class ChassisInfo
{
public:
    static std::string getVehicleTypeStr(uint8_t type)
    {
        switch (type)
        {
            case vehicle_interfaces::msg::ChassisInfo::VEHICLE_TYPE_1D1S1B:
                return "VEHICLE_TYPE_1D1S1B";
            case vehicle_interfaces::msg::ChassisInfo::VEHICLE_TYPE_4D4S4B:
                return "VEHICLE_TYPE_4D4S4B";
            case vehicle_interfaces::msg::ChassisInfo::VEHICLE_TYPE_8D8S8B:
                return "VEHICLE_TYPE_8D8S8B";
            default:
                return "Unknown";
        }
    }

    static HierarchicalPrint hprint(const vehicle_interfaces::msg::ChassisInfo& msg)
    {
        HierarchicalPrint ret;
        ret.push(0, "<ChassisInfo>");
        ret.push(1, "vehicle_type: " + getVehicleTypeStr(msg.vehicle_type));
        ret.push(1, "wheel_position");
        for (int i = 0; i < msg.wheel_position.size(); i++)
            ret.push(2, "[" + std::to_string(i) + "]: " + Point2d::to_string(msg.wheel_position[i]));
        ret.push(1, "chassis_centroid: " + Point2d::to_string(msg.chassis_centroid));
        ret.push(1, "chassis_cg: " + Point2d::to_string(msg.chassis_cg));

        ret.push(1, "drive_motor_mapping_vec");
        for (int i = 0; i < msg.drive_motor_mapping_vec.size(); i++)
        {
            ret.push(2, "[" + std::to_string(i) + "]");
            ret.append(3, MappingData::hprint(msg.drive_motor_mapping_vec[i]));
        }
        ret.push(1, "steering_motor_mapping_vec");
        for (int i = 0; i < msg.steering_motor_mapping_vec.size(); i++)
        {
            ret.push(2, "[" + std::to_string(i) + "]");
            ret.append(3, MappingData::hprint(msg.steering_motor_mapping_vec[i]));
        }
        ret.push(1, "brake_motor_mapping_vec");
        for (int i = 0; i < msg.brake_motor_mapping_vec.size(); i++)
        {
            ret.push(2, "[" + std::to_string(i) + "]");
            ret.append(3, MappingData::hprint(msg.brake_motor_mapping_vec[i]));
        }

        ret.push(1, "drive_motor_correction_vec");
        for (int i = 0; i < msg.drive_motor_correction_vec.size(); i++)
            ret.push(2, "[" + std::to_string(i) + "]: " + std::to_string(msg.drive_motor_correction_vec[i]));
        ret.push(1, "steering_motor_correction_vec");
        for (int i = 0; i < msg.steering_motor_correction_vec.size(); i++)
            ret.push(2, "[" + std::to_string(i) + "]: " + std::to_string(msg.steering_motor_correction_vec[i]));
        ret.push(1, "brake_motor_correction_vec");
        for (int i = 0; i < msg.brake_motor_correction_vec.size(); i++)
            ret.push(2, "[" + std::to_string(i) + "]: " + std::to_string(msg.brake_motor_correction_vec[i]));

        ret.push(1, "drive_motor_pwm_value");
        ret.append(2, MotorValueRange::hprint(msg.drive_motor_pwm_value));
        ret.push(1, "drive_motor_rpm_value");
        ret.append(2, MotorValueRange::hprint(msg.drive_motor_rpm_value));
        ret.push(1, "steering_motor_pwm_value");
        ret.append(2, MotorValueRange::hprint(msg.steering_motor_pwm_value));
        ret.push(1, "steering_motor_angle_value");
        ret.append(2, MotorValueRange::hprint(msg.steering_motor_angle_value));
        ret.push(1, "brake_motor_pwm_value");
        ret.append(2, MotorValueRange::hprint(msg.brake_motor_pwm_value));
        ret.push(1, "brake_motor_psi_value");
        ret.append(2, MotorValueRange::hprint(msg.brake_motor_psi_value));
        return ret;
    }
};

class ControlServerStatus
{
public:
    static std::string getControllerActionStr(uint8_t act)
    {
        switch (act)
        {
            case vehicle_interfaces::msg::ControlServerStatus::CONTROLLER_ACTION_NONE:
                return "CONTROLLER_ACTION_NONE";
            case vehicle_interfaces::msg::ControlServerStatus::CONTROLLER_ACTION_SELECT:
                return "CONTROLLER_ACTION_SELECT";
            case vehicle_interfaces::msg::ControlServerStatus::CONTROLLER_ACTION_REMOVE:
                return "CONTROLLER_ACTION_REMOVE";
            default:
                return "Unknown";
        }
    }

    static std::string getServerActionStr(uint8_t act)
    {
        switch (act)
        {
            case vehicle_interfaces::msg::ControlServerStatus::SERVER_ACTION_NONE:
                return "SERVER_ACTION_NONE";
            case vehicle_interfaces::msg::ControlServerStatus::SERVER_ACTION_SET_TIMER:
                return "SERVER_ACTION_SET_TIMER";
            case vehicle_interfaces::msg::ControlServerStatus::SERVER_ACTION_SET_PERIOD:
                return "SERVER_ACTION_SET_PERIOD";
            default:
                return "Unknown";
        }
    }

    static std::string getTimerStatusStr(uint8_t act)
    {
        switch (act)
        {
            case vehicle_interfaces::msg::ControlServerStatus::TIMER_STATUS_NONE:
                return "SERVER_SCAN_TIMER_STATUS_NONE";
            case vehicle_interfaces::msg::ControlServerStatus::TIMER_STATUS_STOP:
                return "SERVER_SCAN_TIMER_STATUS_STOP";
            case vehicle_interfaces::msg::ControlServerStatus::TIMER_STATUS_START:
                return "SERVER_SCAN_TIMER_STATUS_START";
            default:
                return "Unknown";
        }
    }

    static std::string getChassisActionStr(uint8_t act)
    {
        switch (act)
        {
            case vehicle_interfaces::msg::ControlServerStatus::CHASSIS_ACTION_NONE:
                return "CHASSIS_ACTION_NONE";
            case vehicle_interfaces::msg::ControlServerStatus::CHASSIS_ACTION_SET:
                return "CHASSIS_ACTION_STOP";
            default:
                return "Unknown";
        }
    }

    static HierarchicalPrint hprint(const vehicle_interfaces::msg::ControlServerStatus& msg)
    {
        HierarchicalPrint ret;
        ret.push(0, "<ControlServerStatus>");
        ret.push(1, "controller_action: " + getControllerActionStr(msg.controller_action));
        ret.push(1, "controller_service_name: " + msg.controller_service_name);
        ret.push(1, "server_action: " + getServerActionStr(msg.server_action));
        ret.push(1, "output_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_output_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_output_period_ms) + " ms");
        ret.push(1, "safety_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_safety_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_safety_period_ms) + " ms");
        ret.push(1, "idclient_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_idclient_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_idclient_period_ms) + " ms");
        ret.push(1, "publish_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_publish_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_publish_period_ms) + " ms");
        ret.push(1, "chassis_action: " + getChassisActionStr(msg.chassis_action));
        ret.push(1, "chassis_info");
        ret.append(2, ChassisInfo::hprint(msg.chassis_info));

        return ret;
    }
};

class DataServerStatus
{
public:
    static std::string getServerActionStr(uint8_t act)
    {
        switch (act)
        {
            case vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_NONE:
                return "SERVER_ACTION_NONE";
            case vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_STOP:
                return "SERVER_ACTION_STOP";
            case vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_START:
                return "SERVER_ACTION_START";
            case vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_SET_TIMER:
                return "SERVER_ACTION_SET_TIMER";
            case vehicle_interfaces::msg::DataServerStatus::SERVER_ACTION_SET_PERIOD:
                return "SERVER_ACTION_SET_PERIOD";
            default:
                return "Unknown";
        }
    }

    static std::string getTimerStatusStr(uint8_t act)
    {
        switch (act)
        {
            case vehicle_interfaces::msg::DataServerStatus::TIMER_STATUS_NONE:
                return "SERVER_SCAN_TIMER_STATUS_NONE";
            case vehicle_interfaces::msg::DataServerStatus::TIMER_STATUS_STOP:
                return "SERVER_SCAN_TIMER_STATUS_STOP";
            case vehicle_interfaces::msg::DataServerStatus::TIMER_STATUS_START:
                return "SERVER_SCAN_TIMER_STATUS_START";
            default:
                return "Unknown";
        }
    }

    static HierarchicalPrint hprint(const vehicle_interfaces::msg::DataServerStatus& msg)
    {
        HierarchicalPrint ret;
        ret.push(0, "<DataServerStatus>");
        ret.push(1, "server_action: " + getServerActionStr(msg.server_action));
        ret.push(1, "scan_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_scan_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_scan_period_ms) + " ms");
        ret.push(1, "sample_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_sample_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_sample_period_ms) + " ms");
        ret.push(1, "dump_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_dump_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_dump_period_ms) + " ms");
        ret.push(1, "countdown_timer");
        ret.push(2, "status: " + getTimerStatusStr(msg.server_countdown_timer_status));
        ret.push(2, "period: " + std::to_string(msg.server_countdown_period_ms) + " ms");
        return ret;
    }
};



}// namespace msg_show





/**
 * ================================================================
 * Namespace msg_to_json.
 * ================================================================
 */

namespace msg_to_json
{

class MappingData;
class MotorValueRange;
class Bbox2d;
class Point2d;
class Point2f;
class Size2f;

/** msg_content */
class MappingData
{
public:
    static nlohmann::json dump(const vehicle_interfaces::msg::MappingData& src)
    {
        nlohmann::json ret;
        ret["input_vec"] = src.input_vec;
        ret["output_vec"] = src.output_vec;
        return ret;
    }
};

class MotorValueRange
{
public:
    static nlohmann::json dump(const vehicle_interfaces::msg::MotorValueRange& src)
    {
        nlohmann::json ret;
        ret["min"] = src.min;
        ret["max"] = src.max;
        ret["init"] = src.init;
        return ret;
    }
};

/** msg_geo */
class Point2d
{
public:
    static nlohmann::json dump(const vehicle_interfaces::msg::Point2d& src)
    {
        nlohmann::json ret;
        ret["x"] = src.x;
        ret["y"] = src.y;
        return ret;
    }
};

class Point2f
{
public:
    static nlohmann::json dump(const vehicle_interfaces::msg::Point2f& src)
    {
        nlohmann::json ret;
        ret["x"] = src.x;
        ret["y"] = src.y;
        return ret;
    }
};

class Size2f
{
public:
    static nlohmann::json dump(const vehicle_interfaces::msg::Size2f& src)
    {
        nlohmann::json ret;
        ret["width"] = src.width;
        ret["height"] = src.height;
        return ret;
    }
};

class Bbox2d
{
public:
    static nlohmann::json dump(const vehicle_interfaces::msg::Bbox2d& src)
    {
        nlohmann::json ret;
        ret["o"] = Point2f::dump(src.o);
        ret["origin_type"] = src.origin_type;
        ret["size"] = Size2f::dump(src.size);
        ret["label"] = src.label;
        ret["id"] = src.id;
        return ret;
    }
};

}// namespace msg_to_json





/**
 * ================================================================
 * Namespace json_to_msg.
 * ================================================================
 */

namespace json_to_msg
{

class MappingData;
class MotorValueRange;
class Bbox2d;
class Point2d;
class Point2f;
class Size2f;

/** msg_content */
class MappingData
{
public:
    static vehicle_interfaces::msg::MappingData dump(const nlohmann::json& src)
    {
        vehicle_interfaces::msg::MappingData ret;

        for (const auto& i : src["input_vec"].items())
            ret.input_vec.push_back(i.value());

        for (const auto& i : src["output_vec"].items())
            ret.output_vec.push_back(i.value());

        return ret;
    }
};

class MotorValueRange
{
public:
    static vehicle_interfaces::msg::MotorValueRange dump(const nlohmann::json& src)
    {
        vehicle_interfaces::msg::MotorValueRange ret;
        ret.min = src["min"];
        ret.max = src["max"];
        ret.init = src["init"];
        return ret;
    }
};

/** msg_geo */
class Point2d
{
public:
    static vehicle_interfaces::msg::Point2d dump(const nlohmann::json& src)
    {
        vehicle_interfaces::msg::Point2d ret;
        ret.x = src["x"];
        ret.y = src["y"];
        return ret;
    }
};

class Point2f
{
public:
    static vehicle_interfaces::msg::Point2f dump(const nlohmann::json& src)
    {
        vehicle_interfaces::msg::Point2f ret;
        ret.x = src["x"];
        ret.y = src["y"];
        return ret;
    }
};

class Size2f
{
public:
    static vehicle_interfaces::msg::Size2f dump(const nlohmann::json& src)
    {
        vehicle_interfaces::msg::Size2f ret;
        ret.width = src["width"];
        ret.height = src["height"];
        return ret;
    }
};

class Bbox2d
{
public:
    static vehicle_interfaces::msg::Bbox2d dump(const nlohmann::json& src)
    {
        vehicle_interfaces::msg::Bbox2d ret;
        ret.o = Point2f::dump(src["o"]);
        ret.origin_type = src["origin_type"];
        ret.size = Size2f::dump(src["size"]);
        ret.label = src["label"];
        ret.id = src["id"];
        return ret;
    }
};

}// namespace json_to_msg





/**
 * ================================================================
 * Namespace msg_to_msg.
 * ================================================================
 */

namespace msg_to_msg
{

class ControlChassis
{
public:
    /**
     * Convert vehicle_interfaces::msg::Chassis to vehicle_interfaces::msg::ControlChassis.
     * @param src vehicle_interfaces::msg::Chassis
     * @return vehicle_interfaces::msg::ControlChassis
     */
    static vehicle_interfaces::msg::ControlChassis convert(const vehicle_interfaces::msg::Chassis& src)
    {
        vehicle_interfaces::msg::ControlChassis ret;
        ret.unit_type = src.unit_type;
        ret.drive_motor = src.drive_motor;
        ret.steering_motor = src.steering_motor;
        ret.brake_motor = src.brake_motor;
        ret.parking_signal = src.parking_signal;
        ret.controller_frame_id = src.controller_frame_id;
        ret.controller_interrupt = src.controller_interrupt;
        return ret;
    }
};

class Chassis
{
public:
    /**
     * Convert vehicle_interfaces::msg::ControlChassis to vehicle_interfaces::msg::Chassis.
     * @param src vehicle_interfaces::msg::ControlChassis
     * @return vehicle_interfaces::msg::Chassis
     * @note The returned message will not be completed because the original message has more fields than the target message.
     */
    static vehicle_interfaces::msg::Chassis convert(const vehicle_interfaces::msg::ControlChassis& src)
    {
        vehicle_interfaces::msg::Chassis ret;
        ret.unit_type = src.unit_type;
        ret.drive_motor = src.drive_motor;
        ret.steering_motor = src.steering_motor;
        ret.brake_motor = src.brake_motor;
        ret.parking_signal = src.parking_signal;
        ret.controller_frame_id = src.controller_frame_id;
        ret.controller_interrupt = src.controller_interrupt;
        return ret;
    }
};

class ControlSteeringWheel
{
public:
    /**
     * Convert vehicle_interfaces::msg::SteeringWheel to vehicle_interfaces::msg::ControlSteeringWheel.
     * @param src vehicle_interfaces::msg::SteeringWheel
     * @return vehicle_interfaces::msg::ControlSteeringWheel
     */
    static vehicle_interfaces::msg::ControlSteeringWheel convert(const vehicle_interfaces::msg::SteeringWheel& src)
    {
        vehicle_interfaces::msg::ControlSteeringWheel ret;
        ret.gear = src.gear;
        ret.steering = src.steering;
        ret.pedal_throttle = src.pedal_throttle;
        ret.pedal_brake = src.pedal_brake;
        ret.pedal_clutch = src.pedal_clutch;
        ret.func_0 = src.func_0;
        ret.func_1 = src.func_1;
        ret.func_2 = src.func_2;
        ret.func_3 = src.func_3;
        ret.controller_frame_id = src.controller_frame_id;
        ret.controller_interrupt = src.controller_interrupt;
        return ret;
    }
};

class SteeringWheel
{
public:
    /**
     * Convert vehicle_interfaces::msg::ControlSteeringWheel to vehicle_interfaces::msg::SteeringWheel.
     * @param src vehicle_interfaces::msg::ControlSteeringWheel
     * @return vehicle_interfaces::msg::SteeringWheel
     * @note The returned message will not be completed because the original message has more fields than the target message.
     */
    static vehicle_interfaces::msg::SteeringWheel convert(const vehicle_interfaces::msg::ControlSteeringWheel& src)
    {
        vehicle_interfaces::msg::SteeringWheel ret;
        ret.gear = src.gear;
        ret.steering = src.steering;
        ret.pedal_throttle = src.pedal_throttle;
        ret.pedal_brake = src.pedal_brake;
        ret.pedal_clutch = src.pedal_clutch;
        ret.func_0 = src.func_0;
        ret.func_1 = src.func_1;
        ret.func_2 = src.func_2;
        ret.func_3 = src.func_3;
        ret.controller_frame_id = src.controller_frame_id;
        ret.controller_interrupt = src.controller_interrupt;
        return ret;
    }
};

}// namespace msg_to_msg

}