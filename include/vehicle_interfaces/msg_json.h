#pragma once
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
/** msg */
#include "vehicle_interfaces/msg/chassis.hpp"
#include "vehicle_interfaces/msg/steering_wheel.hpp"
/** msg_content */
#include "vehicle_interfaces/msg/control_chassis.hpp"
#include "vehicle_interfaces/msg/control_steering_wheel.hpp"
#include "vehicle_interfaces/msg/mapping_data.hpp"
#include "vehicle_interfaces/msg/motor_value_range.hpp"
/** msg_geo */
#include "vehicle_interfaces/msg/bbox2d.hpp"
#include "vehicle_interfaces/msg/point2d.hpp"
#include "vehicle_interfaces/msg/point2f.hpp"
#include "vehicle_interfaces/msg/size2f.hpp"

namespace vehicle_interfaces
{

namespace msg_to_json
{

class MappingData;
class MotorValueRange;
class Bbox2d;
class Point2d;
class Point2f;
class Size2f;

/** msg_content*/
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

/** msg_geo*/
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

namespace json_to_msg
{

class MappingData;
class MotorValueRange;
class Bbox2d;
class Point2d;
class Point2f;
class Size2f;

/** msg_content*/
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

/** msg_geo*/
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
        return ret;
    }
};

}// namespace msg_to_msg

}