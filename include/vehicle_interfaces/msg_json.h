#pragma once
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
/** msg_content*/
#include "vehicle_interfaces/msg/mapping_data.hpp"
#include "vehicle_interfaces/msg/motor_value_range.hpp"
/** msg_geo*/
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

}

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

}

}