/*
QoS definition in rclcpp/qos.hpp
QoS private member: rmw_qos_profile_t (rmw/types.h)

typedef struct RMW_PUBLIC_TYPE rmw_qos_profile_t
{
    enum rmw_qos_history_policy_t history;
    size_t depth;
    enum rmw_qos_reliability_policy_t reliability;
    enum rmw_qos_durability_policy_t durability;
    struct rmw_time_t deadline;
    struct rmw_time_t lifespan;
    enum rmw_qos_liveliness_policy_t liveliness;
    struct rmw_time_t liveliness_lease_duration;


    bool avoid_ros_namespace_conventions;
} rmw_qos_profile_t;

enum RMW_PUBLIC_TYPE rmw_qos_history_policy_t
{
    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    RMW_QOS_POLICY_HISTORY_UNKNOWN
};

enum RMW_PUBLIC_TYPE rmw_qos_reliability_policy_t
{
    RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_RELIABILITY_UNKNOWN
};

enum RMW_PUBLIC_TYPE rmw_qos_durability_policy_t
{
    RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_POLICY_DURABILITY_UNKNOWN
};

enum RMW_PUBLIC_TYPE rmw_qos_liveliness_policy_t
{
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT = 0,
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC = 1,
    RMW_DECLARE_DEPRECATED(
        RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,
        RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE_DEPRECATED_MSG) = 2,
    // Using `3` for backwards compatibility.
    RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3,
    RMW_QOS_POLICY_LIVELINESS_UNKNOWN = 4
};
*/