# QoS Setting Workflow

## QoS Profile Description
- QoS definition and implementation in rclcpp/qos.hpp and rclcpp/qos.cpp
- QoS profile: rmw_qos_profile_t (rmw/types.h)
    ```cpp
    typedef struct RMW_PUBLIC_TYPE rmw_qos_profile_s
    {
        enum rmw_qos_history_policy_e history;
        size_t depth;
        enum rmw_qos_reliability_policy_e reliability;
        enum rmw_qos_durability_policy_e durability;
        struct rmw_time_s deadline;
        struct rmw_time_s lifespan;
        enum rmw_qos_liveliness_policy_e liveliness;
        struct rmw_time_s liveliness_lease_duration;

        bool avoid_ros_namespace_conventions;
    } rmw_qos_profile_t;

    enum RMW_PUBLIC_TYPE rmw_qos_history_policy_e
    {
        RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        RMW_QOS_POLICY_HISTORY_KEEP_ALL,
        RMW_QOS_POLICY_HISTORY_UNKNOWN
    } rmw_qos_history_policy_t;

    enum RMW_PUBLIC_TYPE rmw_qos_reliability_policy_e
    {
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_RELIABILITY_UNKNOWN
    } rmw_qos_reliability_policy_t;

    enum RMW_PUBLIC_TYPE rmw_qos_durability_policy_e
    {
        RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_POLICY_DURABILITY_UNKNOWN
    } rmw_qos_durability_policy_t;

    enum RMW_PUBLIC_TYPE rmw_qos_liveliness_policy_e
    {
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT = 0,
        RMW_QOS_POLICY_LIVELINESS_AUTOMATIC = 1,
        RMW_DECLARE_DEPRECATED(
            RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,
            RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE_DEPRECATED_MSG) = 2,
        // Using `3` for backwards compatibility.
        RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC = 3,
        RMW_QOS_POLICY_LIVELINESS_UNKNOWN = 4
    } rmw_qos_liveliness_policy_t;
    ```

## Set QoS Profile Using Parameter Service

### Study
- Show the name and value of parameters under cli
    - List all parameters for all nodes
        ```bash
        ros2 param list
        ```
    - List parameters for specific node
        ```bash
        ros2 param list <node_name>
        ```
    - Show specific parameter value
        ```bash
        ros2 param get <node_name> <param_name>
        ```
- Dump parameters for specific node under cli
    ```bash
    ros2 param dump <node_name>
    ```

### QoS overriding
qos_overrides parameters cannot be configured after startup.
    Ref: [6 ros2 param load](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html): **Read-only parameters can only be modified at startup and not afterwards, that is why there are some warnings for the “qos_overrides” parameters.**
- Add overriding options under publisher
    ```cpp
    /* ref: https://github.com/ros2/demos/blob/rolling/quality_of_service_demo/rclcpp/src/qos_overrides_talker.cpp */
    rclcpp::PublisherOptions pub_opts;
    // Update the subscription options to allow reconfigurable qos settings.
    pub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
    {
        // Here all policies that are desired to be reconfigurable are listed.
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability
    },
    [](const rclcpp::QoS & qos) {
        /** This is a qos validation callback, that can optionally be provided.
         * Here, arbitrary constraints in the final qos profile can be checked.
            * The function will return true if the user provided qos profile is accepted.
            * If the profile is not accepted, the user will get an InvalidQosOverridesException.
            */
        rclcpp::QosCallbackResult result;
        result.successful = false;
        if (qos.depth() > 10u) {
        result.reason = "expected history depth less or equal than 10";
        return result;
        }
        result.successful = true;
        return result;
    }
    /**
     * The "id" option is useful when you want to have subscriptions
        * listening to the same topic with different QoS profiles within a node.
        * You can try uncommenting and modifying
        * `qos_overrides./qos_overrides_topic.subscription`
        * to
        * `qos_overrides./qos_overrides_topic.subscription_custom_identifier`
        *
        * Uncomment the next line to try it.
        */
    // , "custom_identifier"
    };

    // Create publisher
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("qos_overrides_chatter", 1, pub_opts);
    ```
- Add overriding options under subscription
    ```cpp
    /* ref: https://github.com/ros2/demos/blob/rolling/quality_of_service_demo/rclcpp/src/qos_overrides_listener.cpp */
    rclcpp::SubscriptionOptions sub_opts;
    // Update the subscription options to allow reconfigurable qos settings.
    sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
      {
        // Here all policies that are desired to be reconfigurable are listed.
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability,
      },
      [](const rclcpp::QoS & qos) {
        /** This is a qos validation callback, that can optionally be provided.
         * Here, arbitrary constraints in the final qos profile can be checked.
         * The function will return true if the user provided qos profile is accepted.
         * If the profile is not accepted, the user will get an InvalidQosOverridesException.
         */
        rclcpp::QosCallbackResult result;
        result.successful = false;
        if (qos.depth() > 10u) {
          result.reason = "expected history depth less or equal than 10";
          return result;
        }
        result.successful = true;
        return result;
      }
      /**
       * The "id" option is useful when you want to have subscriptions
       * listening to the same topic with different QoS profiles within a node.
       * You can try uncommenting and modifying
       * `qos_overrides./qos_overrides_topic.subscription`
       * to
       * `qos_overrides./qos_overrides_topic.subscription_custom_identifier`
       *
       * Uncomment the next line to try it.
       */
      // , "custom_identifier"
    };

    // Create subscription
    sub_ = create_subscription<sensor_msgs::msg::Image>("qos_overrides_chatter", 1, callback, sub_opts);
    ```