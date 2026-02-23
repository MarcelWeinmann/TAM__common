# ros2_watchdog_cpp

This library exists to give a common solution for making sure that new data is arriving on time. Instead of tracking this in a module itself, the module can use this library to make tracking timeouts a simple process.

Full example using everything there is in this package is located [in this file](./src/example_monitor.cpp).

## 👀 Topic Watchdog 👀

### Single Subscriptions

Instead of creating a simple subscription:

```cpp
this->create_subscription<std_msgs::msg::Int16>(
    "test", // Topic
    10, // QoS
    std::bind(&TestNode::topic_callback, this, _1) // Message callback
);
```

Instead do this to automatically check for timeouts on important topics:

```cpp
// Add this to your node
topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);

topic_watchdog_->add_subscription<std_msgs::msg::Int16>(
    "test", // Topic
    10, // QoS
    std::bind(&TestNode::topic_callback, this, _1), // Message callback
    std::bind(&TestNode::timeout_callback, this, _1, _2), // Timeout callback
    200ms // Timeout is triggered after 200ms
);

...

// Required function signature
void timeout_callback(bool timeout, std::chrono::milliseconds timeout_now)
{
    // Do something in case of timeout
    // e.g. report timeout in node monitor
}

...

// You will still need to trigger the check for timeouts manually
void timer_callback()
{
    topic_watchdog_->check_timeouts();
}
```

### Subscriptions with Diagnostics

If you need to subscribe to a critial message and its corresponding diagnostics, you should
use the synced subscription. It sets up a synced subscription for an arbitrary msg-type `<MSG>`
with a corresponding `diagnostic_msgs::msg::DiagnosticArray>` using the [message_filters](https://github.com/ros2/message_filters)
package.
Simply add the subscriptions to the two topics to the watchdog using the corresponding function:

```cpp
// Synchronized subscriptions with diagnostics
auto sub_topic = topic_watchdog_->add_synced_subscription<MSG>(
    "/test_sync", tam::ros::get_qos());
auto sub_diag = topic_watchdog_->add_synced_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/test_sync_diag", tam::ros::get_qos());
```

Further, register the callback-function to synchronize them. It needs to have the following function signature
(`ConstSharedPtr` required):

```cpp
  void topic_callback_sync(const <MSG> msg_ptr, const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg_ptr);
```

Register it together with the appropriate timeout-callback (function signature see above):

```cpp
topic_watchdog_->register_synced_callback(
    sub_topic, sub_diag, std::bind(&TestNode::topic_callback_sync, this, _1, _2),
    std::bind(&TestNode::timeout_callback, this, _1, _2),
    500ms);
```

## 📺 Node Monitor 📺

The Node Monitor provides standardized health monitoring and diagnostics for your ROS2 node. It publishes `diagnostic_msgs::msg::DiagnosticArray` messages to `/core/orchestration/<node_name>_status`, making node health visible to dashboards and monitoring tools.

### Basic Setup

```cpp
class MyNode : public rclcpp::Node
{
private:
  tam::core::NodeMonitor::UniquePtr monitor_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  MyNode() : Node("MyNode")
  {
    // Initialize monitor
    monitor_ = std::make_unique<tam::core::NodeMonitor>(this);

    // or if you want to have a custom topic name (appended with "_status")
    monitor_ = std::make_unique<tam::core::Nodemonitor>(this, "/vehicle/sensor/lidar")
    
    // Your initialization code here...
    
    // Signal that initialization is complete
    monitor_->initialization_finished();
    
    // Create timer to periodically publish diagnostics
    timer_ = this->create_wall_timer(
      500ms, [this]() { monitor_->update(); }
    );
  }
};
```

### Synced Diagnostics

In case you want to use the Node Monitor to indicate the health status of a specific topic, which may be subscribed with a syned subscription,
you need to specify the header of `DiagnosticArray`-message when updating the monitor:

```cpp
auto header = std_msgs::msg::Header();
header.stamp = this->get_clock()->now();
header.frame_id = "sync_topics";
auto msg = std::make_shared<MSG>();
msg->header = header;
// Fill message
topic_pub_->publish(*msg);
// Now, publish the diagnostic status with the same header via the monitor:
monitor_->update(header);
```

### Setting Error Levels

Track different error conditions with unique keys. The published diagnostic level will be the worst across all keys:

```cpp
// Either...
monitor_->set_error_lvl("temperature", tam::types::ErrorLvl::ERROR);
// or...
monitor_->set_error_lvl("temperature", tam::types::ErrorLvl::WARN);
// or ...
monitor_->set_error_lvl("temperature", tam::types::ErrorLvl::OK);
```

### Reporting Values

Add key-value pairs to diagnostics for debugging and visualization:

```cpp
// Either ...
monitor_->report_value("std_dev", std_dev);          // double
// or ...
monitor_->report_value("frame_count", frame_count);  // int
// or ...
monitor_->report_value("mode", "tracking");          // string
```

### Status Codes

Set module-specific status codes for dashboard integration:

```cpp
monitor_->set_status_code(30);
```

### Status Messages

Update the human-readable status message:

```cpp
monitor_->set_message("Running");
```

### Multiple Stati (Advanced)

By default, all setter-methods will apply the `DiagnosticStatus` named after the name of the parent-node.
This is also the status that is used for dashboard-integration. However, for all setter- and getter-methods,
you can specify a different `status_name` as a first argument for easier integration of higher-dimensional diagnostics:

```cpp
// Default uses node name
monitor_->set_error_lvl("sensor_error", tam::types::ErrorLvl::ERROR);

// Track separate status for a subsystem
monitor_->set_error_lvl("Subsystem1", "comm_error", tam::types::ErrorLvl::WARN);
monitor_->set_message("Subsystem1", "Communication degraded");
monitor_->report_value("Subsystem1", "latency_ms", 150);
```

### Exception Handling

**Important:** All getter methods throw `std::runtime_error` if the requested status or key doesn't exist:

```cpp
try {
  auto lvl = monitor_->get_max_error_lvl("NonExistent");  // Throws if status doesn't exist
} catch (const std::runtime_error & e) {
  RCLCPP_ERROR(this->get_logger(), "Monitor error: %s", e.what());
}
```
