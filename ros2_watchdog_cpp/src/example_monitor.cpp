// Copyright 2026 TUMFTM
#include <rclcpp/rclcpp.hpp>

#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/timeout_value_provider.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/int16.hpp"
#include "tum_helpers_cpp/containers.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
class TestNode : public rclcpp::Node
{
private:
  tam::core::NodeMonitor::UniquePtr monitor_;
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr obj_subs_;

  // Publishers to test sync
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr sync_topic_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sync_diag_pub_;
  rclcpp::TimerBase::SharedPtr sync_pub_timer_;

public:
  TestNode() : Node("Test")
  {
    tam::core::TimeoutValueProvider timeout_values;
    monitor_ = std::make_unique<tam::core::NodeMonitor>(this);
    topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);
    pub_timer_ =
      this->create_wall_timer(500ms, std::bind(&TestNode::function_queue_callback, this));

    // Setup subscription with timeout callback
    topic_watchdog_->add_subscription<std_msgs::msg::Int16>(
      "test", tam::ros::get_qos(), std::bind(&TestNode::topic_callback, this, _1),
      std::bind(&TestNode::timeout_callback, this, _1, _2),
      timeout_values.value_ms("TrackingController"));

    // Alternatively -> Manually timeout subscription
    // NOTE: Do not use auto as return type here. If so the compiler
    // won't be able to deduce the type when trying to register the timeoutet
    // callback in the create_subscription method of the rclcpp node
    std::function<void(std_msgs::msg::Int16::SharedPtr)> timeoutet_callback =
      topic_watchdog_->timeout_callback(
        [this](auto msg) { topic_callback(msg); },
        [this](auto a, auto b) { this->timeout_callback(a, b); },
        timeout_values.value_ms("TrackingController"));

    this->create_subscription<std_msgs::msg::Int16>(
      "/test_sub", tam::ros::get_qos(), timeoutet_callback);

    // Synchronized subscriptions with diagnostics
    auto sub_topic = topic_watchdog_->add_synced_subscription<sensor_msgs::msg::Temperature>(
      "/test_sync", tam::ros::get_qos());
    auto sub_diag = topic_watchdog_->add_synced_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/test_sync_diag", tam::ros::get_qos());

    topic_watchdog_->register_synced_callback(
      sub_topic, sub_diag, std::bind(&TestNode::topic_callback_syn, this, _1, _2),
      std::bind(&TestNode::timeout_callback, this, _1, _2),
      timeout_values.value_ms("TrackingController"));

    // Create publisher to test sync
    sync_topic_pub_ =
      this->create_publisher<sensor_msgs::msg::Temperature>("/test_sync", tam::ros::get_qos());
    sync_diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/test_sync_diag", tam::ros::get_qos());

    sync_pub_timer_ =
      this->create_wall_timer(200ms, std::bind(&TestNode::publish_sync_topics, this));

    // Set the initialization finished separately
    monitor_->initialization_finished();
    monitor_->set_status_code(30);  // Can be module specific (only for debugging purpose!)
  }
  void function_queue_callback()
  {
    topic_watchdog_->get_update_function();
    update_callback();
    monitor_->get_update_function();
  }
  void update_callback() { monitor_->set_message("Running"); }
  void topic_callback(std_msgs::msg::Int16::SharedPtr msg)
  {
    monitor_->report_value("std_dev", static_cast<int>(msg->data));
    if (msg->data > 2.0) {
      monitor_->set_error_lvl("std_dev_1", tam::types::ErrorLvl::ERROR);
    } else {
      monitor_->set_error_lvl("std_dev_1", tam::types::ErrorLvl::OK);
    }
    monitor_->report_value("OtherValue", "Value", std::to_string(msg->data * 2));
    monitor_->set_message("OtherValue", "OtherMessage");
    monitor_->get_max_error_lvl("OtherValue");
  }
  void topic_callback_syn(
    const sensor_msgs::msg::Temperature::ConstSharedPtr msg_ptr,
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr)
  {
    RCLCPP_INFO(
      this->get_logger(), "Synchronized callback received - Header time: %u.%u",
      msg_ptr->header.stamp.sec, msg_ptr->header.stamp.nanosec);
  }
  void timeout_callback(bool timeout, std::chrono::milliseconds timeout_now)
  {
    monitor_->set_error_lvl("topic_timeout", tam::types::ErrorLvl::OK);
    if (timeout_now > 300ms) {
      monitor_->set_error_lvl("topic_timeout", tam::types::ErrorLvl::WARN);
    }
    if (timeout) {
      monitor_->set_error_lvl("topic_timeout", tam::types::ErrorLvl::ERROR);
    }
  }
  void publish_sync_topics()
  {
    auto header = std_msgs::msg::Header();
    header.stamp = this->get_clock()->now();
    header.frame_id = "sync_frame";
    auto msg = std::make_shared<sensor_msgs::msg::Temperature>();
    msg->header = header;
    msg->temperature = 25.0;
    sync_topic_pub_->publish(*msg);

    auto diag_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    diag_msg->header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "TestStatus";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "All good";
    diag_msg->status.push_back(status);
    sync_diag_pub_->publish(*diag_msg);

    monitor_->update(header);
  }
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
