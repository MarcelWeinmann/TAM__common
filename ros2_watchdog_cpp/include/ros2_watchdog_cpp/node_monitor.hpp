// Copyright 2026 TUMFTM
#pragma once
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/common.hpp"

using namespace std::chrono_literals;
namespace tam::core
{
/// @brief  If instantiated with a Node Handle, this publishes diagnostic_msgs/Status for the
/// current node. Use public functions to set tam::types::ErrorLvl, Message or values
class NodeMonitor
{
public:
  // clang-format off
  /// @brief Create a Monitor for your Node
  /// @param node Instance to your parent Node
  /// @param automatic_causing_key will set the key-value pair "causing_key" automatically based on
  /// the key associated with the error (default=true). Set to false if you want to specify the
  /// "causing_key" manually
  explicit NodeMonitor(rclcpp::Node * node, const std::string & custom_topic = "", const bool automatic_causing_key = true);  // NOLINT
  /// @brief returns a std::function of the NodeMonitors update function
  std::function<void()> get_update_function();
  /// @brief Trigger publisher of the diagnostic_msgs
  /// @param header Header to stamp the diagnostic message with (default is now)
  void update(const std_msgs::msg::Header & header);
  void update() {
    std_msgs::msg::Header header;
    header.stamp = this->node_->get_clock()->now();
    update(header);
  }
  /// @brief Set the Error Level for a specific key. The published Error Level is the worst Error
  /// Level across all keys
  /// @param status_name Name of the status (overload takes node name by default)
  /// @param key Key to the according error level.
  /// @param lvl Error Level (OK, WARN, ERROR)
  void set_error_lvl(const std::string & status_name, const std::string & key, const tam::types::ErrorLvl & lvl);  // NOLINT
  void set_error_lvl(const std::string & key, const tam::types::ErrorLvl & lvl) { set_error_lvl(this->node_->get_name(), key, lvl); }  // NOLINT
  /// @brief Update the Message that is published within the diagnostic_msgs/Status
  /// @param status_name Name of the status (overload takes node name by default)
  /// @param message Message
  void set_message(const std::string & status_name, const std::string & message);
  void set_message(const std::string & message) { set_message(this->node_->get_name(), message); }
  /// @brief Will set the status message from "Initializing"- ERROR, to "Initialized" - OK
  void initialization_finished();
  /// @brief Adds a key-value pair to the diagnostic_msgs/Status.
  /// @param status_name Name of the status (overload takes node name by default)
  /// @param name name of the value
  /// @param value value
  void report_value(const std::string & status_name, const std::string & name, const std::string & value);  // NOLINT
  void report_value(const std::string & status_name, const std::string & name, const int value) { report_value(status_name, name,  std::to_string(value)); }  // NOLINT
  void report_value(const std::string & status_name, const std::string & name, const double value) { report_value(status_name, name, std::to_string(value)); }  // NOLINT
  void report_value(const std::string & name, const std::string & value) { report_value(this->node_->get_name(), name, value); }  // NOLINT
  void report_value(const std::string & name, const int value) { report_value(name, std::to_string(value)); }  // NOLINT
  void report_value(const std::string & name, const double value) { report_value(name, std::to_string(value)); }  // NOLINT
  /// @brief Adds a user defined status code diagnostic_msgs/Status in the key-value field
  /// "status_code"
  /// @param code User defined status code. Only for debugging purpose.
  /// @param status_name Name of the status (overload takes node name by default)
  void set_status_code(const std::string & status_name, const int code);
  void set_status_code(const int code) { set_status_code(this->node_->get_name(), code); }
  /// @brief Get the current status code from the key-value field "status_code"
  /// @param status_name Name of the status (overload takes node name by default)
  /// @returns status code as int
  int get_status_code(const std::string & status_name);
  int get_status_code() { return get_status_code(this->node_->get_name()); }
  /// @brief Get the maximum Error Level across all keys
  /// @param status_name Name of the status (overload takes node name by default)
  /// @returns Maximum Error Level
  tam::types::ErrorLvl get_max_error_lvl(const std::string & status_name);
  tam::types::ErrorLvl get_max_error_lvl() { return get_max_error_lvl(this->node_->get_name()); }
private:
  /// @brief Converts the internal key-value map to a vector of KeyValue messages
  /// @param status_name Name of the status (overload takes node name by default)
  /// @returns vector of KeyValue messages
  std::vector<diagnostic_msgs::msg::KeyValue> get_key_value_msg(const std::string & status_name);
  std::vector<diagnostic_msgs::msg::KeyValue> get_key_value_msg() { return get_key_value_msg(this->node_->get_name()); }  // NOLINT
  // clang-format on
private:
  /// @brief Internal struct to hold diagnostic status information
  struct DiagnosticStatus
  {
    std::unordered_map<std::string, tam::types::ErrorLvl> error_items{};
    std::unordered_map<std::string, std::string> key_value_map{};
    std::string message{};
  };
  /// @brief Pointer to the parent node
  rclcpp::Node * node_;

  /// @brief Publisher for diagnostic messages
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_{};
  /// @brief Diagnostic message that is published
  diagnostic_msgs::msg::DiagnosticArray diagnostic_msg_{};

  /// @brief Map of single stati
  std::unordered_map<std::string, DiagnosticStatus> stati_{};
  bool automatic_causing_key_{true};

public:
  using SharedPtr = std::shared_ptr<tam::core::NodeMonitor>;
  using UniquePtr = std::unique_ptr<tam::core::NodeMonitor>;
};
}  // namespace tam::core
