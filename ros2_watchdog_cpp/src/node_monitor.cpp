// Copyright 2026 TUMFTM
#include "ros2_watchdog_cpp/node_monitor.hpp"

#include "tum_ros_helpers_cpp/qos.hpp"
namespace tam::core
{
NodeMonitor::NodeMonitor(
  rclcpp::Node * node, const std::string & custom_topic, const bool automatic_causing_key)
: node_(node), automatic_causing_key_(automatic_causing_key)
{
  if (!custom_topic.empty() && custom_topic.back() == '/') {
    throw std::invalid_argument("Custom topic should not end with a '/'");
  }
  std::string topic = (custom_topic.empty())
                        ? std::string("/core/orchestration/") + std::string(this->node_->get_name())
                        : custom_topic;
  this->diagnostic_pub_ = this->node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    topic + std::string("_status"), tam::ros::get_qos());

  // Initialize diagnostic status for node name
  this->stati_[this->node_->get_name()] = DiagnosticStatus{};
  this->set_message("Initializing");
  this->set_error_lvl("startup", tam::types::ErrorLvl::ERROR);

  // Initialize diagnostic msg with status for node name
  diagnostic_msgs::msg::DiagnosticStatus status{};
  status.name = this->node_->get_name();
  this->diagnostic_msg_.status.clear();
  this->diagnostic_msg_.status.push_back(status);
}
std::function<void()> NodeMonitor::get_update_function()
{
  return [this]() { this->update(); };
}
void NodeMonitor::update(const std_msgs::msg::Header & header)
{
  // Stamp with provided header time
  diagnostic_msg_.header = header;
  // Iterate over all stati and fill diagnostic message
  diagnostic_msg_.status.clear();
  for (auto & [name, status] : this->stati_) {
    diagnostic_msgs::msg::DiagnosticStatus status_msg{};
    status_msg.name = name;
    status_msg.message = status.message;
    status_msg.values = get_key_value_msg(name);
    status_msg.level = tam::type_conversions::diagnostic_level_from_type(get_max_error_lvl(name));
    // Add the status (node name always first)
    if (name == this->node_->get_name())
      diagnostic_msg_.status.insert(diagnostic_msg_.status.begin(), status_msg);
    else
      diagnostic_msg_.status.push_back(status_msg);
  }
  // Publish diagnostic message
  this->diagnostic_pub_->publish(diagnostic_msg_);
}
void NodeMonitor::set_error_lvl(
  const std::string & status_name, const std::string & key, const tam::types::ErrorLvl & lvl)
{
  this->stati_[status_name].error_items.insert_or_assign(key, lvl);
}
void NodeMonitor::set_message(const std::string & status_name, const std::string & message)
{
  this->stati_[status_name].message = message;
}
void NodeMonitor::initialization_finished()
{
  set_error_lvl("startup", tam::types::ErrorLvl::OK);
  set_message("Initialized");
}
void NodeMonitor::set_status_code(const std::string & status_name, const int code)
{
  report_value(status_name, "status_code", std::to_string(code));
}
int NodeMonitor::get_status_code(const std::string & status_name)
{
  if (stati_.find(status_name) == stati_.end()) {
    throw std::out_of_range("[NodeMonitor]: status_name '" + status_name + "' not found in stati_");
  }
  if (
    stati_.at(status_name).key_value_map.find("status_code") ==
    stati_.at(status_name).key_value_map.end()) {
    throw std::out_of_range(
      "[NodeMonitor]: 'status_code' not found for status_name '" + status_name + "'");
  }
  return std::stoi(stati_.at(status_name).key_value_map.at("status_code"));
}
void NodeMonitor::report_value(
  const std::string & status_name, const std::string & name, const std::string & value)
{
  this->stati_[status_name].key_value_map.insert_or_assign(name, value);
}
tam::types::ErrorLvl NodeMonitor::get_max_error_lvl(const std::string & status_name)
{
  if (stati_.find(status_name) == stati_.end()) {
    throw std::out_of_range("[NodeMonitor]: status_name '" + status_name + "' not found in stati_");
  }
  auto & error_items = this->stati_.at(status_name).error_items;
  if (error_items.empty()) {
    return tam::types::ErrorLvl::OK;
  }
  auto min = std::max_element(
    error_items.begin(), error_items.end(),
    [](const auto & l, const auto & r) { return l.second < r.second; });
  if (automatic_causing_key_) {
    report_value(status_name, "causing_key", min->first);
  }
  return min->second;
}
std::vector<diagnostic_msgs::msg::KeyValue> NodeMonitor::get_key_value_msg(
  const std::string & status_name)
{
  if (stati_.find(status_name) == stati_.end()) {
    throw std::out_of_range("[NodeMonitor]: status_name '" + status_name + "' not found in stati_");
  }
  const auto & key_value_map = this->stati_.at(status_name).key_value_map;
  std::vector<diagnostic_msgs::msg::KeyValue> msg;
  msg.reserve(key_value_map.size());
  for (const auto & [key, value] : key_value_map) {
    msg.emplace_back();  // Construct in place
    msg.back().key = key;
    msg.back().value = value;
  }
  return msg;
}
}  // namespace tam::core
