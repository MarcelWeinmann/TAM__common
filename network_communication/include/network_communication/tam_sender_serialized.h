// Copyright 2024 Simon Hoffmann
#pragma once
#include <network_communication/helpers.h>
#include <network_communication/udp_sender.h>

#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <tum_ros_helpers_cpp/qos.hpp>
#include <utility>
#include <vector>

#include "tum_msgs/msg/tum_connection_status.hpp"
namespace tam::network
{
using std::placeholders::_1;
class SerializedSender : public rclcpp::Node
{
private:
  struct RosMsgSender
  {
    rclcpp::SubscriptionBase::SharedPtr sendMsgSubs;
    std::unique_ptr<tam::network::BaseSender> Sender{nullptr};
    float min_pub_age_ms{0.0};
  };

public:
  explicit SerializedSender(const std::string & status_topic) : Node("TAM_Serialized_Sender")
  {
    _statusSubs = this->create_subscription<tum_msgs::msg::TUMConnectionStatus>(
      status_topic, tam::ros::get_qos(),
      std::bind(&SerializedSender::statusMessageReceived, this, _1));
  }
  SerializedSender(
    const std::string & status_topic, const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    _statusSubs = this->create_subscription<tum_msgs::msg::TUMConnectionStatus>(
      status_topic, tam::ros::get_qos(),
      std::bind(&SerializedSender::statusMessageReceived, this, _1));
  }
  SerializedSender(const std::string node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
    use_status_topic = false;
    this->declare_parameter("ip_address", "127.0.0.1");
    _connectionStat = tum_msgs::msg::TUMConnectionStatus::CONNECTED;
  }
  SerializedSender() : SerializedSender("TAM_Serialized_Sender", rclcpp::NodeOptions()){};
  std::string add_processer(
    const std::string & topic, const std::string & msg_type,
    std::unique_ptr<BaseSender> && protocol_strategy, const float min_send_age_ms = 0.0)
  {
    std::shared_ptr<RosMsgSender> sender =
      _rosMsgSenders.emplace_back(std::make_shared<RosMsgSender>());
    sender->Sender = std::move(protocol_strategy);

    std::function<void(const std::shared_ptr<rclcpp::SerializedMessage>)> fnc =
      std::bind(&SerializedSender::sendMessageReceived, this, _1, sender);
    sender->sendMsgSubs =
      this->create_generic_subscription(topic, msg_type, tam::ros::get_qos(), fnc);
    sender->min_pub_age_ms = min_send_age_ms;
    if (!use_status_topic) {
      sender->Sender->change_destination(this->get_parameter("ip_address").as_string());
    }
    return sender->sendMsgSubs->get_topic_name();
  }
  std::string get_connection_status_topic() { return _statusSubs->get_topic_name(); }

private:
  bool use_status_topic{true};
  std::vector<std::shared_ptr<RosMsgSender>> _rosMsgSenders;
  uint8_t _connectionStat{tum_msgs::msg::TUMConnectionStatus::DISCONNECTED};
  rclcpp::Subscription<tum_msgs::msg::TUMConnectionStatus>::SharedPtr _statusSubs;
  bool connected() { return _connectionStat == tum_msgs::msg::TUMConnectionStatus::CONNECTED; }
  void sendMessageReceived(
    const std::shared_ptr<rclcpp::SerializedMessage> event,
    std::shared_ptr<RosMsgSender> rosMsgSender)
  {
    static rclcpp::Time last_update{this->get_clock()->now()};
    rclcpp::Time now = this->get_clock()->now();
    float age_ms = (now - last_update).seconds() * 1e3;
    if (connected() && age_ms >= rosMsgSender->min_pub_age_ms) {
      uint8_t * data = event->get_rcl_serialized_message().buffer;
      std::vector<uint8_t> vec(data, data + event->get_rcl_serialized_message().buffer_length);
      int nofBytesSent = rosMsgSender->Sender->send_data(vec);
      last_update = now;
    }
  }
  void statusMessageReceived(const tum_msgs::msg::TUMConnectionStatus & msg)
  {
    _connectionStat = msg.connection_status;
    std::vector<std::string> receiverIps{};
    if (connected()) {
      receiverIps = msg.ip_addresses;
      auto receiverList = tam::network::helpers::get_string_from_ip_list(receiverIps);

      for (auto rosMsgSender : _rosMsgSenders) {
        if (receiverList != rosMsgSender->Sender->get_destination_ip()) {
          RCLCPP_INFO(
            this->get_logger(), "%s: sending to ip address %s", this->get_name(),
            receiverList.c_str());
          rosMsgSender->Sender->change_destination(receiverIps);
        }
      }
    }
  }
};
};  // namespace tam::network
