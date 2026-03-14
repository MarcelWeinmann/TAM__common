// Copyright 2024 Simon Hoffmann
#pragma once
#include <network_communication/helpers.h>
#include <network_communication/udp_receiver.h>

#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tum_msgs/msg/tum_connection_status.hpp>
#include <tum_ros_helpers_cpp/qos.hpp>
#include <utility>
#include <vector>
namespace tam::network
{
class SerializedReceiver : public rclcpp::Node
{
private:
  struct RosMsgReceiver
  {
    rclcpp::GenericPublisher::SharedPtr recvMsgPubs;
    std::unique_ptr<tam::network::BaseReceiver> Receiver{nullptr};
    bool printedInfo{false};
    std::unique_ptr<std::thread> thread{nullptr};
  };

public:
  explicit SerializedReceiver(const std::string & node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options)
  {
  }
  explicit SerializedReceiver() : Node("TAM_Serialized_Receiver", rclcpp::NodeOptions()) {}
  ~SerializedReceiver()
  {
    for (auto & receiver : _rosMsgReceivers) {
      if (receiver->thread->joinable()) receiver->thread->join();
    }
  }
  // Returns the actual topic name (relevant e.g. in case of remaps)
  std::string add_processer(
    const std::string & topic, const std::string & msg_type,
    std::unique_ptr<BaseReceiver> && protocol_strategy)
  {
    auto receiver = _rosMsgReceivers.emplace_back(std::make_shared<RosMsgReceiver>());
    receiver->Receiver = std::move(protocol_strategy);
    receiver->recvMsgPubs = this->create_generic_publisher(topic, msg_type, tam::ros::get_qos());
    receiver->thread =
      std::make_unique<std::thread>(&SerializedReceiver::receive_msgs, this, receiver);

    return receiver->recvMsgPubs->get_topic_name();
  }

private:
  std::vector<std::shared_ptr<RosMsgReceiver>> _rosMsgReceivers;
  void receive_msgs(std::shared_ptr<RosMsgReceiver> receiver)
  {
    receiver->Receiver->waiting_for_client_connect();
    while (rclcpp::ok()) {
      std::vector<uint8_t> data = receiver->Receiver->receive();

      rclcpp::SerializedMessage msg{};
      msg.get_rcl_serialized_message().buffer = data.data();
      msg.get_rcl_serialized_message().buffer_length = data.size();
      msg.get_rcl_serialized_message().buffer_capacity = data.capacity();

      receiver->recvMsgPubs->publish(msg);
      msg.release_rcl_serialized_message();
      // Release the serialized message to avoid freeing it twice.
      // The memory is still managed by the vector.
      if (!receiver->printedInfo) {
        receiver->printedInfo = true;
        RCLCPP_INFO(
          this->get_logger(), "%s: receiving topic %s", this->get_name(),
          receiver->recvMsgPubs->get_topic_name());
      }
    }
    receiver->Receiver->disconnect();
  }
};
}  // namespace tam::network
