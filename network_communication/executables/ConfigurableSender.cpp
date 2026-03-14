// Copyright 2025 Simon Sagmeister
#include <network_communication/tam_sender_serialized.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "network_communication/helpers.h"
#include "network_communication/udp_sender.h"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto sender = std::make_shared<tam::network::SerializedSender>("/connection_status");
  std::cout << "Creating Configurable Sender. Node Name: " << sender->get_fully_qualified_name()
            << "| Listening to Connection Topic: " << sender->get_connection_status_topic()
            << std::endl;

  // region read the config from the pkg share directory
  auto config_directory =
    ament_index_cpp::get_package_share_directory("network_communication").append("/config");
  auto config_file_path = config_directory + "/SenderReceiverConfig.csv";
  auto configs = tam::network::helpers::parseSenderReceiverConfig(config_file_path);

  for (auto config : configs) {
    auto topic_name = sender->add_processer(
      config.topic_name, config.msg_type, std::make_unique<tam::network::UdpSender>(config.port));
    std::cout << "     Topic: " << topic_name << " | Type: " << config.msg_type
              << " | Port: " << config.port << std::endl;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sender);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}