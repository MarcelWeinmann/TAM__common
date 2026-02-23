// Copyright 2025 Simon Sagmeister
#include <network_communication/tam_receiver_serialized.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <vector>

#include "network_communication/helpers.h"
#include "network_communication/udp_receiver.h"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto receiver = std::make_shared<tam::network::SerializedReceiver>();
  std::cout << "Creating Configurable Receiver. Node Name: " << receiver->get_fully_qualified_name()
            << std::endl;

  // region read the config from the pkg share directory
  auto config_directory =
    ament_index_cpp::get_package_share_directory("network_communication").append("/config");
  auto config_file_path = config_directory + "/SenderReceiverConfig.csv";
  auto configs = tam::network::helpers::parseSenderReceiverConfig(config_file_path);

  for (auto config : configs) {
    auto topic_name = receiver->add_processer(
      config.topic_name, config.msg_type, std::make_unique<tam::network::UdpReceiver>(config.port));
    std::cout << "     Topic: " << topic_name << " | Type: " << config.msg_type
              << " | Port: " << config.port << std::endl;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(receiver);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}