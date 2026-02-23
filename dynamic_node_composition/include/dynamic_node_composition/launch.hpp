// Copyright 2025 Simon Sagmeister
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "dynamic_node_composition/composition.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_handler_cpp/vehicle_handler.hpp"
namespace tam::dynamic_composition
{
template <typename ExecutorT, bool validate_parameters = true>
void launch(int argc, char * argv[])
{
  std::cout << "===========================================" << std::endl;
  std::cout << "==== Starting Dynamic Node Composition ====" << std::endl;
  std::cout << "===========================================" << std::endl;

  auto components_to_load =
    tam::dynamic_composition::parse_components_from_command_line_arguments(argc, argv);

  // Init rclcpp
  rclcpp::init(argc, argv);

  std::unique_ptr<tam::common::VehicleHandler> vehicle_ =
    tam::common::VehicleHandler::from_pkg_config();
  vehicle_->init_param_backend();
  std::cout << "Initialzed Parameter backend via VehicleHandler!" << std::endl;

  // Create Node options
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.enable_rosout(false);
  options.start_parameter_event_publisher(false);
  options.enable_topic_statistics(false);

  // Create the component loader
  // Make sure to keep the component loader in scope for the lifetime of the nodes
  tam::dynamic_composition::ComponentLoader component_loader;

  auto nodes = component_loader.load_and_create_nodes(components_to_load, options);

  if (nodes.empty()) {
    std::cout << "No components have been specified! Exiting!" << std::endl;
    return;
  }

  // Validating parameters
  if constexpr (validate_parameters) {
    std::cout << "==== Validating parameter overrides ====" << std::endl;
    for (const auto & node : nodes) {
      tam::pmg::validate_param_overrides(
        argc, argv, node.get());  // Protects against trying to set non existant parameters
    }
    std::cout << "=============== Success ================" << std::endl;
  }

  // Create executor and load nodes
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  std::cout << "==== Adding nodes to executor ====" << std::endl;
  for (const auto & node : nodes) {
    std::cout << ">> Adding node to executor | " << node->get_fully_qualified_name() << std::endl;
    executor.add_node(node->get_node_base_interface());
  }

  executor.spin();

  std::cout << "================================================" << std::endl;
  std::cout << "==== Shutting Down Dynamic Node Composition ====" << std::endl;
  std::cout << "================================================" << std::endl;

  // Remove all the nodes from the executor
  for (const auto & node : nodes) {
    std::cout << ">> Removing node from executor | " << node->get_fully_qualified_name()
              << std::endl;
    executor.remove_node(node->get_node_base_interface());
  }
  nodes.clear();

  rclcpp::shutdown();
}
}  // namespace tam::dynamic_composition
