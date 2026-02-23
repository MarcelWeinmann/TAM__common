// Copyright 2024 Marcel Weinmann
#include <logging/recorder.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Create the required default constructor to have a composable node
using BagRecorderGeneric = tam::logging::BagRecorder;
namespace logging
{
struct BagRecorder : public BagRecorderGeneric
{
  explicit BagRecorder(const rclcpp::NodeOptions & options) : BagRecorderGeneric(options) {}
};
}  // namespace logging
RCLCPP_COMPONENTS_REGISTER_NODE(logging::BagRecorder)
