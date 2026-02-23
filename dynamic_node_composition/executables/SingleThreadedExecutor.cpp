// Copyright 2025 Simon Sagmeister
#include "dynamic_node_composition/launch.hpp"
int main(int argc, char * argv[])
{
  // Launch the dynamic node composition with the SingleThreadedExecutor
  tam::dynamic_composition::launch<rclcpp::executors::SingleThreadedExecutor, false>(argc, argv);
  return 0;
}
