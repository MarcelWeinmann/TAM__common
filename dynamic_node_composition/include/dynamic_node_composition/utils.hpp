// Copyright 2024 Simon Sagmeister
#pragma once
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/has_resource.hpp"
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/split.hpp"
namespace tam::dynamic_composition::utils
{
std::string get_shared_library_path(ComponentDescription const & comp);
}  // namespace tam::dynamic_composition::utils
