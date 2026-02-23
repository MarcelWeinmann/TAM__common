// Copyright 2025 Maximilian Leitenstern
#pragma once
#include <tf2_ros/buffer.h>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace tam::ros
{
/**
 * @brief Wait for a static transform to be available in the buffer.
 * @param node_ptr Pointer to the node.
 * @param buffer Pointer to the tf2 buffer.
 * @param target The target frame.
 * @param source The source frame.
 * @param wait_interval_ms The interval to wait for the transform to be available (in milliseconds).
 * @return The transform between the target and source frames.
 * @note This function will block until the transform is available or the node is shut down.
 */
inline geometry_msgs::msg::TransformStamped wait_static_transform(
  rclcpp::Node * node_ptr, tf2_ros::Buffer * buffer, const std::string & target,
  const std::string & source, int wait_interval_ms = 3000)
{
  // Check for availability of transform
  bool tf_wait = true;
  while (tf_wait) {
    try {
      // Lookup transformation from source to target frame
      buffer->lookupTransform(target, source, tf2::TimePointZero);
      RCLCPP_INFO(node_ptr->get_logger(), "Transforms are available");
      tf_wait = false;
    } catch (tf2::TransformException & ex) {
      // If the transform is not available, catch the exception and continue waiting
      RCLCPP_ERROR(node_ptr->get_logger(), "Transform not available: %s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(wait_interval_ms));
    }
  }
  // Read the transforms from the buffer
  return buffer->lookupTransform(target, source, tf2::TimePointZero);
}
/**
 * @brief transform tf2 transform to Eigen Isometry transform
 *        Eigen::Isometry is the same as Eigen::Affine but with the assumption that
 *        the linear part (left top 3x3 matrix) is a rotation. This is used to speed
 *        up several computations
 *
 * @param[in] transform          - geometry_msgs::msg::TransformStamped:
 *                                 input transform
 * @param[out]                   - Eigen::Isometry3f
 *                                 transformation matrix in Eigen format
 */
inline Eigen::Isometry3f transform2eigen(const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
  const auto & t = transform.transform;
  iso.translate(Eigen::Vector3f{t.translation.x, t.translation.y, t.translation.z});
  iso.rotate(Eigen::Quaternionf{t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z});
  return iso;
}
}  // namespace tam::ros
