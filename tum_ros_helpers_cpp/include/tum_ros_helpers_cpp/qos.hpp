// Copyright 2024 Dominic
#pragma once
#include <rclcpp/rclcpp.hpp>
namespace tam::ros
{
// Specify the type of topic
enum TopicType {
  DEFAULT,
  PERCEPTION_SENSOR_DATA,
  EVENT  // For topics that are sent once instead of continuously.
};
inline rclcpp::QoS get_qos(TopicType topic_type = TopicType::DEFAULT)
{
  switch (topic_type) {
    case TopicType::EVENT:
      return rclcpp::QoS(10).reliable().durability_volatile().keep_last(10);
    case TopicType::PERCEPTION_SENSOR_DATA:
    case TopicType::DEFAULT:
      return rclcpp::QoS(1).best_effort().durability_volatile().keep_last(1);
  }
}
}  // namespace tam::ros
