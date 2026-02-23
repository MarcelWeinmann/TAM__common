// Copyright 2025 Maximilian Leitenstern

#pragma once

#include <deque>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
namespace tam::ros
{
/**
 * @brief Class for monitoring the frequency of a ROS2 node.
 *
 * Keeps track of publishing times in a rolling window and
 * calculates the publishing frequency.
 */
class FrequencyMonitor
{
public:
  /**
   * @brief Construct a new Frequency Monitor with specified window size
   *
   * @param window_size Number of timestamps to keep in buffer
   */
  explicit FrequencyMonitor(size_t window_size = 100) : window_size_(window_size)
  {
    if (window_size_ == 0) {
      throw std::invalid_argument("Window size must be greater than 0");
    }
  }
  /**
   * @brief Record a new timestamp
   */
  void set_time(const rclcpp::Time & stamp)
  {
    timestamps_.push_back(stamp);

    if (timestamps_.size() > window_size_) {
      timestamps_.pop_front();
    }
  }
  /**
   * @brief Get the current publishing frequency in Hz
   *
   * @return double Frequency in Hz or 0.0 if not enough data points
   */
  double get_frequency() const
  {
    if (timestamps_.size() < 2) {
      return 0.0;
    }
    double duration = (timestamps_.back() - timestamps_.front()).seconds();
    if (duration <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(timestamps_.size() - 1) / duration;
  }
  /**
   * @brief Clear all recorded timestamps
   */
  void reset() { timestamps_.clear(); }

private:
  size_t window_size_{};
  std::deque<rclcpp::Time> timestamps_{};
};
}  // namespace tam::ros
