// Copyright 2026 TUMFTM
#pragma once

#warning \
  "TSLDebugReader is deprecated. Please use the tsl_ros2_utils_cpp package from github.com/TUMFTM/tsl instead." // NOLINT

#include <string>
#include <vector>

#include "tsl_msgs/msg/tsl_definition.hpp"
#include "tsl_msgs/msg/tsl_values.hpp"
class TSLDebugReader
{
public:
  // A simple structure to store information for one monitored signal.
  struct MonitoredSignal
  {
    std::string name = "unknown";  // Signal name
    // True if the signal was found in float32_names, false if in bool_names.
    bool is_float = false;
    // The index in the corresponding vector. Use static_cast<size_t>(-1) if not found.
    size_t index = static_cast<size_t>(-1);
  };
  /// Constructor takes the list of target signal names.

  /**
   * @brief Constructor for TSLDebugReader.
   *
   * @param[in] target_signal_names List of names of signals for which to monitor values.
   */
  explicit TSLDebugReader(const std::vector<std::string> & target_signal_names)
  : target_signal_names_(target_signal_names)
  {
  }
  /// Update the internal mapping from signal names to indices.
  /// Returns a vector of MonitoredSignal structures.

  /**
   * @brief Update the internal mapping from signal names to indices.
   * Has to be done only once if the msgs definition does not change over time.
   *
   * This function takes a TSLDefinition message and updates the internal mapping
   * from signal names to indices. It returns a vector of MonitoredSignal structures.
   *
   * @param[in] definition_msg TSLDefinition message. Must not be nullptr.
   * @return A vector of MonitoredSignal structures, one for each target signal
   * name. If a target signal name is not found in the definition, the index is
   * set to -1.
   */
  std::vector<MonitoredSignal> updateIndices(
    const tsl_msgs::msg::TSLDefinition::SharedPtr & definition_msg)
  {
    // Clear any previous mapping.
    monitored_signals_.clear();

    // For each target signal, search for it in the definition.
    for (const auto & name : target_signal_names_) {
      bool found = false;

      // Look in the float32_names first.
      for (size_t i = 0; i < definition_msg->float32_names.size(); ++i) {
        if (definition_msg->float32_names[i] == name) {
          monitored_signals_.push_back({name, true, i});
          found = true;
          break;
        }
      }

      // If not found in float32, look in bool_names.
      if (!found) {
        for (size_t i = 0; i < definition_msg->bool_names.size(); ++i) {
          if (definition_msg->bool_names[i] == name) {
            monitored_signals_.push_back({name, false, i});
            found = true;
            break;
          }
        }
      }

      // If not found, store an invalid index.
      if (!found) {
        monitored_signals_.push_back({name, false, static_cast<size_t>(-1)});
      }
    }

    return monitored_signals_;
  }
  /// Given a TSL values message, returns a vector with the values for the monitored signals.

  /**
   * @brief Get the values of the monitored signals from a TSLValues message.
   *
   * This is a function that loops through the monitored signals and
   * extracts the values from the TSLValues message. The output vector will have the
   * same order as the monitored_signals_ vector.
   *
   * @param[in] values_msg A shared pointer to a TSLValues message.
   * @return A vector of float values, one for each monitored signal.
   */
  std::vector<float> getValues(const tsl_msgs::msg::TSLValues::SharedPtr & values_msg) const
  {
    std::vector<float> output;

    // Loop through the monitored signals.
    for (const auto & signal : monitored_signals_) {
      float value = 0.0f;
      if (signal.index != static_cast<size_t>(-1)) {
        if (signal.is_float) {
          if (signal.index < values_msg->float32_values.size()) {
            value = values_msg->float32_values[signal.index];
          }
        } else {
          if (signal.index < values_msg->bool_values.size()) {
            // Convert bool to float (true becomes 1.0, false becomes 0.0).
            value = values_msg->bool_values[signal.index] ? 1.0f : 0.0f;
          }
        }
      }
      output.push_back(value);
    }

    return output;
  }

private:
  std::vector<std::string> target_signal_names_;
  std::vector<MonitoredSignal> monitored_signals_;
};
