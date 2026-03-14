// Copyright 2024 Marcel Weinmann
#include <pthread.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <queue>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sstream>
#include <std_msgs/msg/u_int64.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// TAM Packages
#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"
namespace tam::logging
{
class BagRecorder : public rclcpp::Node
{
  struct RecievedMsg
  {
    std::string topic;
    std::string msg_type;
    std::shared_ptr<rclcpp::SerializedMessage> msg;
    rclcpp::Time time_stamp;
    // Add a constructor to initialize the members
    RecievedMsg(
      const std::string & topic, const std::string & msg_type,
      std::shared_ptr<rclcpp::SerializedMessage> msg, rclcpp::Time time_stamp)
    : topic(topic), msg_type(msg_type), msg(msg), time_stamp(time_stamp)
    {
    }
  };

public:
  explicit BagRecorder(rclcpp::NodeOptions options) : Node("BagRecorder", options)
  {
    // declare node parameter
    this->declare_parameter("input_csv_path", "");

    // initialize the bag writer
    std::string output_bag_path = this->declare_parameter("output_bag_path", "");
    if (output_bag_path == "") std::cerr << "logging | no valid output file path" << std::endl;

    // Initialize monitor
    this->monitor_ = std::make_unique<tam::core::NodeMonitor>(this);

    // Initialize timer
    this->pub_timer_ =
      tam::create_timer(this, 500ms, std::bind(&BagRecorder::timer_callback, this));

    // Check if you should use an extra write thread
    use_extra_write_thread_ = this->declare_parameter("use_extra_write_thread", true);

    // Configure Storage Options
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = this->get_parameter("output_bag_path").as_string();
    storage_opts.storage_id = "mcap";
    storage_opts.max_bagfile_size = this->declare_parameter("max_bagfile_size", 2000000000l);
    storage_opts.max_bagfile_duration = this->declare_parameter("max_bagfile_duration", 0l);
    storage_opts.max_cache_size = this->declare_parameter(
      "max_cache_size",
      100 * 1024 * 1024);  // This is the default setting of the normal ros2 bag recorder
    storage_opts.storage_preset_profile =
      this->declare_parameter("storage_preset_profile", "zstd_fast");
    storage_opts.storage_config_uri = this->declare_parameter("storage_config_uri", "");

    // Configure converter options
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(), rmw_get_serialization_format()});

    // Create the bag writer
    __writer_ = std::make_unique<rosbag2_cpp::Writer>();
    __writer_->open(storage_opts, converter_options);

    // Get history depth for subscribers
    std::size_t history_depth = this->declare_parameter("history_depth", 100);
    write_queue_depth_ = this->declare_parameter("write_queue_depth", 100);

    // Parse topic and types based on the csv and create generic subscriptions
    std::ifstream file(this->get_parameter("input_csv_path").as_string());
    if (!file.is_open()) std::cerr << "logging | can't open csv file" << std::endl;
    std::string line{};
    std::getline(file, line);
    while (std::getline(file, line)) {
      std::stringstream string_line(line);
      std::string topic, msg_type, reliability_type, durability_type;
      std::getline(string_line, topic, ',');
      std::getline(string_line, msg_type, ',');
      std::getline(string_line, reliability_type, ',');
      std::getline(string_line, durability_type, ',');
      durability_type.erase(
        std::remove(durability_type.begin(), durability_type.end(), '\r'), durability_type.end());
      if (sub_map_.find(topic) == sub_map_.end()) {
        sub_map_[topic] = this->create_generic_subscription(
          topic, msg_type, create_qos_profile(history_depth, reliability_type, durability_type),
          [this, topic, msg_type](std::shared_ptr<rclcpp::SerializedMessage> message) {
            this->on_subscription(topic, msg_type, message);
          });

        rosbag2_storage::TopicMetadata topic_metadata;
        topic_metadata.name = topic;
        topic_metadata.type = msg_type;
        topic_metadata.serialization_format = rmw_get_serialization_format();

        __writer_->create_topic(topic_metadata);
      }
    }
    file.close();

    if (!use_extra_write_thread_) {
      return;
    }

    // Spawn a separate worker thread to write the messages to disk
    __thread_running_ = true;
    write_thread_ = std::thread([this]() {
      std::chrono::steady_clock::time_point write_start;
      bool first_iteration{true};
      while (true) {
        std::unique_lock<std::mutex> lock(work_pkg_mutex_);
        // Write an artificial message to the bag showing the anount of messages in the work package
        // byusing the bag writer
        work_cv_.wait(lock, [this] { return !__work_pkg_.empty() || __stop_thread_; });
        // Writing statistics
        auto old_write_start = write_start;
        write_start = std::chrono::steady_clock::now();

        std_msgs::msg::UInt64 msg_size, msg_duration, msg_time_between_successive_write_triggers;
        msg_size.data = __work_pkg_.size();
        msg_time_between_successive_write_triggers.data =
          std::chrono::duration_cast<std::chrono::microseconds>(write_start - old_write_start)
            .count();
        // Write the messages to the bag
        for (const auto & message : __work_pkg_) {
          __writer_->write(message.msg, message.topic, message.msg_type, message.time_stamp);
        }
        __work_pkg_.clear();  // Clear the work package after writing

        // More writing statistics
        msg_duration.data = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - write_start)
                              .count();

        if (__stop_thread_) {
          // Exit the thread if stop is requested and no work is left
          __thread_running_ = false;
          lock.unlock();
          break;
        }
        __writer_->write<std_msgs::msg::UInt64>(
          msg_duration, "/bag_statistics/extra_write_thread/queue_write_duration_us", this->now());
        __writer_->write<std_msgs::msg::UInt64>(
          msg_size, "/bag_statistics/extra_write_thread/queue_size_before_write", this->now());
        if (first_iteration) {
          first_iteration = false;
        } else {
          __writer_->write<std_msgs::msg::UInt64>(
            msg_time_between_successive_write_triggers,
            "/bag_statistics/extra_write_thread/time_between_successive_write_triggers",
            this->now());
        }
        lock.unlock();
      }
    });

    // Set low scheduling priority for the thread
    pthread_t native_handle = write_thread_.native_handle();
    struct sched_param param;
    param.sched_priority = 0;  // Lowest priority
    if (pthread_setschedparam(native_handle, SCHED_IDLE, &param) != 0) {
      std::cerr << "Failed to set thread scheduling priority" << std::endl;
    }
    write_thread_.detach();
    // Finish initialization
    monitor_->initialization_finished();
  }
  ~BagRecorder()
  {
    // First delete all subscriptions
    sub_map_.clear();

    // Signal the thread to stop
    if (use_extra_write_thread_) {
      {
        std::lock_guard<std::mutex> lock(work_pkg_mutex_);
        __stop_thread_ = true;
      }
      work_cv_.notify_one();  // Wake up the thread to allow it to exit

      while (true) {
        bool thread_running{false};
        {
          std::cout << "Waiting for the extra write thread to finish" << std::endl;
          std::lock_guard<std::mutex> lock(work_pkg_mutex_);
          thread_running = __thread_running_;
        }
        if (!thread_running) {
          std::cout << "Extra thread exited" << std::endl;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    // Close the writer
    __writer_ = nullptr;
  }

private:
  void timer_callback()
  {
    // Only publish running to indicate the logger is alive
    this->monitor_->set_error_lvl("Running", tam::types::ErrorLvl::OK);
    // Update monitor
    this->monitor_->update();
  }
  /**
   * write the ros2 msg to the bag
   */
  void on_subscription(
    const std::string & topic, const std::string & msg_type,
    std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    if (!use_extra_write_thread_) {
      // Diectly write the message to the bag if not using a separate thread
      __writer_->write(msg, topic, msg_type, this->now());
      return;
    }

    // Add message to the queue when using the separate thread.
    msg_queue_.emplace(topic, msg_type, msg, this->now());
    if (msg_queue_.size() >= write_queue_depth_) {
      if (!work_pkg_mutex_.try_lock()) return;
      __work_pkg_.reserve(std::min(3 * write_queue_depth_, msg_queue_.size()) + __work_pkg_.size());
      std::size_t iteration_cnt{0};
      while (!msg_queue_.empty() && iteration_cnt < 3 * write_queue_depth_) {
        __work_pkg_.push_back(msg_queue_.front());
        msg_queue_.pop();
        iteration_cnt++;
      }
      work_pkg_mutex_.unlock();
      work_cv_.notify_one();
    }
  }
  rclcpp::QoS create_qos_profile(
    std::size_t history_depth, std::string reliability, std::string durability)
  {
    // Convert to lowercase for case-insensitive comparison
    rclcpp::QoS qos_profile(history_depth);
    std::transform(reliability.begin(), reliability.end(), reliability.begin(), ::tolower);
    std::transform(durability.begin(), durability.end(), durability.begin(), ::tolower);

    // Set reliability
    if (reliability == "reliable") {
      qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    } else if (reliability == "besteffort" || reliability == "best_effort") {
      qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    } else {
      std::cerr << "Invalid reliability setting: " + reliability << std::endl;
    }

    // Set durability
    if (durability == "transientlocal" || durability == "transient_local") {
      qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
    } else if (durability == "volatile") {
      qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    } else {
      std::cerr << "Invalid durability setting: " + durability << std::endl;
    }

    return qos_profile;
  }

private:
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> sub_map_{};
  std::unique_ptr<rosbag2_cpp::Writer> __writer_{};
  std::size_t write_queue_depth_{100};
  bool use_extra_write_thread_{true};

  // Extra write thread
  std::queue<RecievedMsg> msg_queue_{};
  std::thread write_thread_;
  std::mutex work_pkg_mutex_;
  std::condition_variable work_cv_;
  // Before accessing the variables the mutex need to be locked
  bool __stop_thread_{false};
  std::vector<RecievedMsg> __work_pkg_{};
  bool __thread_running_{false};
  // Node monitor
  tam::core::NodeMonitor::UniquePtr monitor_{};
  rclcpp::TimerBase::SharedPtr pub_timer_;
};
}  // namespace tam::logging
