// Copyright 2023 Simon Hoffmann
#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "param_management_cpp/param_value_manager.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"
#include "track_handler_cpp/raceline.hpp"
#include "track_handler_cpp/track.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::common
{
class RaceTrackHandler
{
public:
  /**
   * @brief Creates a RaceTrackHandler using either the package default
   *        config_overwrite path (when overwrite_root is empty) or a custom one.
   *
   * @param overwrite_root Optional custom root containing "config.yml" and "<TrackName>/..."
   * @return std::unique_ptr<RaceTrackHandler>
   * @throws std::invalid_argument if a non-empty overwrite_root does not exist or is not a
   * directory
   */
  static std::unique_ptr<RaceTrackHandler> from_pkg_config(
    const std::filesystem::path & overwrite_root = {});
  /**
   * @brief Create a track object. Depending on track_handler_cpp/config_overwrite, a different
   * Track is loaded
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_track_prediction() const;
  /**
   * @brief DEPRECATED: use create_raceline_track_prediction()
   * Create a track object. Depending on track_handler_cpp/config_overwrite, a different
   * Track is loaded
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_track();
  /**
   * @brief DEPRECATED: use create_raceline_track()
   * Create a track object. Depending on track_handler_cpp/config_overwrite, a different
   * Track is loaded
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_raceline_track_prediction() const;
  /**
   * @brief Create a track object. Depending on track_handler_cpp/config_overwrite, a different
   * Track is loaded
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_raceline_track();
  /**
   * @brief Create a track object that has a CENTERLINE as reference line. ATTENTIONE: Do not use
   * for calculations with raceline!
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_centerline_track() const;
  /**
   * @brief Create a track object that has a CENTERLINE as reference line. ATTENTIONE: Do not use
   * for calculations with raceline!
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_centerline_track_prediction() const;
  /**
   * @brief Create a track object. Depending on track_handler_cpp/config_overwrite, a different
   * Pitlane is loaded
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_pitlane_prediction() const;
  /**
   * @brief Create a track object. Depending on track_handler_cpp/config_overwrite, a different
   * Pitlane is loaded
   *
   * @return std::unique_ptr<Track>
   */
  std::unique_ptr<Track> create_pitlane() const;
  /**
   * @brief Create a raceline object. Depending on track_handler_cpp/config_overwrite, a different
   * Raceline is loaded
   *
   * @return std::unique_ptr<Raceline>
   */
  std::unique_ptr<Raceline> create_raceline_prediction() const;
  /**
   * @brief Create a raceline object. Depending on track_handler_cpp/config_overwrite, a different
   * Raceline is loaded. Also ensures a preloaded "main" handler exists in raceline_handlers_.
   *
   * @return std::unique_ptr<Raceline>
   */
  std::unique_ptr<Raceline> create_raceline();

  /**
   * @brief (Deprecated) Const wrapper for create_raceline().
   */
  [[deprecated("Use the non-const overload create_raceline().")]] std::unique_ptr<Raceline>
  create_raceline() const;
  /**
   * @brief Create a raceline object for the pitlane. Depending on
   * track_handler_cpp/config_overwrite, a different Raceline is loaded
   *
   * @return std::unique_ptr<Raceline>
   */
  std::unique_ptr<Raceline> create_raceline_pitlane() const;
  /**
   * @brief Returns the current Track-Key (e.g. Vegas, Monza, etc,)
   *
   * @return track key
   */
  std::string get_track_name() const { return track; }
  /**
   * @brief Returns the path from where track-files are currently loaded
   *
   * @return track path
   */
  std::string get_track_path_default(const std::string & track_key) const;
  /**
   * @brief Returns the path from where raceline-files are currently loaded
   *
   * @return raceline path
   */
  std::string get_raceline_path_default(const std::string & track_key) const;
  /**
   * @brief Returns the name of the .csv file which is loaded as Track
   *
   * @return csv. filename of track
   */
  std::string get_track_file() const;
  /**
   * @brief Returns the name of the .csv file which is loaded as Pit
   *
   * @return csv. filename of pit
   */
  std::string return_raceline_path() const;
  /**
   * @brief Returns the path of the .csv file which is loaded as Raceline
   *
   * @return csv. pathname of raceline
   */
  std::string get_pit_file() const;
  /**
   * @brief Returns the name of the .csv file which is loaded as Raceline
   *
   * @return csv. filename of raceline
   */
  std::string return_pitlane_path() const;
  /**
   * @brief Returns the path of the .csv file which is loaded as Pitlane
   *
   * @return csv. pathname of pitlane
   */
  std::string get_raceline_file() const;
  /**
   * @brief Get the initial heading at the track
   *
   * @return initial heading double
   */
  std::string get_track_file_pred() const;
  /**
   * @brief Returns the name of the .csv file which is loaded as Pit
   *
   * @return csv. filename of pit
   */
  std::string get_pit_file_pred() const;
  /**
   * @brief Returns the name of the .csv file which is loaded as Raceline
   *
   * @return csv. filename of raceline
   */
  std::string get_raceline_file_pred() const;
  /**
   * @brief Get the initial heading at the track
   *
   * @return initial heading double
   */
  double get_initial_heading() const;
  /**
   * @brief Returns the origin of the local cartesian track coordinate system in geodetic
   * coordinates (lat, long, h)
   *
   * @return tam::types::common::Vector3D<double> [lat, long, h]
   */
  tam::types::common::Vector3D<double> get_geo_origin() const;
  /**
   * @brief Returns the starting position of vehicle pos_id, specified in config.yml
   * @param pos_id Argument to select which start position to return starting with 1 (Starting with
   * multiple vehicles in sim)
   * @return std::vector<double> with 3 elements [x, y, yaw]
   */
  std::vector<double> get_sim_start_pos(int pos_id) const;
  /**
   * @brief Get a specific parameter specified in e.g. config_overwrite/Monza/config.yml
   *
   * @param param_name name of the parameter (e.g. speed_limit.slow_zone)
   * @return tam::types::param::ParameterValue
   */
  tam::pmg::ParameterValue get_param(const std::string & param_name) const;

  /**
   * @brief Returns a preloaded Track instance for a given raceline directory (e.g. "left", "right",
   * "pit")
   *
   * @return std::shared_ptr<Track>
   */
  std::shared_ptr<Track> get_track_handler(const std::string & name) const;
  /**
   * @brief Returns a preloaded Raceline instance for a given raceline directory (e.g. "left",
   * "right", "pit")
   *
   * @return std::shared_ptr<Raceline>
   */
  std::shared_ptr<Raceline> get_raceline_handler(const std::string & name) const;
  /**
   * @brief Returns a preloaded CENTERLINE-based Track instance for a given raceline directory (e.g.
   * "left", "right", "pit")
   *
   * @return std::shared_ptr<Track>
   */
  std::shared_ptr<Track> get_track_handler_centerline(const std::string & name) const;

  // ───────────────────────────────────────── New: list available handlers
  // ─────────────────────────────────────────
  /**
   * @brief List all available raceline-based Track handlers created by from_pkg_config().
   *        The handler's internal get_name() gives its identifier (e.g. "pit", "left").
   *
   * @return vector of shared_ptr<Track>
   */
  std::vector<std::shared_ptr<Track>> list_track_handlers() const;

  /**
   * @brief List all available Raceline objects created by from_pkg_config().
   *        The handler's internal get_name() gives its identifier (e.g. "pit", "left").
   *
   * @return vector of shared_ptr<Raceline>
   */
  std::vector<std::shared_ptr<Raceline>> list_raceline_handlers() const;
  // race_track_handler.cpp — add this method definition
  std::filesystem::path get_config_overwrite_path() const { return overwrite_path; }

private:
  RaceTrackHandler();
  std::filesystem::path overwrite_path;
  tam::pmg::ParamValueManager::UniquePtr param_manager_ =
    std::make_unique<tam::pmg::ParamValueManager>();
  static std::string load_from_env(const std::string & key);
  std::string track{""};
  static void declare_and_load_params(
    std::unique_ptr<RaceTrackHandler> & th, const std::string & config_folder);
  static void load_track_name(
    std::unique_ptr<RaceTrackHandler> & th, const std::string & config_folder);

  std::unordered_map<std::string, std::shared_ptr<Track>> track_handlers_;
  std::unordered_map<std::string, std::shared_ptr<Raceline>> raceline_handlers_;
  std::unordered_map<std::string, std::shared_ptr<Track>> centerline_track_handlers_;
};
}  // namespace tam::common
