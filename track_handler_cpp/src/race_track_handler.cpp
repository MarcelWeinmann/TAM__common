// Copyright 2023 Simon Hoffmann
#include "track_handler_cpp/race_track_handler.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
namespace tam::common
{
namespace fs = std::filesystem;
RaceTrackHandler::RaceTrackHandler()
{
  overwrite_path =
    ament_index_cpp::get_package_share_directory("track_handler_cpp").append("/config_overwrite");
}
std::unique_ptr<RaceTrackHandler> RaceTrackHandler::from_pkg_config(const fs::path & overwrite_root)
{
  // private ctor OK here
  std::unique_ptr<RaceTrackHandler> th(new RaceTrackHandler());

  // If caller provided a custom overwrite root, use it; otherwise keep ctor's default
  if (!overwrite_root.empty()) {
    th->overwrite_path = fs::absolute(overwrite_root);
    if (!fs::exists(th->overwrite_path) || !fs::is_directory(th->overwrite_path)) {
      throw std::invalid_argument(
        "[RaceTrackHandler::from_pkg_config] Path does not exist or is not a directory: " +
        th->overwrite_path.string());
    }
  }

  // Load track name & params based on overwrite_path
  load_track_name(th, th->overwrite_path);
  declare_and_load_params(th, th->overwrite_path);

  // Preload one raceline file for each directory (e.g. left, right, pit, ...)
  fs::path base = fs::path(th->overwrite_path).append(th->track).append("raceline");
  if (fs::exists(base) && fs::is_directory(base)) {
    // Each subdirectory represents a handler key (e.g. "left", "right", "pit", ...)
    for (const auto & dir_entry : fs::directory_iterator(base)) {
      if (!fs::is_directory(dir_entry.path())) {
        continue;
      }
      std::string key = dir_entry.path().filename().string();

      fs::path selected_csv;
      for (const auto & file_entry : fs::directory_iterator(dir_entry.path())) {
        if (!fs::is_regular_file(file_entry.path())) {
          continue;
        }
        if (file_entry.path().extension() != ".csv") {
          continue;
        }
        const std::string fname = file_entry.path().filename().string();
        if (selected_csv.empty()) {
          selected_csv = file_entry.path();
        }
        if (
          fname.find("raceline") != std::string::npos &&
          selected_csv.filename().string().find("raceline") == std::string::npos) {
          selected_csv = file_entry.path();
        }
      }
      if (selected_csv.empty()) {
        continue;
      }

      auto t_up = Track::create_from_csv(selected_csv.string(), TrackReferenceLines::RACELINE);
      auto r_up = Raceline::create_from_csv(selected_csv.string());
      t_up->set_handler_name(key);
      r_up->set_handler_name(key);
      th->track_handlers_[key] = std::shared_ptr<Track>(std::move(t_up));
      th->raceline_handlers_[key] = std::shared_ptr<Raceline>(std::move(r_up));
      // additionally preload a CENTERLINE-based track handler for the same source
      auto t_center_up =
        Track::create_from_csv(selected_csv.string(), TrackReferenceLines::CENTERLINE);
      t_center_up->set_handler_name(key);
      th->centerline_track_handlers_[key] = std::shared_ptr<Track>(std::move(t_center_up));
      std::cout << "[RaceTrackHandler]: Preloaded '" << key << "' - " << selected_csv << "\n";
    }
  }

  return th;
}
std::string RaceTrackHandler::get_track_path_default(const std::string & track_key) const
{
  return fs::path(overwrite_path).append(track_key).append("raceline").string();
}
std::string RaceTrackHandler::get_raceline_path_default(const std::string & track_key) const
{
  return fs::path(overwrite_path).append(track_key).append("raceline").string();
}
void RaceTrackHandler::load_track_name(
  std::unique_ptr<RaceTrackHandler> & th, const std::string & config_folder)
{
  fs::path path = fs::path(config_folder).append("config.yml");
  if (!fs::exists(path)) {
    throw std::invalid_argument("Config file not found: " + path.string());
  }
  YAML::Node config = YAML::LoadFile(path.string());
  if (config["track"]) {
    th->track = config["track"].as<std::string>();
    if (boost::starts_with(th->track, "$")) {
      th->track.erase(boost::remove_if(th->track, boost::is_any_of("${}")), th->track.end());
      th->track = load_from_env(th->track);
    }
  } else {
    throw std::invalid_argument(
      std::string{"Value "} + std::string{"\"track\""} + std::string{" not set in file: "} +
      path.string());
  }
}
std::string RaceTrackHandler::load_from_env(const std::string & key)
{
  char * val = getenv(key.c_str());
  if (val == NULL) {
    throw std::invalid_argument(
      std::string{"Environment Variable \""} + key + std::string{"\" not found"});
  }

  return std::string(val);
}
void RaceTrackHandler::declare_and_load_params(
  std::unique_ptr<RaceTrackHandler> & th, const std::string & config_folder)
{
  // Declare
  th->param_manager_->declare_parameter("global.pitlane", " ", tam::pmg::ParameterType::STRING, "");
  th->param_manager_->declare_parameter(
    "global.raceline", " ", tam::pmg::ParameterType::STRING, "");
  th->param_manager_->declare_parameter(
    "global.initial_heading", 0.0, tam::pmg::ParameterType::DOUBLE, "");
  th->param_manager_->declare_parameter(
    "global.geo_origin", std::vector<double>{0.0, 0.0, 0.0}, tam::pmg::ParameterType::DOUBLE_ARRAY,
    "");
  th->param_manager_->declare_parameter(
    "prediction.pitlane", " ", tam::pmg::ParameterType::STRING, "");
  th->param_manager_->declare_parameter(
    "prediction.raceline", " ", tam::pmg::ParameterType::STRING, "");
  // Simulation
  th->param_manager_->declare_parameter(
    "simulation.start_pos_1", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");
  th->param_manager_->declare_parameter(
    "simulation.start_pos_2", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");
  th->param_manager_->declare_parameter(
    "simulation.start_pos_3", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");
  th->param_manager_->declare_parameter(
    "simulation.start_pos_4", std::vector<double>{0.0, 0.0, 0.0},
    tam::pmg::ParameterType::DOUBLE_ARRAY, "");

  fs::path path = fs::path(config_folder).append(th->track).append("config.yml");
  tam::pmg::load_overwrites_from_yaml(
    th->param_manager_.get(), path.string(), "/TrackHandler", true, true);
}
std::unique_ptr<Track> RaceTrackHandler::create_track()
{
  std::cout << "[RaceTrackHandler.create_track()]: DEPRECATED: use "
               "create_raceline_track() "
            << "\n";
  return create_raceline_track();
}
std::unique_ptr<Track> RaceTrackHandler::create_raceline_track()
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_track_file());
  std::cout << "[RaceTrackHandler]: Loading Track - " << path << "\n";

  // Ensure a handler named "main" is registered and accessible via get_track_handler("main").
  // (Lazily register; do not overwrite if it already exists.)
  if (track_handlers_.find("main") == track_handlers_.end()) {
    auto t_main_up = Track::create_from_csv(path, TrackReferenceLines::RACELINE);
    t_main_up->set_handler_name("main");
    track_handlers_["main"] = std::shared_ptr<Track>(std::move(t_main_up));
  } else {
    // Keep the name consistent even if already present (idempotent).
    track_handlers_.at("main")->set_handler_name("main");
  }

  // Return a fresh instance to the caller; also named "main" for consistency.
  auto ret = Track::create_from_csv(path, TrackReferenceLines::RACELINE);
  ret->set_handler_name("main");
  return ret;
}
std::unique_ptr<Track> RaceTrackHandler::create_centerline_track() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_track_file());
  std::cout << "[RaceTrackHandler]: Loading Track - " << path << "\n";
  return Track::create_from_csv(path, TrackReferenceLines::CENTERLINE);
}
std::unique_ptr<Track> RaceTrackHandler::create_pitlane() const
{
  fs::path path = fs::path(overwrite_path).append(track).append("raceline").append(get_pit_file());
  std::cout << "[RaceTrackHandler]: Loading Pitlane - " << path << "\n";
  return Track::create_from_csv(path);
}
// ─────────────────────────────── UPDATED METHOD (mirrors "main" registration)
// ───────────────────────────────
std::unique_ptr<Raceline> RaceTrackHandler::create_raceline()
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_raceline_file());
  std::cout << "[RaceTrackHandler]: Loading Raceline - " << path << "\n";

  // Ensure a handler named "main" is registered and accessible via get_raceline_handler("main").
  // (Lazily register; do not overwrite if it already exists.)
  if (raceline_handlers_.find("main") == raceline_handlers_.end()) {
    auto r_main_up = Raceline::create_from_csv(path.string());
    r_main_up->set_handler_name("main");
    raceline_handlers_["main"] = std::shared_ptr<Raceline>(std::move(r_main_up));
  } else {
    // Keep the name consistent even if already present (idempotent).
    raceline_handlers_.at("main")->set_handler_name("main");
  }

  // Return a fresh instance to the caller; also named "main" for consistency.
  auto ret = Raceline::create_from_csv(path.string());
  ret->set_handler_name("main");
  return ret;
}
// Const wrapper kept for ABI/backward compatibility; deprecated in the header.
std::unique_ptr<Raceline> RaceTrackHandler::create_raceline() const
{
  return const_cast<RaceTrackHandler *>(this)->create_raceline();
}
std::unique_ptr<Raceline> RaceTrackHandler::create_raceline_pitlane() const
{
  fs::path path = fs::path(overwrite_path).append(track).append("raceline").append(get_pit_file());
  std::cout << "[RaceTrackHandler]: Loading Pitlane Raceline - " << path << "\n";
  return Raceline::create_from_csv(path);
}
std::unique_ptr<Track> RaceTrackHandler::create_track_prediction() const
{
  std::cout << "[RaceTrackHandler.create_track_prediction()]: DEPRECATED: use "
               "create_raceline_track_prediction() "
            << "\n";
  return create_raceline_track_prediction();
}
std::unique_ptr<Track> RaceTrackHandler::create_raceline_track_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_track_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Track - " << path << "\n";
  return Track::create_from_csv(path, TrackReferenceLines::RACELINE);
}
std::unique_ptr<Track> RaceTrackHandler::create_centerline_track_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_track_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Track - " << path << "\n";
  return Track::create_from_csv(path, TrackReferenceLines::CENTERLINE);
}
std::unique_ptr<Track> RaceTrackHandler::create_pitlane_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_pit_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Pitlane - " << path << "\n";
  return Track::create_from_csv(path);
}
std::unique_ptr<Raceline> RaceTrackHandler::create_raceline_prediction() const
{
  fs::path path =
    fs::path(overwrite_path).append(track).append("raceline").append(get_raceline_file_pred());
  std::cout << "[RaceTrackHandler]: Loading Prediction Raceline - " << path << "\n";
  return Raceline::create_from_csv(path);
}
std::string RaceTrackHandler::get_track_file() const
{
  return param_manager_->get_value("global.raceline").as_string();
}
std::string RaceTrackHandler::return_raceline_path() const
{
  return fs::path(overwrite_path)
    .append(track)
    .append("raceline")
    .append(get_track_file())
    .string();
}
std::string RaceTrackHandler::get_pit_file() const
{
  return param_manager_->get_value("global.pitlane").as_string();
}
std::string RaceTrackHandler::return_pitlane_path() const
{
  return fs::path(overwrite_path).append(track).append("raceline").append(get_pit_file()).string();
}
std::string RaceTrackHandler::get_raceline_file() const
{
  return param_manager_->get_value("global.raceline").as_string();
}
double RaceTrackHandler::get_initial_heading() const
{
  return param_manager_->get_value("global.initial_heading").as_double();
}
std::string RaceTrackHandler::get_raceline_file_pred() const
{
  return param_manager_->get_value("prediction.raceline").as_string();
}
std::string RaceTrackHandler::get_track_file_pred() const
{
  return param_manager_->get_value("prediction.raceline").as_string();
}
std::string RaceTrackHandler::get_pit_file_pred() const
{
  return param_manager_->get_value("prediction.pitlane").as_string();
}
tam::types::common::Vector3D<double> RaceTrackHandler::get_geo_origin() const
{
  return tam::types::common::Vector3D<double>(
    param_manager_->get_value("global.geo_origin").as_double_array());
}
std::vector<double> RaceTrackHandler::get_sim_start_pos(int pos_id) const
{
  return param_manager_->get_value("simulation.start_pos_" + std::to_string(pos_id))
    .as_double_array();
}
tam::pmg::ParameterValue RaceTrackHandler::get_param(const std::string & param_name) const
{
  return param_manager_->get_value(param_name);
}
std::shared_ptr<Track> RaceTrackHandler::get_track_handler(const std::string & name) const
{
  auto it = track_handlers_.find(name);
  if (it == track_handlers_.end()) {
    throw std::invalid_argument("[RaceTrackHandler]: No Track found for key: " + name);
  }
  return it->second;
}
std::shared_ptr<Track> RaceTrackHandler::get_track_handler_centerline(
  const std::string & name) const
{
  auto it = centerline_track_handlers_.find(name);
  if (it == centerline_track_handlers_.end()) {
    throw std::invalid_argument("[RaceTrackHandler]: No Centerline Track found for key: " + name);
  }
  return it->second;
}
std::shared_ptr<Raceline> RaceTrackHandler::get_raceline_handler(const std::string & name) const
{
  auto it = raceline_handlers_.find(name);
  if (it == raceline_handlers_.end()) {
    throw std::invalid_argument("[RaceTrackHandler]: No Raceline found for key: " + name);
  }
  return it->second;
}
// ───────────────────────────────────────── New: list available handlers
// ─────────────────────────────────────────
std::vector<std::shared_ptr<Track>> RaceTrackHandler::list_track_handlers() const
{
  std::vector<std::shared_ptr<Track>> out;
  out.reserve(track_handlers_.size());
  for (const auto & kv : track_handlers_) {
    out.emplace_back(kv.second);
  }
  return out;
}
std::vector<std::shared_ptr<Raceline>> RaceTrackHandler::list_raceline_handlers() const
{
  std::vector<std::shared_ptr<Raceline>> out;
  out.reserve(raceline_handlers_.size());
  for (const auto & kv : raceline_handlers_) {
    out.emplace_back(kv.second);
  }
  return out;
}
}  // namespace tam::common
