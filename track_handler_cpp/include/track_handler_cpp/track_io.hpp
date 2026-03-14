// Copyright 2023 Simon Hoffmann
#pragma once
#include <bit>      // std::bit_cast
#include <cstdint>  // uint64_t
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <optional>
#include <vector>

#include "track_handler_cpp/track_types.hpp"
#include "tum_helpers_cpp/file_handling.hpp"
namespace tam::common
{
inline std::optional<RacelineData::Key> get_raceline_key_from_header(const std::string & header)
{
  if (header == "s_ref_rl_m") {
    return RacelineData::s;
  }
  if (header == "v_rl_mps") {
    return RacelineData::v;
  }
  if (header == "n_rl_m") {
    return RacelineData::n;
  }
  if (header == "chi_rl_rad") {
    return RacelineData::chi;
  }
  if (header == "ax_rl_mps2") {
    return RacelineData::ax;
  }
  if (header == "ay_rl_mps2") {
    return RacelineData::ay;
  }
  if (header == "jx_rl_mps3") {
    return RacelineData::jx;
  }
  if (header == "jy_rl_mps3") {
    return RacelineData::jy;
  }
  return std::nullopt;
}
inline std::optional<TrackData::Key> get_track_key_from_header(
  const std::string & header, TrackReferenceLines ref_line = TrackReferenceLines::RACELINE)
{
  std::string ref = "rl";
  switch (ref_line) {
    case TrackReferenceLines::RACELINE:
      ref = "rl";
      break;
    case TrackReferenceLines::CENTERLINE:
      ref = "cl";
      break;
    default:
      throw std::invalid_argument(
        "[track_handler_cpp]: Invalid Reference line specified for loading Track");
      break;
  }

  if (header == "s_ref_" + ref + "_m") {
    return TrackData::s;
  }
  if (header == "x_ref_" + ref + "_m") {
    return TrackData::x;
  }
  if (header == "y_ref_" + ref + "_m") {
    return TrackData::y;
  }
  if (header == "z_ref_" + ref + "_m") {
    return TrackData::z;
  }
  if (header == "theta_ref_" + ref + "_rad") {
    return TrackData::theta;
  }
  if (header == "mu_ref_" + ref + "_rad") {
    return TrackData::mu;
  }
  if (header == "phi_ref_" + ref + "_rad") {
    return TrackData::phi;
  }
  if (header == "dtheta_ref_" + ref + "_radpm") {
    return TrackData::dtheta;
  }
  if (header == "dmu_ref_" + ref + "_radpm") {
    return TrackData::dmu;
  }
  if (header == "dphi_ref_" + ref + "_radpm") {
    return TrackData::dphi;
  }
  if (header == "omega_x_ref_" + ref + "_radpm") {
    return TrackData::omega_x;
  }
  if (header == "omega_y_ref_" + ref + "_radpm") {
    return TrackData::omega_y;
  }
  if (header == "omega_z_ref_" + ref + "_radpm") {
    return TrackData::omega_z;
  }
  if (header == "w_tr_right_ref_" + ref + "_m") {
    return TrackData::w_right;
  }
  if (header == "w_tr_left_ref_" + ref + "_m") {
    return TrackData::w_left;
  }
  if (header == "w_tr_right_s_based_ref_" + ref + "_m") {
    return TrackData::w_right_s_based;
  }
  if (header == "w_tr_left_s_based_ref_" + ref + "_m") {
    return TrackData::w_left_s_based;
  }
  if (header == "ssa_w_tr_right_ref_" + ref + "_m") {
    return TrackData::w_right_ssa;
  }
  if (header == "ssa_w_tr_left_ref_" + ref + "_m") {
    return TrackData::w_left_ssa;
  }
  if (header == "ssa_w_tr_right_s_based_ref_" + ref + "_m") {
    return TrackData::w_right_ssa_s_based;
  }
  if (header == "ssa_w_tr_left_s_based_ref_" + ref + "_m") {
    return TrackData::w_left_ssa_s_based;
  }
  if (header == "sda_w_tr_right_ref_" + ref + "_m") {
    return TrackData::w_right_sda;
  }
  if (header == "sda_w_tr_left_ref_" + ref + "_m") {
    return TrackData::w_left_sda;
  }
  if (header == "sda_w_tr_right_s_based_ref_" + ref + "_m") {
    return TrackData::w_right_sda_s_based;
  }
  if (header == "sda_w_tr_left_s_based_ref_" + ref + "_m") {
    return TrackData::w_left_sda_s_based;
  }

  return std::nullopt;
}
inline RacelineData get_raceline_from_file(const std::string & path)
{
  Eigen::MatrixXd test = tam::helpers::files::load_csv<Eigen::MatrixXd>(path);
  std::vector<std::string> header = tam::helpers::files::get_csv_header(path);

  RacelineData data_;
  for (size_t i = 0; i < header.size(); i++) {
    auto key = get_raceline_key_from_header(header.at(i));
    if (key.has_value()) {
      // Remove nan
      auto iterator = std::find_if(
        test.col(i).begin(), test.col(i).end(), [](auto & i) { return std::isnan(i); });
      int idx = std::distance(test.col(i).begin(), iterator);
      data_.data.at(key.value()) = test.col(i).head(idx);
    }
  }
  return data_;
}
// inline bool is_nan_bits(double x) noexcept {
//   const uint64_t u = std::bit_cast<uint64_t>(x);
//   // exponent all 1s and mantissa non-zero => NaN
//   return ((u & 0x7ff0000000000000ULL) == 0x7ff0000000000000ULL) &&
//          ((u & 0x000fffffffffffffULL) != 0ULL);
// }

// inline TrackData get_track_from_file(
// const std::string& path,
// TrackReferenceLines ref = TrackReferenceLines::RACELINE)
// {
//   Eigen::MatrixXd test = tam::helpers::files::load_csv<Eigen::MatrixXd>(path);
//   std::vector<std::string> header = tam::helpers::files::get_csv_header(path);

//   TrackData data_;
//   for (size_t i = 0; i < header.size(); ++i) {
//     auto key = get_track_key_from_header(header.at(i), ref);
//     if (!key.has_value()) continue;

//     // Column view
//     const auto col = test.col(static_cast<Eigen::Index>(i));

//     // Find first NaN via bitwise check (works under -ffast-math)
//     Eigen::Index cut = col.size();
//     for (Eigen::Index r = 0; r < col.size(); ++r) {
//       if (is_nan_bits(col(r))) { cut = r; break; }
//     }

//     // Store a deep copy of the valid prefix
//     data_.data.at(key.value()) = col.head(cut).eval();
//   }

//   if (data_.data.at(TrackData::w_left_s_based).size() == 0 ||
//       data_.data.at(TrackData::w_right_s_based).size() == 0) {
//     std::cout << "\n\n\n"
//               << "============================================================\n"
//               << "  WARNING: w_left_s_based and w_right_s_based not found in file!\n"
//               << "  Specified RACELINE FILE is old and does not support s_based TB feature.\n"
//               << "  Filling w_left_s_based and w_right_s_based with w_left and w_right values.\n"
//               << "============================================================\n\n\n";
//     data_.data.at(TrackData::w_left_s_based)  = data_.data.at(TrackData::w_left);
//     data_.data.at(TrackData::w_right_s_based) = data_.data.at(TrackData::w_right);
//   }

//   return data_;
// }

inline bool is_nan_bits(double x) noexcept
{
  const uint64_t u = std::bit_cast<uint64_t>(x);
  return ((u & 0x7ff0000000000000ULL) == 0x7ff0000000000000ULL) &&
         ((u & 0x000fffffffffffffULL) != 0ULL);
}
inline TrackData get_track_from_file(
  const std::string & path, TrackReferenceLines ref = TrackReferenceLines::RACELINE)
{
  Eigen::MatrixXd test = tam::helpers::files::load_csv<Eigen::MatrixXd>(path);
  std::vector<std::string> header = tam::helpers::files::get_csv_header(path);

#ifndef NDEBUG
  std::cerr << "[TRACK DEBUG] " << path << "\n"
            << "  CSV matrix: rows=" << test.rows() << "  cols=" << test.cols() << "\n"
            << "  header size=" << header.size() << "\n";
#endif

  TrackData data_;
  for (size_t i = 0; i < header.size(); ++i) {
    auto key = get_track_key_from_header(header.at(i), ref);
    if (!key.has_value()) continue;

    const Eigen::Index col_idx = static_cast<Eigen::Index>(i);
    if (col_idx >= test.cols()) {
#ifndef NDEBUG
      std::cerr << "  [TRACK DEBUG] header[" << i << "]='" << header[i]
                << "': column index beyond matrix cols (" << test.cols() << "), skipping.\n";
#endif
      continue;
    }

    const auto col = test.col(col_idx);

    // Find first NaN
    Eigen::Index cut = col.size();
    for (Eigen::Index r = 0; r < col.size(); ++r) {
      if (is_nan_bits(col(r))) {
        cut = r;
        break;
      }
    }

    data_.data.at(key.value()) = col.head(cut).eval();

#ifndef NDEBUG
    std::cerr << "  [TRACK DEBUG] map '" << header[i] << "' -> key=" << key.value()
              << "  col_size=" << col.size()
              << "  first_NaN_at=" << (cut == col.size() ? -1 : static_cast<long>(cut))
              << "  assigned_len=" << data_.data.at(key.value()).size() << "\n";
#endif
  }

#ifndef NDEBUG
  auto print_size = [&](const char * name, int k) {
    // If all slots are guaranteed to exist, this is fine:
    std::cerr << "    " << name << ": " << data_.data.at(k).size() << "\n";

    // If you want to be ultra defensive (optional):
    // std::size_t sz = 0;
    // try { sz = static_cast<std::size_t>(data_.data.at(k).size()); }
    // catch (...) { sz = 0; }
    // std::cerr << "    " << name << ": " << sz << "\n";
  };

  std::cerr << "  [TRACK DEBUG] final vector sizes:\n";
  print_size("s", TrackData::s);
  print_size("x", TrackData::x);
  print_size("y", TrackData::y);
  print_size("z", TrackData::z);
  print_size("theta", TrackData::theta);
  print_size("mu", TrackData::mu);
  print_size("phi", TrackData::phi);
  print_size("w_left", TrackData::w_left);
  print_size("w_right", TrackData::w_right);
  print_size("w_left_s_based", TrackData::w_left_s_based);
  print_size("w_right_s_based", TrackData::w_right_s_based);
#endif

  if (
    data_.data.at(TrackData::w_left_s_based).size() == 0 ||
    data_.data.at(TrackData::w_right_s_based).size() == 0) {
    std::cout << "\n\n\n"
              << "============================================================\n"
              << "  WARNING: w_left_s_based and w_right_s_based not found in file!\n"
              << "  Specified RACELINE FILE is old and does not support s_based TB feature.\n"
              << "  Filling w_left_s_based and w_right_s_based with w_left and w_right values.\n"
              << "============================================================\n\n\n";
    data_.data.at(TrackData::w_left_s_based) = data_.data.at(TrackData::w_left);
    data_.data.at(TrackData::w_right_s_based) = data_.data.at(TrackData::w_right);
  }

  // Secondary areas READ GUARD
  if (
    data_.data.at(TrackData::w_right_ssa).size() == 0 ||
    data_.data.at(TrackData::w_left_ssa).size() == 0 ||
    data_.data.at(TrackData::w_right_ssa_s_based).size() == 0 ||
    data_.data.at(TrackData::w_left_ssa_s_based).size() == 0 ||
    data_.data.at(TrackData::w_right_sda).size() == 0 ||
    data_.data.at(TrackData::w_left_sda).size() == 0 ||
    data_.data.at(TrackData::w_right_sda_s_based).size() == 0 ||
    data_.data.at(TrackData::w_left_sda_s_based).size() == 0) {
    std::cout << "\n\n\n"
              << "============================================================\n"
              << "  WARNING: Secondary Areas Not available\n"
              << "  Specified RACELINE FILE is old and does not support SDA and SSA feature.\n"
              << "  Filling everything with w_right_s_based and w_left_s_based values.\n"
              << "============================================================\n\n\n";
    data_.data.at(TrackData::w_left_ssa) = data_.data.at(TrackData::w_left_s_based);
    data_.data.at(TrackData::w_right_ssa) = data_.data.at(TrackData::w_right_s_based);
    data_.data.at(TrackData::w_left_ssa_s_based) = data_.data.at(TrackData::w_left_s_based);
    data_.data.at(TrackData::w_right_ssa_s_based) = data_.data.at(TrackData::w_right_s_based);
    data_.data.at(TrackData::w_left_sda) = data_.data.at(TrackData::w_left_s_based);
    data_.data.at(TrackData::w_right_sda) = data_.data.at(TrackData::w_right_s_based);
    data_.data.at(TrackData::w_left_sda_s_based) = data_.data.at(TrackData::w_left_s_based);
    data_.data.at(TrackData::w_right_sda_s_based) = data_.data.at(TrackData::w_right_s_based);
  }

#ifndef NDEBUG
  // Extra guard for angles before precompute_trig() ever runs.
  const auto th_sz = data_.data.at(TrackData::theta).size();
  const auto mu_sz = data_.data.at(TrackData::mu).size();
  const auto ph_sz = data_.data.at(TrackData::phi).size();
  if (th_sz == 0 || mu_sz == 0 || ph_sz == 0 || !(th_sz == mu_sz && mu_sz == ph_sz)) {
    std::cerr << "[TRACK DEBUG] angle-size mismatch before init(): "
              << "theta=" << th_sz << "  mu=" << mu_sz << "  phi=" << ph_sz << "\n";
  }
#endif

  return data_;
}
// inline TrackData get_track_from_file(
//   const std::string & path, TrackReferenceLines ref = TrackReferenceLines::RACELINE)
// {
//   Eigen::MatrixXd test = tam::helpers::files::load_csv<Eigen::MatrixXd>(path);
//   std::vector<std::string> header = tam::helpers::files::get_csv_header(path);

//   TrackData data_;
//   for (size_t i = 0; i < header.size(); i++) {
//     auto key = get_track_key_from_header(header.at(i), ref);
//     if (key.has_value()) {
//       // Remove nan
//       auto iterator = std::find_if(
//         test.col(i).begin(), test.col(i).end(), [](auto & i) { return std::isnan(i); });
//       int idx = std::distance(test.col(i).begin(), iterator);
//       data_.data.at(key.value()) = test.col(i).head(idx);
//     }
//   }

//   if (data_.data.at(TrackData::w_left_s_based).size() == 0 ||
//       data_.data.at(TrackData::w_right_s_based).size() == 0) {
//     std::cout << "\n"
//               << "\n"
//               << "\n"
//               << "============================================================\n"
//               << "  WARNING: w_left_s_based and w_right_s_based not found in file!\n"
//               << "  SPecified RACELINE FILE is old and does not support s_based TB feature.\n"
//               << "  Filling w_left_s_based and w_right_s_based with w_left and w_right values.\n"
//               << "============================================================\n"
//               << "\n"
//               << "\n"
//               << std::endl;
//     data_.data.at(TrackData::w_left_s_based) = data_.data.at(TrackData::w_left);
//     data_.data.at(TrackData::w_right_s_based) = data_.data.at(TrackData::w_right);
//   }

//   return data_;
// }

}  // namespace tam::common