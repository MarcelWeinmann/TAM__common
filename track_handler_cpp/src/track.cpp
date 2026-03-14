// Copyright 2023 Simon Hoffmann
#include "track_handler_cpp/track.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "track_handler_cpp/acceleration.hpp"
#include "track_handler_cpp/rotation.hpp"
#include "track_handler_cpp/track_io.hpp"
#include "tum_helpers_cpp/numerical.hpp"
namespace tam::common
{
Track::Track(TrackData && data)
{
  // Init data
  data_ = std::make_unique<TrackData>(std::move(data));
  // Init Track
  this->init();
}
void Track::init()
{
  // Todo: set heading & s from hear
  this->cosy_ =
    tam::helpers::cosy::CurvilinearCosy::create(
      data_->data.at(TrackData::x), data_->data.at(TrackData::y), data_->data.at(TrackData::z))
      ->build();
  std::cout << "[track_handler_cpp]: Loaded track with length " << this->length() << "!\n";
  precompute_trig();
  std::cout << "[track_handler_cpp]: Precomputed trigonometric functions.\n";
  // create_normal_vector(); // precompute trig already includes this
  create_trackbounds();
  std::cout << "[track_handler_cpp]: Created track bounds.\n";
  calc_d_omega();
  std::cout << "[track_handler_cpp]: Calculated d_omega.\n";
  if (!tam::helpers::track::has_equal_sized_data(data_->data)) {
    throw std::invalid_argument(
      "[track_handler_cpp]: Trying to load track content with unequal size");
  }
  std::cout << "[track_handler_cpp]: Track data has equal size.\n";
}
std::unique_ptr<Track> Track::create_from_csv(const std::string & path, TrackReferenceLines ref)
{  // make unique doesn't allow protected access
  return std::unique_ptr<Track>(new Track(tam::common::get_track_from_file(path, ref)));
}
void Track::create_normal_vector()
{
  const Eigen::ArrayXd theta = this->theta().array();
  const Eigen::ArrayXd mu = this->mu().array();
  const Eigen::ArrayXd phi = this->phi().array();
  Eigen::MatrixXd normal;
  normal.resize(this->s_coord().rows(), 3);
  normal.col(0) = theta.cos() * mu.sin() * phi.sin() - theta.sin() * phi.cos();
  normal.col(1) = theta.sin() * mu.sin() * phi.sin() + theta.cos() * phi.cos();
  normal.col(2) = mu.cos() * phi.sin();
  normal.rowwise().normalize();
  this->data_->data.at(TrackData::normal_x) = normal.col(0);
  this->data_->data.at(TrackData::normal_y) = normal.col(1);
  this->data_->data.at(TrackData::normal_z) = normal.col(2);
}
void Track::create_trackbounds()
{
  data_->data.at(TrackData::tb_left_x) =
    data_->data.at(TrackData::x).array() +
    data_->data.at(TrackData::normal_x).array() * data_->data.at(TrackData::w_left).array();
  data_->data.at(TrackData::tb_left_y) =
    data_->data.at(TrackData::y).array() +
    data_->data.at(TrackData::normal_y).array() * data_->data.at(TrackData::w_left).array();
  data_->data.at(TrackData::tb_left_z) =
    data_->data.at(TrackData::z).array() +
    data_->data.at(TrackData::normal_z).array() * data_->data.at(TrackData::w_left).array();
  //
  data_->data.at(TrackData::tb_right_x) =
    data_->data.at(TrackData::x).array() +
    data_->data.at(TrackData::normal_x).array() * data_->data.at(TrackData::w_right).array();
  data_->data.at(TrackData::tb_right_y) =
    data_->data.at(TrackData::y).array() +
    data_->data.at(TrackData::normal_y).array() * data_->data.at(TrackData::w_right).array();
  data_->data.at(TrackData::tb_right_z) =
    data_->data.at(TrackData::z).array() +
    data_->data.at(TrackData::normal_z).array() * data_->data.at(TrackData::w_right).array();
}
void Track::calc_d_omega()
{
  data_->data.at(TrackData::domega_x) =
    tam::helpers::numerical::gradient(this->omega_x(), this->s_coord());
  data_->data.at(TrackData::domega_y) =
    tam::helpers::numerical::gradient(this->omega_y(), this->s_coord());
  data_->data.at(TrackData::domega_z) =
    tam::helpers::numerical::gradient(this->omega_z(), this->s_coord());
}
bool Track::on_track(const double x, const double y, const double margin = 0.0) const
{
  auto sn = cosy_->convert_to_sn_and_get_idx(x, y);
  double w_left =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_left(), std::get<1>(sn));
  double w_right =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_right(), std::get<1>(sn));
  return (std::get<0>(sn)[1] >= w_right - margin) && (std::get<0>(sn)[1] <= w_left + margin);
}
bool Track::on_track_global(
  const double x, const double y, const double z, const double z_range,
  const double margin = 0.0) const
{
  auto sn = cosy_->convert_to_sn_and_get_idx_global_2d(x, y, z, z_range);
  double w_left =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_left(), std::get<1>(sn));
  double w_right =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_right(), std::get<1>(sn));
  return (std::get<0>(sn)[1] >= w_right - margin) && (std::get<0>(sn)[1] <= w_left + margin);
}
bool Track::on_track_window(
  const double x, const double y, const double z, const double s_start, const double s_end,
  const double z_range, const double margin = 0.0) const
{
  auto sn = cosy_->convert_to_sn_and_get_idx_window_2d(x, y, z, s_start, s_end, z_range);
  double w_left =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_left(), std::get<1>(sn));
  double w_right =
    tam::helpers::numerical::interp_from_idx(this->trackwidth_right(), std::get<1>(sn));
  return (std::get<0>(sn)[1] >= w_right - margin) && (std::get<0>(sn)[1] <= w_left + margin);
}
// void Track::precompute_trig()
// {
//   // Access the stored vectors directly to avoid dangling Array wrappers
//   const Eigen::VectorXd& th_v = data_->data.at(TrackData::theta);  // θ
//   const Eigen::VectorXd& mu_v = data_->data.at(TrackData::mu);     // μ
//   const Eigen::VectorXd& ph_v = data_->data.at(TrackData::phi);    // φ

// #ifndef NDEBUG
//   // Cheap sanity checks (catch mismatches before we touch Array wrappers)
//   assert(th_v.size() > 0 && "theta is empty");
//   assert(mu_v.size() == th_v.size() && "mu has different length than theta");
//   assert(ph_v.size() == th_v.size() && "phi has different length than theta");
// #endif

//   // Create Array views from the *referenced* vectors (safe)
//   const auto th = th_v.array();
//   const auto mu = mu_v.array();
//   const auto ph = ph_v.array();

//   // Cache sin/cos
//   const Eigen::ArrayXd sθ = th.sin();
//   const Eigen::ArrayXd cθ = th.cos();
//   const Eigen::ArrayXd sμ = mu.sin();
//   const Eigen::ArrayXd cμ = mu.cos();
//   const Eigen::ArrayXd sφ = ph.sin();
//   const Eigen::ArrayXd cφ = ph.cos();

//   data_->data[TrackData::theta_sin] = sθ.matrix();
//   data_->data[TrackData::theta_cos] = cθ.matrix();
//   data_->data[TrackData::mu_sin]    = sμ.matrix();
//   data_->data[TrackData::mu_cos]    = cμ.matrix();
//   data_->data[TrackData::phi_sin]   = sφ.matrix();
//   data_->data[TrackData::phi_cos]   = cφ.matrix();

//   // Convenience products
//   const Eigen::ArrayXd cθsμ = cθ * sμ;
//   const Eigen::ArrayXd sθsμ = sθ * sμ;

//   // Rotation R = Z(θ) Y(μ) X(φ), columns ex, ey, ez
//   // ex = [ cθ cμ,  sθ cμ,  -sμ ]
//   data_->data[TrackData::tangent_x] = ( cθ * cμ).matrix();
//   data_->data[TrackData::tangent_y] = ( sθ * cμ).matrix();
//   data_->data[TrackData::tangent_z] = (-sμ     ).matrix();

//   // ey = [ -cφ sθ + sφ cθ sμ,  cφ cθ + sφ sθ sμ,  sφ cμ ]
//   data_->data[TrackData::normal_x]  = (-cφ * sθ + sφ * cθsμ).matrix();
//   data_->data[TrackData::normal_y]  = ( cφ * cθ + sφ * sθsμ).matrix();
//   data_->data[TrackData::normal_z]  = ( sφ * cμ             ).matrix();

//   // ez = [  sφ sθ + cφ cθ sμ,  -sφ cθ + cφ sθ sμ,  cφ cμ ]
//   data_->data[TrackData::binormal_x] = ( sφ * sθ + cφ * cθsμ).matrix();
//   data_->data[TrackData::binormal_y] = (-sφ * cθ + cφ * sθsμ).matrix();
//   data_->data[TrackData::binormal_z] = ( cφ * cμ             ).matrix();
// }

void Track::precompute_trig()
{
  // 1) Fetch inputs once
  const Eigen::VectorXd & th_v = data_->data.at(TrackData::theta);  // θ
  const Eigen::VectorXd & mu_v = data_->data.at(TrackData::mu);     // μ
  const Eigen::VectorXd & ph_v = data_->data.at(TrackData::phi);    // φ

  const Eigen::Index N = th_v.size();

#ifndef NDEBUG
  assert(N > 0 && "precompute_trig: theta is empty");
  assert(mu_v.size() == N && "precompute_trig: mu size mismatch");
  assert(ph_v.size() == N && "precompute_trig: phi size mismatch");
#endif

  // 2) Prepare outputs (create/resize in the map and get raw pointers)
  auto & th_s = data_->data[TrackData::theta_sin];
  th_s.resize(N);
  auto & th_c = data_->data[TrackData::theta_cos];
  th_c.resize(N);
  auto & mu_s = data_->data[TrackData::mu_sin];
  mu_s.resize(N);
  auto & mu_c = data_->data[TrackData::mu_cos];
  mu_c.resize(N);
  auto & ph_s = data_->data[TrackData::phi_sin];
  ph_s.resize(N);
  auto & ph_c = data_->data[TrackData::phi_cos];
  ph_c.resize(N);

  auto & tx = data_->data[TrackData::tangent_x];
  tx.resize(N);
  auto & ty = data_->data[TrackData::tangent_y];
  ty.resize(N);
  auto & tz = data_->data[TrackData::tangent_z];
  tz.resize(N);

  auto & nx = data_->data[TrackData::normal_x];
  nx.resize(N);
  auto & ny = data_->data[TrackData::normal_y];
  ny.resize(N);
  auto & nz = data_->data[TrackData::normal_z];
  nz.resize(N);

  auto & bx = data_->data[TrackData::binormal_x];
  bx.resize(N);
  auto & by = data_->data[TrackData::binormal_y];
  by.resize(N);
  auto & bz = data_->data[TrackData::binormal_z];
  bz.resize(N);

  const double * __restrict thp = th_v.data();
  const double * __restrict mup = mu_v.data();
  const double * __restrict php = ph_v.data();

  double * __restrict th_sp = th_s.data();
  double * __restrict th_cp = th_c.data();
  double * __restrict mu_sp = mu_s.data();
  double * __restrict mu_cp = mu_c.data();
  double * __restrict ph_sp = ph_s.data();
  double * __restrict ph_cp = ph_c.data();

  double * __restrict txp = tx.data();
  double * __restrict typ = ty.data();
  double * __restrict tzp = tz.data();

  double * __restrict nxp = nx.data();
  double * __restrict nyp = ny.data();
  double * __restrict nzp = nz.data();

  double * __restrict bxp = bx.data();
  double * __restrict byp = by.data();
  double * __restrict bzp = bz.data();

  // 3) One pass: compute sin/cos + full rotation columns
  for (Eigen::Index i = 0; i < N; ++i) {
    // fused trig
    double sθ, cθ;
    __builtin_sincos(thp[i], &sθ, &cθ);
    double sμ, cμ;
    __builtin_sincos(mup[i], &sμ, &cμ);
    double sφ, cφ;
    __builtin_sincos(php[i], &sφ, &cφ);

    th_sp[i] = sθ;
    th_cp[i] = cθ;
    mu_sp[i] = sμ;
    mu_cp[i] = cμ;
    ph_sp[i] = sφ;
    ph_cp[i] = cφ;

    const double cθsμ = cθ * sμ;
    const double sθsμ = sθ * sμ;

    // R = Z(θ) Y(μ) X(φ)
    // e_x (tangent): [ cθ cμ,  sθ cμ,  -sμ ]
    txp[i] = cθ * cμ;
    typ[i] = sθ * cμ;
    tzp[i] = -sμ;

    // e_y (normal):  [ -cφ sθ + sφ cθ sμ,  cφ cθ + sφ sθ sμ,  sφ cμ ]
    nxp[i] = -cφ * sθ + sφ * cθsμ;
    nyp[i] = cφ * cθ + sφ * sθsμ;
    nzp[i] = sφ * cμ;

    // e_z (binormal): [  sφ sθ + cφ cθ sμ,  -sφ cθ + cφ sθ sμ,  cφ cμ ]
    bxp[i] = sφ * sθ + cφ * cθsμ;
    byp[i] = -sφ * cθ + cφ * sθsμ;
    bzp[i] = cφ * cμ;
  }
}
std::tuple<double, double, double> Track::calc_apparent_acceleration(
  const double s, const double n, const double chi, const double ax, const double ay,
  const double V) const
{
  auto [ax_tilde, ay_tilde] = tam::common::track::calc_a_tilde(mu(s), phi(s), chi, ax, ay);
  double s_dot = tam::common::track::calc_s_dot(V, n, omega_z(s), chi);
  auto g_tilde =
    tam::common::track::calc_g_tilde(omega_x(s), omega_y(s), mu(s), phi(s), chi, s_dot, V);
  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Track::calc_apparent_acceleration(
  const Eigen::Ref<const Eigen::MatrixXd> s, const Eigen::Ref<const Eigen::MatrixXd> n,
  const Eigen::Ref<const Eigen::MatrixXd> chi, const Eigen::Ref<const Eigen::MatrixXd> ax,
  const Eigen::Ref<const Eigen::MatrixXd> ay, const Eigen::Ref<const Eigen::MatrixXd> V) const
{
  auto [ax_tilde, ay_tilde] = tam::common::track::calc_a_tilde(mu(s), phi(s), chi, ax, ay);
  Eigen::MatrixXd s_dot = tam::common::track::calc_s_dot(V, n, omega_z(s), chi);
  auto g_tilde =
    tam::common::track::calc_g_tilde(omega_x(s), omega_y(s), mu(s), phi(s), chi, s_dot, V);
  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<double, double, double> Track::calc_apparent_acceleration_index(
  const double /*s*/, const double n, const double chi, const double ax, const double ay,
  const double V, const int idx) const
{
  // Fetch scalars once
  const double mu_k = mu()(idx);
  const double phi_k = phi()(idx);
  const double ox_k = omega_x()(idx);
  const double oy_k = omega_y()(idx);
  const double oz_k = omega_z()(idx);

  // Fused trig
  double s_chi, c_chi;
  __builtin_sincos(chi, &s_chi, &c_chi);
  double s_mu, c_mu;
  __builtin_sincos(mu_k, &s_mu, &c_mu);
  double s_phi, c_phi;
  __builtin_sincos(phi_k, &s_phi, &c_phi);

  const double g = tam::constants::g_earth;

  // a_tilde
  const double ax_tilde = ax + g * (-s_mu * c_chi + c_mu * s_phi * s_chi);
  const double ay_tilde = ay + g * (s_mu * s_chi + c_mu * s_phi * c_chi);

  // s_dot
  const double denom = 1.0 - n * oz_k;
  const double s_dot = (V * c_chi) / denom;

  // g_tilde (no clamp)
  const double g_tilde = (ox_k * s_chi - oy_k * c_chi) * s_dot * V + g * c_mu * c_phi;

  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<double, double, double> Track::calc_apparent_acceleration_index_fast(
  const double /*s*/, const double n, const double chi, const double ax, const double ay,
  const double V, const int idx) const
{
  // Precomputed angle trig (filled by precompute_trig())
  const double s_mu = data_->data.at(TrackData::mu_sin)(idx);
  const double c_mu = data_->data.at(TrackData::mu_cos)(idx);
  const double s_phi = data_->data.at(TrackData::phi_sin)(idx);
  const double c_phi = data_->data.at(TrackData::phi_cos)(idx);

  // Raw track angular rates
  const double ox_k = omega_x()(idx);
  const double oy_k = omega_y()(idx);
  const double oz_k = omega_z()(idx);

  // Only χ is per-call input → compute once
  double s_chi, c_chi;
  __builtin_sincos(chi, &s_chi, &c_chi);

  const double g = tam::constants::g_earth;

  // a_tilde
  const double ax_tilde = ax + g * (-s_mu * c_chi + c_mu * s_phi * s_chi);
  const double ay_tilde = ay + g * (s_mu * s_chi + c_mu * s_phi * c_chi);

  // s_dot
  const double denom = 1.0 - n * oz_k;
  const double s_dot = (V * c_chi) / denom;

  // g_tilde
  const double g_tilde = (ox_k * s_chi - oy_k * c_chi) * s_dot * V + g * c_mu * c_phi;

  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
Track::calc_apparent_acceleration_index(
  const Eigen::Ref<const Eigen::MatrixXd>,  // s -  unused here (kept for signature parity)
  const Eigen::Ref<const Eigen::MatrixXd> n, const Eigen::Ref<const Eigen::MatrixXd> chi,
  const Eigen::Ref<const Eigen::MatrixXd> ax, const Eigen::Ref<const Eigen::MatrixXd> ay,
  const Eigen::Ref<const Eigen::MatrixXd> V, const Eigen::Ref<const Eigen::MatrixXi> idx) const
{
  const Eigen::Index R = idx.rows();
  const Eigen::Index C = idx.cols();

#ifndef NDEBUG
  auto same_shape = [&](const Eigen::MatrixXd & m) { return m.rows() == R && m.cols() == C; };
  assert(same_shape(n) && "n has different shape than idx");
  assert(same_shape(chi) && "chi has different shape than idx");
  assert(same_shape(ax) && "ax has different shape than idx");
  assert(same_shape(ay) && "ay has different shape than idx");
  assert(same_shape(V) && "V has different shape than idx");
#endif

  Eigen::MatrixXd ax_tilde(R, C);
  Eigen::MatrixXd ay_tilde(R, C);
  Eigen::MatrixXd g_tilde(R, C);

  // Cache track arrays once (avoid map lookups in the loop)
  const auto & mu_v = mu();
  const auto & phi_v = phi();
  const auto & ox_v = omega_x();
  const auto & oy_v = omega_y();
  const auto & oz_v = omega_z();

  const double g = tam::constants::g_earth;

  for (Eigen::Index r = 0; r < R; ++r) {
    for (Eigen::Index c = 0; c < C; ++c) {
      const int k = idx(r, c);
#ifndef NDEBUG
      assert(k >= 0 && k < mu_v.size());
#endif
      // Track parameters at index k
      const double mu_k = mu_v[k];
      const double phi_k = phi_v[k];
      const double ox_k = ox_v[k];
      const double oy_k = oy_v[k];
      const double oz_k = oz_v[k];

      // Inputs at (r,c)
      const double chi_rc = chi(r, c);
      const double ax_rc = ax(r, c);
      const double ay_rc = ay(r, c);
      const double V_rc = V(r, c);
      const double n_rc = n(r, c);

      // Fused trig where beneficial
      double s_chi, c_chi;
      __builtin_sincos(chi_rc, &s_chi, &c_chi);
      double s_mu, c_mu;
      __builtin_sincos(mu_k, &s_mu, &c_mu);
      double s_phi, c_phi;
      __builtin_sincos(phi_k, &s_phi, &c_phi);

      // a_tilde components (combine formulas, avoid extra trig)
      // ax_tilde = ax + g * ( -sin(mu)*cos(chi) + cos(mu)*sin(phi)*sin(chi) )
      const double ax_t = ax_rc + g * (-s_mu * c_chi + c_mu * s_phi * s_chi);
      // ay_tilde = ay + g * (  sin(mu)*sin(chi) + cos(mu)*sin(phi)*cos(chi) )
      const double ay_t = ay_rc + g * (s_mu * s_chi + c_mu * s_phi * c_chi);

      // s_dot = (V * cos(chi)) / (1 - n * Omega_z)
      const double denom = 1.0 - n_rc * oz_k;
      const double sdot = (V_rc * c_chi) / denom;

      // g_tilde = (Omega_x*sin(chi) - Omega_y*cos(chi)) * s_dot * V + g*cos(mu)*cos(phi)
      const double gt = (ox_k * s_chi - oy_k * c_chi) * sdot * V_rc + g * c_mu * c_phi;

      ax_tilde(r, c) = ax_t;
      ay_tilde(r, c) = ay_t;
      g_tilde(r, c) = gt;  // add std::max(0.0, gt) here if you want clamping
    }
  }

  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
Track::calc_apparent_acceleration_index_fast(
  const Eigen::Ref<const Eigen::MatrixXd> /*s*/,  // unused (kept for parity)
  const Eigen::Ref<const Eigen::MatrixXd> n, const Eigen::Ref<const Eigen::MatrixXd> chi,
  const Eigen::Ref<const Eigen::MatrixXd> ax, const Eigen::Ref<const Eigen::MatrixXd> ay,
  const Eigen::Ref<const Eigen::MatrixXd> V, const Eigen::Ref<const Eigen::MatrixXi> idx) const
{
  const Eigen::Index R = idx.rows();
  const Eigen::Index C = idx.cols();

#ifndef NDEBUG
  auto same_shape = [&](const Eigen::MatrixXd & m) { return m.rows() == R && m.cols() == C; };
  assert(same_shape(n) && "n has different shape than idx");
  assert(same_shape(chi) && "chi has different shape than idx");
  assert(same_shape(ax) && "ax has different shape than idx");
  assert(same_shape(ay) && "ay has different shape than idx");
  assert(same_shape(V) && "V has different shape than idx");
#endif

  Eigen::MatrixXd ax_tilde(R, C);
  Eigen::MatrixXd ay_tilde(R, C);
  Eigen::MatrixXd g_tilde(R, C);

  // --- raw pointers (column-major friendly) -------------------------------
  const int * __restrict p_idx = idx.data();
  const double * __restrict p_n = n.data();
  const double * __restrict p_chi = chi.data();
  const double * __restrict p_ax = ax.data();
  const double * __restrict p_ay = ay.data();
  const double * __restrict p_V = V.data();

  double * __restrict p_ax_out = ax_tilde.data();
  double * __restrict p_ay_out = ay_tilde.data();
  double * __restrict p_g_out = g_tilde.data();

  // Precomputed track trig (from precompute_trig())
  const Eigen::VectorXd & mu_s = data_->data.at(TrackData::mu_sin);
  const Eigen::VectorXd & mu_c = data_->data.at(TrackData::mu_cos);
  const Eigen::VectorXd & phi_s = data_->data.at(TrackData::phi_sin);
  const Eigen::VectorXd & phi_c = data_->data.at(TrackData::phi_cos);

  const double * __restrict p_mu_s = mu_s.data();
  const double * __restrict p_mu_c = mu_c.data();
  const double * __restrict p_phi_s = phi_s.data();
  const double * __restrict p_phi_c = phi_c.data();

  // Angular rates (raw)
  const double * __restrict p_ox = omega_x().data();
  const double * __restrict p_oy = omega_y().data();
  const double * __restrict p_oz = omega_z().data();

  const double g = tam::constants::g_earth;
  const Eigen::Index stride = R;  // column-major: next column = +R

  // Iterate column-major for cache-friendly access
  for (Eigen::Index c = 0; c < C; ++c) {
    const Eigen::Index base = c * stride;

    const int * __restrict idx_col = p_idx + base;
    const double * __restrict n_col = p_n + base;
    const double * __restrict chi_col = p_chi + base;
    const double * __restrict ax_col = p_ax + base;
    const double * __restrict ay_col = p_ay + base;
    const double * __restrict V_col = p_V + base;

    double * __restrict ax_out_col = p_ax_out + base;
    double * __restrict ay_out_col = p_ay_out + base;
    double * __restrict g_out_col = p_g_out + base;

    for (Eigen::Index r = 0; r < R; ++r) {
      const int k = idx_col[r];
#ifndef NDEBUG
      assert(k >= 0 && k < mu_s.size());
#endif
      // Track (precomputed + raw)
      const double s_mu = p_mu_s[k];
      const double c_mu = p_mu_c[k];
      const double s_phi = p_phi_s[k];
      const double c_phi = p_phi_c[k];
      const double ox_k = p_ox[k];
      const double oy_k = p_oy[k];
      const double oz_k = p_oz[k];

      // Inputs
      const double chi_rc = chi_col[r];
      const double ax_rc = ax_col[r];
      const double ay_rc = ay_col[r];
      const double V_rc = V_col[r];
      const double n_rc = n_col[r];

      // Only chi trig per element
      double s_chi, c_chi;
      __builtin_sincos(chi_rc, &s_chi, &c_chi);

      // a_tilde
      const double ax_t = ax_rc + g * (-s_mu * c_chi + c_mu * s_phi * s_chi);
      const double ay_t = ay_rc + g * (s_mu * s_chi + c_mu * s_phi * c_chi);

      // s_dot
      const double denom = 1.0 - n_rc * oz_k;
      const double sdot = (V_rc * c_chi) / denom;

      // g_tilde
      const double gt = (ox_k * s_chi - oy_k * c_chi) * sdot * V_rc + g * c_mu * c_phi;

      ax_out_col[r] = ax_t;
      ay_out_col[r] = ay_t;
      g_out_col[r] = gt;  // clamp with std::max(0.0, gt) if desired
    }
  }

  return {ax_tilde, ay_tilde, g_tilde};
}
std::tuple<double, double> Track::calc_acceleration(
  const double s, const double chi, const double ax_tilde, const double ay_tilde) const
{
  return tam::common::track::calc_a_hat(mu(s), phi(s), chi, ax_tilde, ay_tilde);
}
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> Track::calc_acceleration(
  const Eigen::Ref<const Eigen::MatrixXd> s, const Eigen::Ref<const Eigen::MatrixXd> chi,
  const Eigen::Ref<const Eigen::MatrixXd> ax_tilde,
  const Eigen::Ref<const Eigen::MatrixXd> ay_tilde) const
{
  return tam::common::track::calc_a_hat(mu(s), phi(s), chi, ax_tilde, ay_tilde);
}
Eigen::Vector3d Track::sn2cartesian(const double s, const double n) const
{
  Eigen::Vector3d norm;
  Eigen::Vector3d ref_p;
  for (int i = 0; i < 3; ++i) {
    norm(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_x + i));
    ref_p(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::x + i));
  }
  return ref_p.array() + (norm.array() * n).array();
}
Eigen::MatrixX3d Track::sn2cartesian(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> n) const
{
  Eigen::MatrixX3d norm(s.size(), 3);
  Eigen::MatrixX3d ref_p(s.size(), 3);

  for (int i = 0; i < 3; ++i) {
    norm.col(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::normal_x + i));
    ref_p.col(i) = tam::helpers::numerical::interp(
      s_mod(s, data_->data.at(TrackData::s)(Eigen::last)), data_->data.at(TrackData::s),
      data_->data.at(TrackData::x + i));
  }
  return ref_p.array() + norm.array().colwise() * n.array();
}
double Track::calc_2d_heading_from_chi(const double s, const double chi) const
{
  return tam::common::track::calc_2d_heading_from_chi(chi, theta(s), mu(s), phi(s));
}
Eigen::VectorXd Track::calc_2d_heading_from_chi(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> chi) const
{
  return tam::common::track::calc_2d_heading_from_chi(chi, theta(s), mu(s), phi(s));
}
Eigen::Vector3d Track::angles_to_velocity_frame(const double s, const double chi) const
{
  return tam::common::track::angles_to_velocity_frame(chi, theta(s), mu(s), phi(s));
}
Eigen::MatrixXd Track::angles_to_velocity_frame(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> chi) const
{
  return tam::common::track::angles_to_velocity_frame(chi, theta(s), mu(s), phi(s));
}
double Track::calc_chi_from_2d_heading(const double s, const double heading) const
{
  return tam::common::track::calc_chi_from_2d_heading(heading, theta(s), mu(s), phi(s));
}
Eigen::VectorXd Track::calc_chi_from_2d_heading(
  const Eigen::Ref<const Eigen::VectorXd> s, const Eigen::Ref<const Eigen::VectorXd> heading) const
{
  return tam::common::track::calc_chi_from_2d_heading(heading, theta(s), mu(s), phi(s));
}
Eigen::Vector2d Track::project_2d_point_on_track(const double x, const double y) const
{
  return cosy_->convert_to_sn(x, y);
}
Eigen::Vector2d Track::project_2d_point_on_track_global(
  const double x, const double y, const double z, const double z_range) const
{
  return cosy_->convert_to_sn_global_2d(x, y, z, z_range);
}
Eigen::Vector2d Track::project_2d_point_on_track_window(
  const double x, const double y, const double z, const double s_start, const double s_end,
  const double z_range) const
{
  return cosy_->convert_to_sn_window_2d(x, y, z, s_start, s_end, z_range);
}
Eigen::MatrixX2d Track::project_2d_point_on_track_global(
  Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<const Eigen::VectorXd> y,
  Eigen::Ref<const Eigen::VectorXd> z, const double z_range) const
{
  Eigen::MatrixX2d sn_out(x.rows(), 2);
  for (int i = 0; i < x.size(); ++i) {
    sn_out.row(i) = cosy_->convert_to_sn_global_2d(x(i), y(i), z(i), z_range).transpose();
  }
  return sn_out;
}
Eigen::MatrixX2d Track::project_2d_point_on_track_window(
  Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<const Eigen::VectorXd> y,
  Eigen::Ref<const Eigen::VectorXd> z, Eigen::Ref<const Eigen::VectorXd> s_start,
  Eigen::Ref<const Eigen::VectorXd> s_end, const double z_range) const
{
  Eigen::MatrixX2d sn_out(x.rows(), 2);
  for (int i = 0; i < x.size(); ++i) {
    sn_out.row(i) =
      cosy_->convert_to_sn_window_2d(x(i), y(i), z(i), s_start(i), s_end(i), z_range).transpose();
  }
  return sn_out;
}
Eigen::MatrixX2d Track::project_2d_point_on_track(
  Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<const Eigen::VectorXd> y) const
{
  Eigen::MatrixX2d sn_out(x.rows(), 2);
  for (int i = 0; i < x.size(); ++i) {
    sn_out.row(i) = cosy_->convert_to_sn(x(i), y(i)).transpose();
  }
  return sn_out;
}
Eigen::Vector3d Track::get_3d_from_2d_global(
  const double x, const double y, const double z, const double z_range) const
{
  auto sn = project_2d_point_on_track_global(x, y, z, z_range);

  return sn2cartesian(sn.x(), sn.y());
}
Eigen::Vector3d Track::get_3d_from_2d_window(
  const double x, const double y, const double z, const double s_start, const double s_end,
  const double z_range) const
{
  auto sn = project_2d_point_on_track_window(x, y, z, s_start, s_end, z_range);

  return sn2cartesian(sn.x(), sn.y());
}
Eigen::Vector3d Track::get_3d_from_2d(const double x, const double y) const
{
  auto sn = project_2d_point_on_track(x, y);

  return sn2cartesian(sn.x(), sn.y());
}
std::tuple<int, double> Track::get_sector(const double s, const int num_sectors) const
{
  // Return if Num segments is less than 1
  if (num_sectors <= 0) {
    return std::make_tuple(-1, -1.0);
  }

  // Base caluclations
  double s_max = data_->data.at(TrackData::s)(Eigen::last);
  double s_curr = s_mod(s, s_max);
  double sector_length = s_max / num_sectors;

  // Sector calculations
  double sector_num = std::floor(s_curr / sector_length);
  double remainder = s_curr - (sector_num * sector_length);

  return std::make_tuple(static_cast<int>(sector_num), remainder);
}
Track::SBasedMargins Track::s_based_limits(
  std::string path, std::size_t index_length, float rate) const
{
  std::string keyword = "s_based_limits";
  bool is_s_based_limits = false;
  bool keyword_found = false;

  std::vector<std::vector<std::string>> s_based_limits_comment =
    tam::helpers::files::get_csv_comments(path, keyword);
  if (s_based_limits_comment.empty()) {
    std::cout << "[track_handler_cpp]: No s-based limits found in file, defaulting to 0.0."
              << std::endl;
    is_s_based_limits = false;
  } else {
    std::string keyword_line = s_based_limits_comment[0][0];
    std::string keyword_value = keyword_line.substr(keyword_line.find(":") + 2);
    std::transform(keyword_value.begin(), keyword_value.end(), keyword_value.begin(), ::tolower);
    is_s_based_limits = (keyword_value == "true");
  }

  // divide segements into s-based margins
  int counter = 0;
  std::vector<float> limits;
  if (is_s_based_limits) {
    for (auto & line : s_based_limits_comment) {
      for (auto & value : line) {
        if (value.find("segment_") != std::string::npos) {
          keyword_found = true;
          if (counter != 0 && counter != 5) {  // make sure segment size has not changed
            throw std::runtime_error(
              "Invalid number of values in segment: " + std::to_string(counter) +
              ". Adapt s-based functions if you want to handle more than 5 values");
          }
          counter = 0;
        } else if (value.find("segment_") == std::string::npos && keyword_found == true) {
          std::string string_value = value.substr(value.find(":") + 2);
          counter += 1;
          try {
            limits.push_back(std::stof(string_value));
          } catch (const std::invalid_argument & e) {
            try {
              limits.push_back(static_cast<float>(std::stoi(string_value)));
            } catch (const std::invalid_argument & e) {
              std::cerr << "Invalid argument: " << e.what() << std::endl;
              limits.clear();
              break;
            }
          }
        }
      }
    }
  }
  double track_length = data_->data.at(TrackData::s)(Eigen::last);
  double stepsize = track_length / index_length;

  std::vector<float> s_based_margin_left = std::vector<float>(index_length, 0.0);
  std::vector<float> s_based_margin_right = std::vector<float>(index_length, 0.0);
  std::vector<float> s_based_v_add = std::vector<float>(index_length, 0.0);

  // create vectors for s-based limits with values from comment block
  for (size_t i = 0; i < limits.size(); i += 5) {
    const float start_zone = limits.at(i);
    const float end_zone = limits.at(i + 1);
    const float distance_left = limits.at(i + 2);
    const float distance_right = limits.at(i + 3);
    const float v_add = limits.at(i + 4);

    std::size_t first_idx = static_cast<std::size_t>(std::floor(start_zone / stepsize));
    std::size_t last_idx = static_cast<std::size_t>(std::ceil(end_zone / stepsize));

    std::size_t start_idx = std::max(0ul, first_idx);
    std::size_t end_idx = std::min(index_length - 1, last_idx);

    for (std::size_t j = start_idx; j <= end_idx; ++j) {
      if (distance_left != 0.0) {
        s_based_margin_left[j] = distance_left;
      }
      if (distance_right != 0.0) {
        s_based_margin_right[j] = distance_right;
      }
      s_based_v_add[j] = v_add;
    }

    // Find begin and end of ramp
    std::size_t ramp_up_idx_left, ramp_down_idx_left, ramp_up_idx_right, ramp_down_idx_right;
    if (rate != 0.0) {
      ramp_up_idx_left = static_cast<std::size_t>(std::clamp<double>(
        start_idx - (std::abs(distance_left) / rate), 0, std::numeric_limits<double>::max()));
      ramp_down_idx_left = static_cast<std::size_t>(std::clamp<double>(
        end_idx + (std::abs(distance_left) / rate), 0, std::numeric_limits<double>::max()));
      ramp_up_idx_right = static_cast<std::size_t>(std::clamp<double>(
        start_idx - (std::abs(distance_right) / rate), 0, std::numeric_limits<double>::max()));
      ramp_down_idx_right = static_cast<std::size_t>(std::clamp<double>(
        end_idx + (std::abs(distance_right) / rate), 0, std::numeric_limits<double>::max()));
      ;
    } else {
      ramp_up_idx_left = start_idx;
      ramp_down_idx_left = end_idx;
      ramp_up_idx_right = start_idx;
      ramp_down_idx_right = end_idx;
    }

    // define ramps for safety margins for smoother transitions
    // left side up and down

    auto calc_s_based_margin = [start_idx, end_idx, rate](
                                 std::vector<float> & s_based_margin, std::size_t ramp_up_idx,
                                 std::size_t ramp_down_idx, float distance) {
      for (std::size_t j = ramp_up_idx, k = end_idx; j < start_idx && k < ramp_down_idx; j++, k++) {
        s_based_margin[j] = (s_based_margin[j] == 0.0 || j == start_idx - 1)
                              ? (rate * (j - ramp_up_idx))
                              : (std::max(s_based_margin[j], rate * (j - ramp_up_idx)));
        s_based_margin[k] = (s_based_margin[j] == 0.0 || k == end_idx)
                              ? (distance - rate * (k - end_idx))
                              : (std::max(s_based_margin[k], distance - rate * (k - end_idx)));
      }
    };

    calc_s_based_margin(s_based_margin_left, ramp_up_idx_left, ramp_down_idx_left, distance_left);
    calc_s_based_margin(
      s_based_margin_right, ramp_up_idx_right, ramp_down_idx_right, distance_right);
  }
  // return s-based limits as vectors around the track
  Track::SBasedMargins limits_vector;
  limits_vector.margin_left = s_based_margin_left;
  limits_vector.margin_right = s_based_margin_right;
  limits_vector.v_add = s_based_v_add;
  return limits_vector;
}
double Track::track_length() const { return data_->data.at(TrackData::s)(Eigen::last); }
float Track::v_max_rl(std::string path) const
{
  std::string keyword = "v_max";
  float v_max;
  std::vector<std::vector<std::string>> v_max_comments =
    tam::helpers::files::get_csv_comments(path, keyword);
  std::string value = v_max_comments[0][0];
  std::string v_max_string = value.substr(value.find(":") + 2);
  try {
    v_max = std::stof(v_max_string);
  } catch (const std::invalid_argument & e) {
    std::cerr << "Invalid argument: " << e.what() << std::endl;
    std::cerr << "v_max is set to default value of 100.0" << std::endl;
    v_max = 100.0;
  }
  return v_max;
}
}  // namespace tam::common
