// Copyright 2023 Simon Hoffmann
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

enum class TrackReferenceLines {
  RACELINE,
  CENTERLINE,
};
struct RacelineData
{
  enum Key { s, v, n, chi, ax, ay, jx, jy, NUM_KEYS };
  std::array<Eigen::VectorXd, Key::NUM_KEYS> data;
};
struct TrackData
{
  enum Key {
    s,
    x,
    y,
    z,
    theta,
    mu,
    phi,
    dtheta,
    dmu,
    dphi,
    omega_x,
    omega_y,
    omega_z,
    domega_x,
    domega_y,
    domega_z,
    tb_left_x,
    tb_left_y,
    tb_left_z,
    tb_right_x,
    tb_right_y,
    tb_right_z,
    w_right,
    w_left,
    w_right_s_based,
    w_left_s_based,
    w_right_ssa,
    w_left_ssa,
    w_right_ssa_s_based,
    w_left_ssa_s_based,
    w_right_sda,
    w_left_sda,
    w_right_sda_s_based,
    w_left_sda_s_based,
    theta_sin,
    theta_cos,
    mu_sin,
    mu_cos,
    phi_sin,
    phi_cos,
    tangent_x,
    tangent_y,
    tangent_z,
    normal_x,
    normal_y,
    normal_z,
    binormal_x,
    binormal_y,
    binormal_z,
    NUM_KEYS
  };
  std::array<Eigen::VectorXd, Key::NUM_KEYS> data;
};