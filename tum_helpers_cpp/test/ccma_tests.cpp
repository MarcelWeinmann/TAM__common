// Copyright 2025 Maximilian Leitenstern
#include <chrono>
#include <iostream>
#include <random>

#include "matplotlib_cpp/matplotlibcpp.hpp"
#include "tum_helpers_cpp/ccma.hpp"

namespace plt = matplotlibcpp;
int main(int argc, char * argv[])
{
  // Check the number of arguments
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << "<w_ma> <w_cc> <distrib>" << std::endl;
    return 1;
  }

  // Generate some points
  int n = 100;
  float sigma = 0.1;
  std::vector<float> x = tam::helpers::linspace<float>(0.0f, 2.0f * M_PI, n);
  std::vector<float> y = tam::helpers::linspace<float>(0.0f, 4.0f * M_PI, n);
  std::vector<float> z = tam::helpers::linspace<float>(0.0f, 2.0f, n);
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<float> dist{0.0f, sigma};

  std::vector<Eigen::Vector3f> points{};

  for (size_t i = 0; i < x.size(); ++i) {
    // Add noise to the points
    float noise_x = dist(gen);
    float noise_y = dist(gen);
    float noise_z = dist(gen);
    points.push_back(
      Eigen::Vector3f(2.0f * std::sin(x[i]) + noise_x, std::sin(y[i]) + noise_y, z[i] + noise_z));
  }

  // Create a CCMA object with default parameters
  tam::helpers::CCMA ccma(std::stoi(argv[1]), std::stoi(argv[2]), argv[3]);

  std::vector<Eigen::Vector3f> points_ccma = ccma.filter(points, "padding", true);
  std::cout << "Filtered points " << points_ccma.size() << std::endl;

  std::vector<float> x_raw, y_raw, x_ccma, y_ccma;
  for (size_t i = 0; i < points.size(); ++i) {
    x_raw.push_back(points[i].x());
    y_raw.push_back(points[i].y());
    x_ccma.push_back(points_ccma[i].x());
    y_ccma.push_back(points_ccma[i].y());
  }

  plt::figure();
  plt::plot(x_raw, y_raw, "r-", {{"label", "original"}});
  plt::plot(x_ccma, y_ccma, "b-", {{"label", "filtered"}});
  plt::legend();
  plt::xlabel("x");
  plt::ylabel("y");
  plt::show();

  return 0;
}
