#include <cmath>
#include <iostream>

#include <yaml-cpp/yaml.h>

#ifndef DEOXYS_CARTESIAN_VELOCITY_CONFIG_PATH
#error "DEOXYS_CARTESIAN_VELOCITY_CONFIG_PATH is required"
#endif

int main() {
  const YAML::Node config =
      YAML::LoadFile(DEOXYS_CARTESIAN_VELOCITY_CONFIG_PATH);
  const double time_fraction =
      config["traj_interpolator_cfg"]["time_fraction"].as<double>();
  if (std::abs(time_fraction - 0.5) > 1e-12) {
    std::cerr << "expected Cartesian velocity time_fraction 0.5, got "
              << time_fraction << std::endl;
    return 1;
  }
  return 0;
}
