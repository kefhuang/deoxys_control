#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "utils/traj_interpolators/cosine_cartesian_velocity_traj_interpolator.h"

namespace {
constexpr double kSamplePeriod = 0.001;
constexpr double kGoal = 0.025033843499841165;

void Reset(traj_utils::CosineCartesianVelocityTrajInterpolator& interpolator,
           const Eigen::Vector3d& start,
           const Eigen::Vector3d& goal,
           double time = 0.0) {
  interpolator.Reset(time, start, Eigen::Vector3d::Zero(), goal,
                     Eigen::Vector3d::Zero(), 20, 500, 0.3);
}
}  // namespace

int main() {
  traj_utils::CosineCartesianVelocityTrajInterpolator interpolator;
  Reset(interpolator, Eigen::Vector3d::Zero(),
        Eigen::Vector3d(kGoal, 0.0, 0.0));

  std::vector<double> samples;
  for (int i = 0; i <= 16; ++i) {
    Eigen::Vector3d translation;
    Eigen::Vector3d rotation;
    interpolator.GetNextStep(i * kSamplePeriod, translation, rotation);
    if (!translation.allFinite() || !rotation.allFinite()) {
      std::cerr << "interpolator produced non-finite output" << std::endl;
      return 1;
    }
    samples.push_back(translation.x());
  }
  for (int i = 1; i <= 15; ++i) {
    if (!(samples[i] > samples[i - 1])) {
      std::cerr << "interpolator repeated an active 1 ms sample" << std::endl;
      return 1;
    }
  }

  double max_jerk = 0.0;
  for (size_t i = 2; i < samples.size(); ++i) {
    const double previous_acceleration =
        (samples[i - 1] - samples[i - 2]) / kSamplePeriod;
    const double acceleration =
        (samples[i] - samples[i - 1]) / kSamplePeriod;
    max_jerk = std::max(
        max_jerk,
        std::abs(acceleration - previous_acceleration) / kSamplePeriod);
  }
  if (max_jerk >= 600.0) {
    std::cerr << "jerk bound exceeded: " << max_jerk << std::endl;
    return 1;
  }

  traj_utils::CosineCartesianVelocityTrajInterpolator retargeted;
  Reset(retargeted, Eigen::Vector3d::Zero(),
        Eigen::Vector3d(kGoal, 0.0, 0.0));
  Eigen::Vector3d before;
  Eigen::Vector3d rotation;
  for (int i = 0; i <= 5; ++i) {
    retargeted.GetNextStep(i * kSamplePeriod, before, rotation);
  }
  Reset(retargeted, Eigen::Vector3d::Constant(0.5),
        Eigen::Vector3d(-0.01, 0.0, 0.0), 0.005);
  Eigen::Vector3d after;
  retargeted.GetNextStep(0.005, after, rotation);
  if (!after.isApprox(before, 1e-12)) {
    std::cerr << "retarget introduced a velocity jump" << std::endl;
    return 1;
  }

  traj_utils::CosineCartesianVelocityTrajInterpolator concurrent;
  Reset(concurrent, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  std::atomic_bool start{false};
  std::thread writer([&]() {
    while (!start) {
      std::this_thread::yield();
    }
    for (int i = 0; i < 2000; ++i) {
      const double goal = 0.02 * std::sin(i * 0.01);
      Reset(concurrent, Eigen::Vector3d::Zero(),
            Eigen::Vector3d(goal, 0.0, 0.0), i * kSamplePeriod);
    }
  });
  start = true;
  for (int i = 0; i < 4000; ++i) {
    Eigen::Vector3d translation;
    concurrent.GetNextStep(i * kSamplePeriod, translation, rotation);
    if (!translation.allFinite()) {
      std::cerr << "concurrent access produced non-finite output" << std::endl;
      writer.join();
      return 1;
    }
  }
  writer.join();

  return 0;
}
