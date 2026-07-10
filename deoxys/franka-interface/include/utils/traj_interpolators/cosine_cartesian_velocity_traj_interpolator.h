// Copyright 2023 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <mutex>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_COSINE_CARTESIAN_VELOCITY_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_COSINE_CARTESIAN_VELOCITY_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class CosineCartesianVelocityTrajInterpolator : public BaseTrajInterpolator {
private:
  struct PendingCommand {
    Eigen::Vector3d twist_trans_start = Eigen::Vector3d::Zero();
    Eigen::Vector3d twist_rot_start = Eigen::Vector3d::Zero();
    Eigen::Vector3d twist_trans_goal = Eigen::Vector3d::Zero();
    Eigen::Vector3d twist_rot_goal = Eigen::Vector3d::Zero();
    double duration = 1.0;
    std::uint64_t sequence = 0;
  };

  std::mutex pending_mutex_;
  PendingCommand pending_command_;

  Eigen::Vector3d active_twist_trans_start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d active_twist_trans_goal_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_twist_trans_t_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d active_twist_rot_start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d active_twist_rot_goal_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_twist_rot_t_ = Eigen::Vector3d::Zero();
  double active_start_time_ = 0.0;
  double active_duration_ = 1.0;
  std::uint64_t active_sequence_ = 0;
  bool initialized_ = false;

public:
  inline CosineCartesianVelocityTrajInterpolator() = default;

  inline ~CosineCartesianVelocityTrajInterpolator(){};

  inline void Reset(const double &time_sec,
                    const Eigen::Vector3d &twist_trans_start,
                    const Eigen::Vector3d &twist_rot_start,
                    const Eigen::Vector3d &twist_trans_goal,
                    const Eigen::Vector3d &twist_rot_goal,
                    const int &policy_rate, const int &rate,
                    const double &traj_interpolator_time_fraction) {
    (void)time_sec;
    (void)rate;
    const double duration =
        1. / static_cast<double>(std::max(policy_rate, 1)) *
        traj_interpolator_time_fraction;

    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_command_.twist_trans_start = twist_trans_start;
    pending_command_.twist_rot_start = twist_rot_start;
    pending_command_.twist_trans_goal = twist_trans_goal;
    pending_command_.twist_rot_goal = twist_rot_goal;
    pending_command_.duration =
        std::max(duration, std::numeric_limits<double>::epsilon());
    ++pending_command_.sequence;
  };

  inline void GetNextStep(const double &time_sec, Eigen::Vector3d &twist_trans_t, Eigen::Vector3d &twist_rot_t) {
    PendingCommand pending;
    bool has_new_command = false;
    {
      std::unique_lock<std::mutex> lock(pending_mutex_, std::try_to_lock);
      if (lock.owns_lock() &&
          pending_command_.sequence != active_sequence_) {
        pending = pending_command_;
        has_new_command = true;
      }
    }

    if (has_new_command) {
      if (!initialized_) {
        last_twist_trans_t_ = pending.twist_trans_start;
        last_twist_rot_t_ = pending.twist_rot_start;
        initialized_ = true;
      }
      active_twist_trans_start_ = last_twist_trans_t_;
      active_twist_rot_start_ = last_twist_rot_t_;
      active_twist_trans_goal_ = pending.twist_trans_goal;
      active_twist_rot_goal_ = pending.twist_rot_goal;
      active_start_time_ = time_sec;
      active_duration_ = pending.duration;
      active_sequence_ = pending.sequence;
    }

    if (initialized_) {
      const double normalized_time = std::min(
          std::max((time_sec - active_start_time_) / active_duration_, 0.0),
          1.0);
      const double cosine_profile =
          0.5 * (1.0 - std::cos(M_PI * normalized_time));
      last_twist_trans_t_ =
          active_twist_trans_start_ +
          cosine_profile *
              (active_twist_trans_goal_ - active_twist_trans_start_);
      last_twist_rot_t_ =
          active_twist_rot_start_ +
          cosine_profile *
              (active_twist_rot_goal_ - active_twist_rot_start_);
    }

    twist_trans_t = last_twist_trans_t_;
    twist_rot_t = last_twist_rot_t_;
  };
};
} // namespace traj_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_NULL_VELOCITY_TRAJ_INTERPOLATOR_H_
