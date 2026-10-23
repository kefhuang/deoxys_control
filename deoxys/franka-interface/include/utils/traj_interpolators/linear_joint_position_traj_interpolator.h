// Copyright 2022 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <algorithm>
#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_LINEAR_JOINT_POSITION_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_LINEAR_JOINT_POSITION_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class LinearJointPositionTrajInterpolator : public BaseTrajInterpolator {
private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  Vector7d q_start_;
  Vector7d q_goal_;
  double max_time_;
  double start_time_;

public:
  inline LinearJointPositionTrajInterpolator()
      : max_time_(1.), start_time_(0.){};

  inline ~LinearJointPositionTrajInterpolator(){};

  inline void Reset(const double &time_sec,
                    const Eigen::Matrix<double, 7, 1> &q_start,
                    const Eigen::Matrix<double, 7, 1> &q_goal,
                    const int &policy_rate, const int &rate,
                    const double &traj_interpolator_time_fraction) {
    (void)rate;

    max_time_ =
        1. / static_cast<double>(policy_rate) * traj_interpolator_time_fraction;
    start_time_ = time_sec;

    // Commands are streamed at policy_rate.  Starting a new segment at the
    // previous *goal* assumes that every old segment was fully executed.  That
    // is false whenever libfranka's rate limiter (or scheduling jitter) makes
    // the robot lag behind, and causes the interpolator to continually command
    // points disconnected from the currently executed trajectory.  The
    // callback supplies the latest desired joint state as q_start, so always
    // start the replacement segment there.
    q_start_ = q_start;
    q_goal_ = q_goal;
  };

  inline void ResetForNextStep(
      const double &callback_time_sec, const double &callback_period_sec,
      const Eigen::Matrix<double, 7, 1> &q_start,
      const Eigen::Matrix<double, 7, 1> &q_goal, const int &policy_rate,
      const int &rate, const double &traj_interpolator_time_fraction) {
    // q_start was sent during the preceding callback. The value returned by
    // this callback is the next discrete command, so it must be the first
    // non-zero-time point of the replacement segment.
    Reset(callback_time_sec - callback_period_sec, q_start, q_goal,
          policy_rate, rate, traj_interpolator_time_fraction);
  }

  inline void GetNextStep(const double &time_sec, Vector7d &q_t) {
    // The libfranka callback runs at 1 kHz. Compute a fresh point on every
    // callback instead of holding each point until the independently configured
    // trajectory rate elapses; that 500 Hz sample-and-hold introduced avoidable
    // velocity and acceleration steps in a 1 kHz position stream.
    const double t =
        std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
    q_t = q_start_ + t * (q_goal_ - q_start_);
  };
};
} // namespace traj_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_LINEAR_JOINT_POSITION_TRAJ_INTERPOLATOR_H_
