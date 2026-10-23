// Copyright 2026

#include "base_traj_interpolator.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <franka/rate_limiting.h>

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_STREAMING_JOINT_POSITION_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_STREAMING_JOINT_POSITION_TRAJ_INTERPOLATOR_H_

namespace traj_utils {

// A 1 kHz C2 motion-reference tracker.  Each policy message supplies a paired
// joint position and velocity reference.  Between policy updates the reference
// is advanced at that velocity, so a moving waypoint is crossed without an
// artificial stop.  This class deliberately has no application motion state:
// STREAMING/BRAKING/HOLDING are Arm semantics; Deoxys only tracks its input.
class StreamingJointPositionTrajInterpolator : public BaseTrajInterpolator {
private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  static constexpr double kMaxTrajectoryAcceleration = 1.0;
  static constexpr double kMaxTrajectoryJerk = 100.0;
  static constexpr double kStoppingAcceleration = 0.25;
  static constexpr double kVelocityLimitFraction = 0.5;
  static constexpr double kReferencePeriodsBeforeBrake = 2.5;

  Vector7d reference_position_;
  Vector7d reference_velocity_;
  double natural_frequency_;
  double reference_age_sec_;
  double reference_horizon_sec_;
  bool reference_expired_;

  static inline double Clamp(const double value, const double lower,
                             const double upper) {
    return std::max(lower, std::min(value, upper));
  }

public:
  inline StreamingJointPositionTrajInterpolator()
      : natural_frequency_(20.0), reference_age_sec_(0.0),
        reference_horizon_sec_(0.125), reference_expired_(false) {
    reference_position_.setZero();
    reference_velocity_.setZero();
  }

  inline ~StreamingJointPositionTrajInterpolator() {}

  inline void ResetForNextStep(
      const double callback_time_sec, const double callback_period_sec,
      const Vector7d &q_start, const Vector7d &dq_start,
      const Vector7d &ddq_start, const Vector7d &q_reference,
      const Vector7d &dq_reference, const int policy_rate, const int rate,
      const double traj_interpolator_time_fraction) {
    (void)callback_time_sec;
    (void)callback_period_sec;
    (void)q_start;
    (void)dq_start;
    (void)ddq_start;
    (void)rate;
    reference_position_ = q_reference;
    reference_velocity_ = dq_reference;
    for (std::size_t joint = 0; joint < 7; ++joint) {
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      const double velocity_limit =
          kVelocityLimitFraction * franka::kMaxJointVelocity[joint];
      reference_velocity_[index] =
          Clamp(reference_velocity_[index], -velocity_limit, velocity_limit);
    }
    const double policy_duration =
        traj_interpolator_time_fraction / static_cast<double>(policy_rate);
    natural_frequency_ =
        1.0 / std::max(policy_duration, franka::kDeltaT);
    reference_horizon_sec_ =
        kReferencePeriodsBeforeBrake *
        std::max(policy_duration, franka::kDeltaT);
    reference_age_sec_ = 0.0;
    reference_expired_ = false;
  }

  inline void GetNextStep(const Vector7d &q_d, const Vector7d &dq_d,
                          const Vector7d &ddq_d, Vector7d &q_t) {
    reference_age_sec_ += franka::kDeltaT;
    if (!reference_expired_ &&
        reference_age_sec_ > reference_horizon_sec_) {
      // Freeze at the last finitely extrapolated reference and brake if the
      // policy stream disappears before the outer watchdog closes control.
      reference_position_ += reference_velocity_ * reference_horizon_sec_;
      reference_velocity_.setZero();
      reference_age_sec_ = 0.0;
      reference_expired_ = true;
    }

    const Vector7d moving_position_reference =
        reference_position_ + reference_velocity_ * reference_age_sec_;
    const double stiffness = natural_frequency_ * natural_frequency_;
    const double damping = 2.0 * natural_frequency_;
    const double max_acceleration_change =
        kMaxTrajectoryJerk * franka::kDeltaT;

    for (std::size_t joint = 0; joint < 7; ++joint) {
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      const double velocity_limit =
          kVelocityLimitFraction * franka::kMaxJointVelocity[joint];
      const double position_error =
          moving_position_reference[index] - q_d[index];
      double requested_acceleration;
      if (reference_velocity_[index] == 0.0) {
        // A zero-velocity reference is a point target.  Use a monotonic
        // stopping envelope so braking does not ring around the hold point.
        const double stopping_speed =
            std::sqrt(2.0 * kStoppingAcceleration *
                      std::abs(position_error));
        const double requested_speed =
            std::min(velocity_limit,
                     std::min(natural_frequency_ * std::abs(position_error),
                              stopping_speed));
        const double requested_velocity =
            position_error >= 0.0 ? requested_speed : -requested_speed;
        requested_acceleration =
            natural_frequency_ * (requested_velocity - dq_d[index]);
      } else {
        const double velocity_error =
            reference_velocity_[index] - dq_d[index];
        requested_acceleration =
            stiffness * position_error + damping * velocity_error;
      }
      const double acceleration_target =
          Clamp(requested_acceleration, -kMaxTrajectoryAcceleration,
                kMaxTrajectoryAcceleration);
      const double next_acceleration =
          Clamp(acceleration_target,
                ddq_d[index] - max_acceleration_change,
                ddq_d[index] + max_acceleration_change);
      const double next_velocity =
          Clamp(dq_d[index] + next_acceleration * franka::kDeltaT,
                -velocity_limit, velocity_limit);
      q_t[index] = q_d[index] + next_velocity * franka::kDeltaT;

      if (reference_velocity_[index] == 0.0 &&
          std::abs(position_error) <= 1e-9 &&
          std::abs(dq_d[index]) <= 1e-7 &&
          std::abs(ddq_d[index]) <= 1e-5) {
        q_t[index] = reference_position_[index];
      }
    }
  }

  inline bool reference_expired() const { return reference_expired_; }
};

} // namespace traj_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_STREAMING_JOINT_POSITION_TRAJ_INTERPOLATOR_H_
