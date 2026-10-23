#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_JOINT_POSITION_STOP_STATE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_JOINT_POSITION_STOP_STATE_H_

#include <array>
#include <cmath>
#include <cstddef>

namespace control_utils {

// Repeatedly command one immutable desired position while libfranka's motion
// generator decelerates. MotionFinished is safe only after desired and measured
// motion have remained settled for a short consecutive window.
class JointPositionStopState {
public:
  void Begin(const std::array<double, 7> &desired_positions) {
    hold_positions_ = desired_positions;
    settled_cycles_ = 0;
    stopping_ = true;
  }

  bool stopping() const { return stopping_; }

  const std::array<double, 7> &hold_positions() const {
    return hold_positions_;
  }

  bool Update(const std::array<double, 7> &desired_positions,
              const std::array<double, 7> &desired_velocities,
              const std::array<double, 7> &desired_accelerations,
              const std::array<double, 7> &measured_velocities) {
    bool settled = true;
    for (std::size_t i = 0; i < hold_positions_.size(); ++i) {
      settled = settled &&
                std::abs(desired_positions[i] - hold_positions_[i]) <=
                    PositionTolerance() &&
                std::abs(desired_velocities[i]) <= DesiredVelocityTolerance() &&
                std::abs(desired_accelerations[i]) <=
                    DesiredAccelerationTolerance() &&
                std::abs(measured_velocities[i]) <= MeasuredVelocityTolerance();
    }

    settled_cycles_ = settled ? settled_cycles_ + 1 : 0;
    return settled_cycles_ >= RequiredSettledCycles();
  }

  static std::size_t RequiredSettledCycles() { return 50; }
  // FCI rejects MotionFinished while its desired motion generator state is
  // still moving. These are numerical-zero checks, not ordinary tracking
  // tolerances; measured velocity has a separate sensor-noise allowance.
  static double PositionTolerance() { return 1e-6; }
  static double DesiredVelocityTolerance() { return 1e-6; }
  static double DesiredAccelerationTolerance() { return 1e-4; }
  static double MeasuredVelocityTolerance() { return 1e-2; }

private:
  std::array<double, 7> hold_positions_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0}};
  std::size_t settled_cycles_{0};
  bool stopping_{false};
};

} // namespace control_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_JOINT_POSITION_STOP_STATE_H_
