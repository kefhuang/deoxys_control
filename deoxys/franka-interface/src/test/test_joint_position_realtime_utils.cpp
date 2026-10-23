#include <array>
#include <atomic>
#include <cmath>
#include <iostream>
#include <thread>

#include <Eigen/Dense>
#include <franka/rate_limiting.h>

#include "utils/joint_position_command_mailbox.h"
#include "utils/joint_position_stop_state.h"
#include "utils/traj_interpolators/linear_joint_position_traj_interpolator.h"
#include "utils/traj_interpolators/streaming_joint_position_traj_interpolator.h"

namespace {

bool TestMailboxLatestAndConsistency() {
  control_utils::JointPositionCommandMailbox<8> mailbox;
  if (!mailbox.IsLockFree()) {
    std::cerr << "mailbox atomics are not lock-free" << std::endl;
    return false;
  }
  for (int marker = 1; marker <= 3; ++marker) {
    control_utils::JointPositionCommand command;
    command.positions.fill(static_cast<double>(marker));
    command.velocities.fill(-static_cast<double>(marker));
    if (!mailbox.Push(command)) {
      std::cerr << "mailbox unexpectedly filled" << std::endl;
      return false;
    }
  }

  control_utils::JointPositionCommand latest;
  if (!mailbox.PopLatest(latest) || latest.positions[0] != 3.0 ||
      latest.velocities[0] != -3.0 ||
      latest.sequence != 3 || mailbox.PopLatest(latest)) {
    std::cerr << "mailbox did not return only the latest complete command"
              << std::endl;
    return false;
  }

  constexpr int kCommandCount = 20000;
  std::atomic_bool producer_finished{false};
  std::atomic_bool inconsistent{false};
  double last_marker = 0.0;
  std::uint64_t last_sequence = latest.sequence;
  std::thread producer([&]() {
    for (int marker = 1; marker <= kCommandCount; ++marker) {
      control_utils::JointPositionCommand command;
      command.positions.fill(static_cast<double>(marker));
      command.velocities.fill(-static_cast<double>(marker));
      while (!mailbox.Push(command)) {
        std::this_thread::yield();
      }
    }
    producer_finished.store(true, std::memory_order_release);
  });

  while (!producer_finished.load(std::memory_order_acquire)) {
    if (!mailbox.PopLatest(latest)) {
      std::this_thread::yield();
      continue;
    }
    for (const double value : latest.positions) {
      if (value != latest.positions[0]) {
        inconsistent.store(true, std::memory_order_relaxed);
      }
    }
    for (const double value : latest.velocities) {
      if (value != -latest.positions[0]) {
        inconsistent.store(true, std::memory_order_relaxed);
      }
    }
    if (latest.sequence <= last_sequence ||
        latest.positions[0] <= last_marker) {
      inconsistent.store(true, std::memory_order_relaxed);
    }
    last_sequence = latest.sequence;
    last_marker = latest.positions[0];
  }
  while (mailbox.PopLatest(latest)) {
    for (const double value : latest.positions) {
      if (value != latest.positions[0]) {
        inconsistent.store(true, std::memory_order_relaxed);
      }
    }
    for (const double value : latest.velocities) {
      if (value != -latest.positions[0]) {
        inconsistent.store(true, std::memory_order_relaxed);
      }
    }
    if (latest.sequence <= last_sequence ||
        latest.positions[0] <= last_marker) {
      inconsistent.store(true, std::memory_order_relaxed);
    }
    last_sequence = latest.sequence;
    last_marker = latest.positions[0];
  }
  producer.join();
  if (inconsistent.load(std::memory_order_relaxed)) {
    std::cerr << "mailbox exposed a partially written target" << std::endl;
    return false;
  }
  if (last_marker != static_cast<double>(kCommandCount)) {
    std::cerr << "mailbox did not deliver the final published target"
              << std::endl;
    return false;
  }

  control_utils::JointPositionCommandMailbox<4> full_mailbox;
  control_utils::JointPositionCommand command;
  if (!full_mailbox.Push(command) || !full_mailbox.Push(command) ||
      !full_mailbox.Push(command) || full_mailbox.Push(command) ||
      full_mailbox.dropped() != 1) {
    std::cerr << "mailbox full/drop behavior is incorrect" << std::endl;
    return false;
  }
  return true;
}

bool TestLinearInterpolatorUpdatesEveryMillisecond() {
  traj_utils::LinearJointPositionTrajInterpolator interpolator;
  Eigen::Matrix<double, 7, 1> start =
      Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> goal = start;
  goal[0] = 0.05;
  interpolator.Reset(0.0, start, goal, 20, 500, 1.0);

  double previous = -1.0;
  Eigen::Matrix<double, 7, 1> sample;
  for (int millisecond = 0; millisecond <= 50; ++millisecond) {
    interpolator.GetNextStep(millisecond * 0.001, sample);
    if (!sample.allFinite() ||
        (millisecond > 0 && !(sample[0] > previous))) {
      std::cerr << "linear interpolator repeated or corrupted a 1 ms sample"
                << std::endl;
      return false;
    }
    previous = sample[0];
  }

  const Eigen::Matrix<double, 7, 1> replacement_start = sample;
  goal[0] = 0.10;
  interpolator.Reset(0.050, replacement_start, goal, 20, 500, 1.0);
  interpolator.GetNextStep(0.050, sample);
  if (!sample.isApprox(replacement_start, 1e-12)) {
    std::cerr << "retarget changed position at the segment boundary"
              << std::endl;
    return false;
  }
  interpolator.GetNextStep(0.051, sample);
  if (!(sample[0] > replacement_start[0])) {
    std::cerr << "replacement segment did not advance after 1 ms" << std::endl;
    return false;
  }
  return true;
}

bool TestMovingReferenceCrossesWaypointsWithoutStopping() {
  constexpr double kReferenceVelocity = 0.2;
  constexpr int kPolicySteps = 50;
  traj_utils::StreamingJointPositionTrajInterpolator interpolator;
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> ddq{};
  Eigen::Matrix<double, 7, 1> q_reference =
      Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> dq_reference =
      Eigen::Matrix<double, 7, 1>::Zero();
  dq_reference[0] = kReferenceVelocity;

  for (int step = 0; step < 2000; ++step) {
    const Eigen::Matrix<double, 7, 1> q_d =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q.data());
    const Eigen::Matrix<double, 7, 1> dq_d =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(dq.data());
    const Eigen::Matrix<double, 7, 1> ddq_d =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(ddq.data());
    if (step % kPolicySteps == 0) {
      q_reference[0] = kReferenceVelocity * step * franka::kDeltaT;
      interpolator.ResetForNextStep(
          (step + 1) * franka::kDeltaT, franka::kDeltaT, q_d, dq_d,
          ddq_d, q_reference, dq_reference, 20, 500, 1.0);
    }

    Eigen::Matrix<double, 7, 1> interpolated;
    interpolator.GetNextStep(q_d, dq_d, ddq_d, interpolated);
    std::array<double, 7> raw{};
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(raw.data()) = interpolated;
    const std::array<double, 7> next = franka::limitRate(
        franka::kMaxJointVelocity, franka::kMaxJointAcceleration,
        franka::kMaxJointJerk, raw, q, dq, ddq);
    for (std::size_t joint = 0; joint < next.size(); ++joint) {
      const double next_velocity = (next[joint] - q[joint]) / franka::kDeltaT;
      ddq[joint] = (next_velocity - dq[joint]) / franka::kDeltaT;
      dq[joint] = next_velocity;
    }
    q = next;
  }

  if (std::abs(dq[0] - kReferenceVelocity) > 0.005 ||
      std::abs(q[0] - 0.4) > 0.005 || interpolator.reference_expired()) {
    std::cerr << "moving reference stopped at streamed waypoints: q=" << q[0]
              << " dq=" << dq[0]
              << " expired=" << interpolator.reference_expired()
              << std::endl;
    return false;
  }

  // No further policy input: finite extrapolation must end and the same
  // tracker must safely decelerate rather than running away indefinitely.
  for (int step = 0; step < 1000; ++step) {
    const Eigen::Matrix<double, 7, 1> q_d =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q.data());
    const Eigen::Matrix<double, 7, 1> dq_d =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(dq.data());
    const Eigen::Matrix<double, 7, 1> ddq_d =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(ddq.data());
    Eigen::Matrix<double, 7, 1> interpolated;
    interpolator.GetNextStep(q_d, dq_d, ddq_d, interpolated);
    std::array<double, 7> next{};
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(next.data()) = interpolated;
    for (std::size_t joint = 0; joint < next.size(); ++joint) {
      const double next_velocity = (next[joint] - q[joint]) / franka::kDeltaT;
      ddq[joint] = (next_velocity - dq[joint]) / franka::kDeltaT;
      dq[joint] = next_velocity;
    }
    q = next;
  }
  if (!interpolator.reference_expired() || std::abs(dq[0]) > 1e-3 ||
      q[0] > 0.45) {
    std::cerr << "expired moving reference did not brake: q=" << q[0]
              << " dq=" << dq[0] << std::endl;
    return false;
  }
  return true;
}

bool TestStreamingRetargetKeepsDesiredDerivativesStable() {
  constexpr double kAmplitude = 0.15;
  constexpr double kDuration = 6.0;
  constexpr std::array<int, 8> kPolicyIntervals{
      {50, 49, 50, 51, 50, 67, 33, 50}};
  constexpr double kPi = 3.14159265358979323846;

  traj_utils::StreamingJointPositionTrajInterpolator interpolator;
  std::array<double, 7> desired_positions{};
  std::array<double, 7> desired_velocities{};
  std::array<double, 7> desired_accelerations{};
  Eigen::Matrix<double, 7, 1> goal =
      Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> velocity_reference =
      Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> interpolated = goal;

  double max_velocity = 0.0;
  double max_acceleration = 0.0;
  double max_position = 0.0;
  double max_velocity_error = 0.0;
  double max_policy_acceleration = 0.0;
  double max_limiter_correction = 0.0;
  double previous_policy_velocity = 0.0;
  double previous_policy_time = 0.0;
  bool has_previous_policy_velocity = false;
  std::size_t wrong_direction_samples = 0;
  std::size_t acceleration_limit_samples = 0;
  std::size_t velocity_direction_flips = 0;
  int previous_velocity_direction = 0;
  int next_policy_step = 0;
  std::size_t policy_interval_index = 0;

  for (int step = 0; step < static_cast<int>(kDuration / franka::kDeltaT);
       ++step) {
    const double target_time = step * franka::kDeltaT;
    const double callback_time = target_time + franka::kDeltaT;
    const bool received_target = step == next_policy_step;
    if (received_target) {
      const Eigen::Matrix<double, 7, 1> start =
          Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
              desired_positions.data());
      const Eigen::Matrix<double, 7, 1> start_velocity =
          Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
              desired_velocities.data());
      const Eigen::Matrix<double, 7, 1> start_acceleration =
          Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
              desired_accelerations.data());
      goal[0] = 0.5 * kAmplitude *
                (1.0 - std::cos(2.0 * kPi * target_time / kDuration));
      velocity_reference[0] =
          kPi * kAmplitude / kDuration *
          std::sin(2.0 * kPi * target_time / kDuration);

      // This mirrors the callback: q_d is from the preceding interval, while
      // the returned command must be the first 1 ms point of the new segment.
      interpolator.ResetForNextStep(
          callback_time, franka::kDeltaT, start, start_velocity,
          start_acceleration, goal, velocity_reference, 20, 500, 1.0);
      next_policy_step +=
          kPolicyIntervals[policy_interval_index % kPolicyIntervals.size()];
      ++policy_interval_index;
    }

    const Eigen::Matrix<double, 7, 1> current_position =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            desired_positions.data());
    const Eigen::Matrix<double, 7, 1> current_velocity =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            desired_velocities.data());
    const Eigen::Matrix<double, 7, 1> current_acceleration =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            desired_accelerations.data());
    interpolator.GetNextStep(current_position, current_velocity,
                             current_acceleration, interpolated);
    std::array<double, 7> raw_positions{};
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(raw_positions.data()) =
        interpolated;
    const std::array<double, 7> next_positions = franka::limitRate(
        franka::kMaxJointVelocity, franka::kMaxJointAcceleration,
        franka::kMaxJointJerk, raw_positions, desired_positions,
        desired_velocities, desired_accelerations);
    for (std::size_t joint = 0; joint < next_positions.size(); ++joint) {
      max_limiter_correction =
          std::max(max_limiter_correction,
                   std::abs(next_positions[joint] - raw_positions[joint]));
    }

    std::array<double, 7> next_velocities{};
    std::array<double, 7> next_accelerations{};
    for (std::size_t joint = 0; joint < next_positions.size(); ++joint) {
      next_velocities[joint] =
          (next_positions[joint] - desired_positions[joint]) /
          franka::kDeltaT;
      next_accelerations[joint] =
          (next_velocities[joint] - desired_velocities[joint]) /
          franka::kDeltaT;
    }
    desired_positions = next_positions;
    desired_velocities = next_velocities;
    desired_accelerations = next_accelerations;

    max_velocity =
        std::max(max_velocity, std::abs(desired_velocities[0]));
    max_acceleration =
        std::max(max_acceleration, std::abs(desired_accelerations[0]));
    max_position = std::max(max_position, desired_positions[0]);
    if (std::abs(desired_accelerations[0]) > 9.9) {
      ++acceleration_limit_samples;
    }
    const int velocity_direction = desired_velocities[0] > 0.01
                                       ? 1
                                       : (desired_velocities[0] < -0.01 ? -1
                                                                        : 0);
    if (velocity_direction != 0) {
      if (previous_velocity_direction != 0 &&
          velocity_direction != previous_velocity_direction) {
        ++velocity_direction_flips;
      }
      previous_velocity_direction = velocity_direction;
    }

    if (received_target) {
      const double expected_velocity =
          kPi * kAmplitude / kDuration *
          std::sin(2.0 * kPi * target_time / kDuration);
      max_velocity_error =
          std::max(max_velocity_error,
                   std::abs(desired_velocities[0] - expected_velocity));
      if (has_previous_policy_velocity) {
        max_policy_acceleration = std::max(
            max_policy_acceleration,
            std::abs(desired_velocities[0] - previous_policy_velocity) /
                (target_time - previous_policy_time));
      }
      previous_policy_velocity = desired_velocities[0];
      previous_policy_time = target_time;
      has_previous_policy_velocity = true;
      if (std::abs(expected_velocity) > 0.02 &&
          expected_velocity * desired_velocities[0] < 0.0) {
        ++wrong_direction_samples;
      }
    }
  }

  if (max_velocity > 0.10 || max_acceleration > 0.35 ||
      max_position < 0.12 || max_position > 0.16 ||
      std::abs(desired_positions[0]) > 0.01 ||
      max_velocity_error > 0.025 || max_policy_acceleration > 0.6 ||
      wrong_direction_samples != 0 || acceleration_limit_samples != 0 ||
      velocity_direction_flips > 1 || max_limiter_correction > 1e-12) {
    std::cerr << "streaming retarget destabilized desired derivatives: "
              << "max_velocity=" << max_velocity
              << " max_acceleration=" << max_acceleration
              << " max_position=" << max_position
              << " final_position=" << desired_positions[0]
              << " max_velocity_error=" << max_velocity_error
              << " max_policy_acceleration=" << max_policy_acceleration
              << " max_limiter_correction=" << max_limiter_correction
              << " wrong_direction=" << wrong_direction_samples
              << " acceleration_limit_samples="
              << acceleration_limit_samples
              << " velocity_direction_flips=" << velocity_direction_flips
              << std::endl;
    return false;
  }
  return true;
}

bool TestStreamingStepAllowsSafeLag() {
  traj_utils::StreamingJointPositionTrajInterpolator interpolator;
  std::array<double, 7> desired_positions{};
  std::array<double, 7> desired_velocities{};
  std::array<double, 7> desired_accelerations{};
  Eigen::Matrix<double, 7, 1> goal =
      Eigen::Matrix<double, 7, 1>::Zero();
  goal[0] = 0.15;
  const Eigen::Matrix<double, 7, 1> zeros =
      Eigen::Matrix<double, 7, 1>::Zero();
  interpolator.ResetForNextStep(franka::kDeltaT, franka::kDeltaT, zeros,
                                zeros, zeros, goal, zeros, 20, 500, 1.0);

  double position_at_policy_period = 0.0;
  double max_position = 0.0;
  double max_acceleration = 0.0;
  double max_limiter_correction = 0.0;
  std::size_t velocity_direction_flips = 0;
  int previous_velocity_direction = 0;
  for (int step = 0; step < 2000; ++step) {
    const Eigen::Matrix<double, 7, 1> current_position =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            desired_positions.data());
    const Eigen::Matrix<double, 7, 1> current_velocity =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            desired_velocities.data());
    const Eigen::Matrix<double, 7, 1> current_acceleration =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            desired_accelerations.data());
    Eigen::Matrix<double, 7, 1> interpolated;
    interpolator.GetNextStep(current_position, current_velocity,
                             current_acceleration, interpolated);
    std::array<double, 7> raw_positions{};
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(raw_positions.data()) =
        interpolated;
    const std::array<double, 7> next_positions = franka::limitRate(
        franka::kMaxJointVelocity, franka::kMaxJointAcceleration,
        franka::kMaxJointJerk, raw_positions, desired_positions,
        desired_velocities, desired_accelerations);

    std::array<double, 7> next_velocities{};
    std::array<double, 7> next_accelerations{};
    for (std::size_t joint = 0; joint < next_positions.size(); ++joint) {
      max_limiter_correction =
          std::max(max_limiter_correction,
                   std::abs(next_positions[joint] - raw_positions[joint]));
      next_velocities[joint] =
          (next_positions[joint] - desired_positions[joint]) /
          franka::kDeltaT;
      next_accelerations[joint] =
          (next_velocities[joint] - desired_velocities[joint]) /
          franka::kDeltaT;
    }
    desired_positions = next_positions;
    desired_velocities = next_velocities;
    desired_accelerations = next_accelerations;
    max_acceleration =
        std::max(max_acceleration, std::abs(desired_accelerations[0]));
    max_position = std::max(max_position, desired_positions[0]);
    const int velocity_direction = desired_velocities[0] > 0.01
                                       ? 1
                                       : (desired_velocities[0] < -0.01 ? -1
                                                                        : 0);
    if (velocity_direction != 0) {
      if (previous_velocity_direction != 0 &&
          velocity_direction != previous_velocity_direction) {
        ++velocity_direction_flips;
      }
      previous_velocity_direction = velocity_direction;
    }
    if (step == 49) {
      position_at_policy_period = desired_positions[0];
    }
  }

  if (position_at_policy_period > 0.02 ||
      std::abs(desired_positions[0] - goal[0]) > 3e-4 ||
      max_position > goal[0] + 1e-3 || max_acceleration > 1.001 ||
      max_limiter_correction > 1e-12 || velocity_direction_flips > 1) {
    std::cerr << "step tracker did not preserve safe lag: position_50ms="
              << position_at_policy_period
              << " final_error="
              << std::abs(desired_positions[0] - goal[0])
              << " max_position=" << max_position
              << " max_acceleration=" << max_acceleration
              << " max_limiter_correction=" << max_limiter_correction
              << " velocity_direction_flips=" << velocity_direction_flips
              << std::endl;
    return false;
  }
  return true;
}

bool TestStopRequiresConsecutiveSettledCycles() {
  control_utils::JointPositionStopState stop;
  std::array<double, 7> hold{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  std::array<double, 7> zeros{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  stop.Begin(hold);

  std::array<double, 7> moving = zeros;
  moving[0] = 0.1;
  if (stop.Update(hold, moving, zeros, moving)) {
    std::cerr << "moving state was incorrectly considered stopped" << std::endl;
    return false;
  }

  for (std::size_t cycle = 1;
       cycle < control_utils::JointPositionStopState::RequiredSettledCycles() /
                   2;
       ++cycle) {
    if (stop.Update(hold, zeros, zeros, zeros)) {
      std::cerr << "stop completed before the settled window elapsed"
                << std::endl;
      return false;
    }
  }
  if (stop.Update(hold, moving, zeros, moving)) {
    std::cerr << "motion did not reset the consecutive settled window"
              << std::endl;
    return false;
  }
  for (std::size_t cycle = 1;
       cycle < control_utils::JointPositionStopState::RequiredSettledCycles();
       ++cycle) {
    if (stop.Update(hold, zeros, zeros, zeros)) {
      std::cerr << "stop completed before the restarted settled window elapsed"
                << std::endl;
      return false;
    }
  }
  if (!stop.Update(hold, zeros, zeros, zeros)) {
    std::cerr << "stop did not complete after the settled window" << std::endl;
    return false;
  }
  return true;
}

} // namespace

int main() {
  return TestMailboxLatestAndConsistency() &&
                 TestLinearInterpolatorUpdatesEveryMillisecond() &&
                 TestMovingReferenceCrossesWaypointsWithoutStopping() &&
                 TestStreamingRetargetKeepsDesiredDerivativesStable() &&
                 TestStreamingStepAllowsSafeLag() &&
                 TestStopRequiresConsecutiveSettledCycles()
             ? 0
             : 1;
}
