#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <franka/robot_state.h>

namespace cartesian_velocity_diagnostics {

struct Sample {
  double time = 0.0;
  std::array<double, 6> raw_command{};
  std::array<double, 6> O_dP_EE_c{};   // NOLINT
  std::array<double, 6> O_ddP_EE_c{};  // NOLINT
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> q_d{};
  std::array<double, 7> dq_d{};
  std::array<double, 7> ddq_d{};
  std::array<double, 2> elbow{};
  std::array<double, 2> elbow_d{};
  std::array<double, 2> elbow_c{};
  std::array<double, 2> delbow_c{};
  std::array<double, 2> ddelbow_c{};
};

template <std::size_t Capacity = 100>
class Buffer {
  static_assert(Capacity > 0, "diagnostic buffer capacity must be positive");

 public:
  void Record(double time,
              const franka::RobotState& state,
              const std::array<double, 6>& raw_command) {
    Sample& sample = samples_[write_index_];
    sample.time = time;
    sample.raw_command = raw_command;
    sample.O_dP_EE_c = state.O_dP_EE_c;
    sample.O_ddP_EE_c = state.O_ddP_EE_c;
    sample.q = state.q;
    sample.dq = state.dq;
    sample.q_d = state.q_d;
    sample.dq_d = state.dq_d;
    sample.ddq_d = state.ddq_d;
    sample.elbow = state.elbow;
    sample.elbow_d = state.elbow_d;
    sample.elbow_c = state.elbow_c;
    sample.delbow_c = state.delbow_c;
    sample.ddelbow_c = state.ddelbow_c;

    write_index_ = (write_index_ + 1) % Capacity;
    if (sample_count_ < Capacity) {
      ++sample_count_;
    }
  }

  std::vector<Sample> Snapshot() const {
    std::vector<Sample> result;
    result.reserve(sample_count_);
    const std::size_t oldest =
        (write_index_ + Capacity - sample_count_) % Capacity;
    for (std::size_t i = 0; i < sample_count_; ++i) {
      result.push_back(samples_[(oldest + i) % Capacity]);
    }
    return result;
  }

  void Clear() {
    write_index_ = 0;
    sample_count_ = 0;
  }

 private:
  std::array<Sample, Capacity> samples_{};
  std::size_t write_index_ = 0;
  std::size_t sample_count_ = 0;
};

}  // namespace cartesian_velocity_diagnostics
