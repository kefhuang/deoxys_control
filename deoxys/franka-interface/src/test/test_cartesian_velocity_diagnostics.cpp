#include <array>
#include <iostream>

#include <franka/robot_state.h>

#include "utils/cartesian_velocity_diagnostics.h"

int main() {
  cartesian_velocity_diagnostics::Buffer<3> buffer;

  for (int i = 0; i < 5; ++i) {
    franka::RobotState state;
    state.q.fill(static_cast<double>(i));
    state.dq.fill(static_cast<double>(i) + 0.1);
    state.q_d.fill(static_cast<double>(i) + 0.2);
    state.dq_d.fill(static_cast<double>(i) + 0.3);
    state.ddq_d.fill(static_cast<double>(i) + 0.4);
    state.O_dP_EE_c.fill(static_cast<double>(i) + 0.5);
    state.O_ddP_EE_c.fill(static_cast<double>(i) + 0.6);
    state.O_dP_EE_d.fill(static_cast<double>(i) + 0.65);
    state.elbow.fill(static_cast<double>(i) + 0.7);
    state.elbow_d.fill(static_cast<double>(i) + 0.8);
    state.elbow_c.fill(static_cast<double>(i) + 0.9);
    state.delbow_c.fill(static_cast<double>(i) + 1.0);
    state.ddelbow_c.fill(static_cast<double>(i) + 1.1);
    std::array<double, 6> raw_command;
    raw_command.fill(static_cast<double>(i) + 1.2);
    buffer.Record(i * 0.001, state, raw_command);
  }

  const auto samples = buffer.Snapshot();
  if (samples.size() != 3) {
    std::cerr << "ring buffer size is wrong" << std::endl;
    return 1;
  }
  for (size_t i = 0; i < samples.size(); ++i) {
    const double marker = static_cast<double>(i + 2);
    if (samples[i].time != marker * 0.001 ||
        samples[i].q[0] != marker ||
        samples[i].raw_command[0] != marker + 1.2 ||
        samples[i].O_dP_EE_c[0] != marker + 0.5 ||
        samples[i].O_dP_EE_d[0] != marker + 0.65 ||
        samples[i].elbow_c[0] != marker + 0.9) {
      std::cerr << "ring buffer chronological order or fields are wrong"
                << std::endl;
      return 1;
    }
  }

  buffer.Clear();
  if (!buffer.Snapshot().empty() || buffer.IsFrozen()) {
    std::cerr << "clear did not empty the ring buffer" << std::endl;
    return 1;
  }

  cartesian_velocity_diagnostics::Buffer<10> frozen;
  franka::RobotState stopped_state;
  stopped_state.O_dP_EE_c.fill(0.0);
  std::array<double, 6> moving_command{};
  moving_command[0] = 0.02;
  for (int i = 0; i < 5; ++i) {
    stopped_state.q.fill(static_cast<double>(i));
    frozen.Record(i * 0.001, stopped_state, moving_command);
  }
  if (!frozen.IsFrozen()) {
    std::cerr << "command/state mismatch did not freeze diagnostics"
              << std::endl;
    return 1;
  }
  stopped_state.q.fill(99.0);
  frozen.Record(0.006, stopped_state, moving_command);
  const auto frozen_samples = frozen.Snapshot();
  if (frozen_samples.size() != 5 || frozen_samples.back().q[0] != 4.0) {
    std::cerr << "frozen diagnostics were overwritten" << std::endl;
    return 1;
  }

  return 0;
}
