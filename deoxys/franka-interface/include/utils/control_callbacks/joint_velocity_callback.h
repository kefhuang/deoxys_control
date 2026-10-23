#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <franka/control_types.h>
#include <franka/robot.h>
#include "utils/shared_memory.h"
#include "utils/shared_state.h"
#include "utils/robot_utils.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_CONTROL_CALLBACKS_JOINT_VELOCITY_CALLBACK_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_CONTROL_CALLBACKS_JOINT_VELOCITY_CALLBACK_H_
namespace control_callbacks {
inline std::function<franka::JointVelocities(const franka::RobotState &, franka::Duration)>
CreateJointVelocityCallback(const std::shared_ptr<SharedMemory> &handler,
                            const std::shared_ptr<robot_utils::StatePublisher> &state_publisher,
                            const franka::Model &model,
                            std::shared_ptr<StateInfo> &current,
                            std::shared_ptr<StateInfo> &goal) {
  // Conservative software caps. libfranka applies its own motion-generator
  // limits as a second line of defence.
  constexpr std::array<double, 7> kMaxVelocity{{2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5}};
  constexpr std::array<double, 7> kMaxAcceleration{{5.0, 5.0, 5.0, 5.0, 8.0, 8.0, 8.0}};
  return [handler, state_publisher, model_ptr = &model, current, kMaxVelocity, kMaxAcceleration,
          previous = std::array<double, 7>{}]
      (const franka::RobotState &state, franka::Duration period) mutable {
    current->joint_positions = Eigen::VectorXd::Map(state.q.data(), 7);
    current->joint_velocities = Eigen::VectorXd::Map(state.dq.data(), 7);
    state_publisher->UpdateNewState(state, model_ptr);
    const double dt = std::max(period.toSec(), 1e-6);
    std::array<double, 7> command{};
    for (size_t i = 0; i < 7; ++i) {
      const double requested = handler->joint_velocity_target[i].load(
          std::memory_order_relaxed);
      double target = handler->running && std::isfinite(requested)
                          ? requested : 0.0;
      target = std::max(-kMaxVelocity[i], std::min(target, kMaxVelocity[i]));
      const double raw_delta = target - previous[i];
      const double max_delta = kMaxAcceleration[i] * dt;
      const double delta = std::max(-max_delta, std::min(raw_delta, max_delta));
      command[i] = previous[i] + delta;
    }
    previous = command;
    if (!handler->running) {
      bool stopped = true;
      for (double velocity : command) stopped &= std::abs(velocity) < 1e-4;
      if (stopped) {
        return franka::MotionFinished(franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
      }
    }
    return franka::JointVelocities(command);
  };
}
}
#endif
