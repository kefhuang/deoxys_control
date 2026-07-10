#pragma once

#include <utility>

#include <franka/robot.h>

namespace cartesian_velocity_control_utils {

template <typename Robot, typename Callback>
void RunWithRateLimiting(Robot& robot, Callback&& callback) {
  robot.control(std::forward<Callback>(callback),
                franka::ControllerMode::kJointImpedance,
                true);
}

}  // namespace cartesian_velocity_control_utils
