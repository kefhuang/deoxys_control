#include <iostream>
#include <utility>

#include <franka/robot.h>

#include "utils/cartesian_velocity_control_utils.h"

struct FakeRobot {
  franka::ControllerMode mode = franka::ControllerMode::kCartesianImpedance;
  bool limit_rate = false;
  bool callback_forwarded = false;

  template <typename Callback>
  void control(Callback&& callback,
               franka::ControllerMode requested_mode,
               bool requested_limit_rate) {
    mode = requested_mode;
    limit_rate = requested_limit_rate;
    std::forward<Callback>(callback)();
  }
};

int main() {
  FakeRobot robot;
  cartesian_velocity_control_utils::RunWithRateLimiting(
      robot, [&robot]() { robot.callback_forwarded = true; });

  if (!robot.callback_forwarded) {
    std::cerr << "callback was not forwarded" << std::endl;
    return 1;
  }
  if (robot.mode != franka::ControllerMode::kJointImpedance) {
    std::cerr << "controller mode changed" << std::endl;
    return 1;
  }
  if (!robot.limit_rate) {
    std::cerr << "libfranka rate limiting was not enabled" << std::endl;
    return 1;
  }

  return 0;
}
