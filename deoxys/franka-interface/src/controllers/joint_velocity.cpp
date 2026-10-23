#include "controllers/joint_velocity.h"

namespace controller {
bool JointVelocityController::ParseMessage(const FrankaControlMessage &msg) {
  return msg.control_msg().UnpackTo(&control_msg_);
}

void JointVelocityController::ComputeGoal(
    const std::shared_ptr<StateInfo> &, std::shared_ptr<StateInfo> &goal) {
  goal->joint_velocities << control_msg_.goal().q1(), control_msg_.goal().q2(),
      control_msg_.goal().q3(), control_msg_.goal().q4(),
      control_msg_.goal().q5(), control_msg_.goal().q6(),
      control_msg_.goal().q7();
}

std::array<double, 7> JointVelocityController::Step(
    const franka::RobotState &,
    const Eigen::Matrix<double, 7, 1> &desired_dq) {
  std::array<double, 7> result{};
  Eigen::VectorXd::Map(result.data(), 7) = desired_dq;
  return result;
}
}
