#include <Eigen/Dense>
#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_VELOCITY_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_VELOCITY_H_
namespace controller {
class JointVelocityController : public BaseController {
 protected:
  FrankaJointVelocityControllerMessage control_msg_;
 public:
  JointVelocityController() = default;
  explicit JointVelocityController(franka::Model &model) { model_ = &model; }
  bool ParseMessage(const FrankaControlMessage &msg) override;
  void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                   std::shared_ptr<StateInfo> &goal_info) override;
  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Matrix<double, 7, 1> &) override;
};
}
#endif
