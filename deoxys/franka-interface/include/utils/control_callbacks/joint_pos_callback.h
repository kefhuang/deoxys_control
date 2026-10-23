// Copyright 2022 Yifeng Zhu

#include <cstddef>
#include <franka/model.h>
#include <franka/robot.h>

#include "utils/control_utils.h"
#include "utils/joint_position_stop_state.h"
#include "utils/robot_utils.h"
#include "utils/shared_memory.h"
#include "utils/shared_state.h"
#include "utils/traj_interpolators/streaming_joint_position_traj_interpolator.h"

#ifndef UTILS_CONTROL_CALLBACKS_JOINT_POS_CALLBACK_H_
#define UTILS_CONTROL_CALLBACKS_JOINT_POS_CALLBACK_H_

namespace control_callbacks {
std::function<franka::JointPositions(const franka::RobotState &,
                                     franka::Duration)>
CreateJointPositionCallback(
    const std::shared_ptr<SharedMemory> &global_handler,
    const std::shared_ptr<robot_utils::StatePublisher> state_publisher,
    const franka::Model &model, std::shared_ptr<StateInfo> &current_state_info,
    const int policy_rate, const int traj_rate) {
  const std::shared_ptr<traj_utils::StreamingJointPositionTrajInterpolator>
      interpolator = std::static_pointer_cast<
          traj_utils::StreamingJointPositionTrajInterpolator>(
          global_handler->traj_interpolator_ptr);
  const franka::Model *model_ptr = &model;
  double elapsed = 0.0;
  bool has_target = false;
  control_utils::JointPositionStopState stop_state;

  return [global_handler, state_publisher, model_ptr, current_state_info,
          interpolator, policy_rate, traj_rate, elapsed, has_target,
          stop_state](const franka::RobotState &robot_state,
                      franka::Duration period) mutable -> franka::JointPositions {
    const double callback_period = period.toSec();
    elapsed += callback_period;
    current_state_info->joint_positions =
        Eigen::VectorXd::Map(robot_state.q_d.data(), 7);
    current_state_info->joint_velocities =
        Eigen::VectorXd::Map(robot_state.dq.data(), 7);
    Eigen::Affine3d current_T_EE_in_base_frame(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    current_state_info->pos_EE_in_base_frame
        << current_T_EE_in_base_frame.translation();
    current_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(current_T_EE_in_base_frame.linear());

    const Eigen::Matrix<double, 7, 1> start =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q_d.data());
    const Eigen::Matrix<double, 7, 1> start_velocity =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            robot_state.dq_d.data());
    const Eigen::Matrix<double, 7, 1> start_acceleration =
        Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
            robot_state.ddq_d.data());

    const bool stop_requested =
        global_handler->joint_position_stop_requested.load(
            std::memory_order_acquire) ||
        !global_handler->running.load(std::memory_order_acquire);
    if (stop_requested) {
      if (!stop_state.stopping()) {
        JointPositionSessionState expected =
            JointPositionSessionState::ACTIVE;
        global_handler->joint_position_session.compare_exchange_strong(
            expected, JointPositionSessionState::STOPPING,
            std::memory_order_acq_rel, std::memory_order_acquire);
        stop_state.Begin(robot_state.q_d);
        global_handler->joint_position_commands.DiscardAll();
        const Eigen::Matrix<double, 7, 1> hold =
            Eigen::Map<const Eigen::Matrix<double, 7, 1>>(
                stop_state.hold_positions().data());
        const Eigen::Matrix<double, 7, 1> zero_velocity =
            Eigen::Matrix<double, 7, 1>::Zero();
        interpolator->ResetForNextStep(
            elapsed, callback_period, start, start_velocity,
            start_acceleration, hold, zero_velocity, policy_rate, traj_rate,
            1.0);
      }

      Eigen::Matrix<double, 7, 1> desired_q;
      interpolator->GetNextStep(start, start_velocity, start_acceleration,
                                desired_q);
      std::array<double, 7> joint_positions;
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(joint_positions.data()) =
          desired_q;
      const bool stopped =
          stop_state.Update(robot_state.q_d, robot_state.dq_d,
                            robot_state.ddq_d, robot_state.dq);
      state_publisher->UpdateNewState(robot_state, model_ptr);
      franka::JointPositions output(joint_positions);
      if (stopped) {
        global_handler->joint_position_session.store(
            JointPositionSessionState::STOPPED, std::memory_order_release);
        global_handler->running.store(false, std::memory_order_release);
        if (global_handler->joint_position_termination_requested.load(
                std::memory_order_acquire)) {
          global_handler->termination.store(true, std::memory_order_release);
        }
        return franka::MotionFinished(output);
      }
      return output;
    }

    control_utils::JointPositionCommand command;
    if (global_handler->joint_position_commands.PopLatest(command)) {
      Eigen::Matrix<double, 7, 1> goal;
      Eigen::Matrix<double, 7, 1> velocity_reference;
      for (std::size_t i = 0; i < command.positions.size(); ++i) {
        goal[static_cast<Eigen::Index>(i)] = command.positions[i];
        velocity_reference[static_cast<Eigen::Index>(i)] =
            command.velocities[i];
      }
      if (command.is_delta) {
        goal += start;
      }
      // Retarget the stateful tracker without resetting the desired
      // derivatives that libfranka executed during the preceding interval.
      interpolator->ResetForNextStep(
          elapsed, callback_period, start, start_velocity,
          start_acceleration, goal, velocity_reference, policy_rate, traj_rate,
          command.time_fraction);
      has_target = true;
    }

    Eigen::Matrix<double, 7, 1> desired_q;
    if (has_target) {
      interpolator->GetNextStep(start, start_velocity, start_acceleration,
                                desired_q);
    } else {
      desired_q =
          Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q_d.data());
    }

    state_publisher->UpdateNewState(robot_state, model_ptr);

    std::array<double, 7> joint_positions;
    Eigen::Map<Eigen::Matrix<double, 7, 1>>(joint_positions.data()) = desired_q;
    // Rate limiting is deliberately applied once by libfranka's control loop.
    // Do not rate-limit here and then pass the result through libfranka's
    // default low-pass filter: the filter changes q_d behind this callback's
    // limiter and creates a ~0.003 rad/s limit cycle.
    return franka::JointPositions(joint_positions);
  };
}

} // NAMESPACE control_callbacks

#endif // UTILS_CONTROL_CALLBACKS_JOINT_POS_CALLBACK_H_
