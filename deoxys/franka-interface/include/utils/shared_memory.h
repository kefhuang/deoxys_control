// Copyright 2022 Yifeng Zhu

#include <atomic>
#include <array>
#include <memory>
#include <spdlog/spdlog.h>

#include "controllers/base_controller.h"
#include "utils/control_utils.h"
#include "utils/joint_position_command_mailbox.h"
#include "utils/traj_interpolators/base_traj_interpolator.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_MEMORY_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_MEMORY_H_

enum class JointPositionSessionState {
  IDLE,
  STARTING,
  READY,
  ACTIVE,
  STOPPING,
  STOPPED,
};

struct SharedMemory {
  std::atomic_bool running{false};     // controlling control callback
  std::atomic_bool termination{false}; // controlling main loop
  double time = 0.0;

  std::shared_ptr<controller::BaseController> controller_ptr;
  std::shared_ptr<traj_utils::BaseTrajInterpolator> traj_interpolator_ptr;
  std::shared_ptr<spdlog::logger> logger;

  // for torque control
  std::atomic<double> max_torque;
  std::atomic<double> min_torque;

  // for velocity control
  std::atomic<double> max_trans_speed;
  std::atomic<double> min_trans_speed;
  std::atomic<double> max_rot_speed;
  std::atomic<double> min_rot_speed;
  // Lock-free mailbox from the policy thread to the 1 kHz libfranka callback.
  // StateInfo is also used by legacy controllers and is not thread-safe.
  std::array<std::atomic<double>, 7> joint_velocity_target{};

  // A complete joint-position target crosses the policy/real-time boundary as
  // one SPSC queue entry. The real-time callback is the sole owner of the
  // active joint-position interpolator.
  control_utils::JointPositionCommandMailbox<16> joint_position_commands;
  std::atomic<JointPositionSessionState> joint_position_session{
      JointPositionSessionState::IDLE};
  std::atomic_bool joint_position_stop_requested{false};
  std::atomic_bool joint_position_termination_requested{false};

  // for trajectory interpolation
  double traj_interpolator_time_fraction = 1.0;

  std::atomic_int no_msg_counter;
  std::atomic_bool start{
      false}; // indicate if a message is received and start controlling
};

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_MEMORY_H_
