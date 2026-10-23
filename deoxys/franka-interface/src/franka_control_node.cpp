// Copyright 2022 Yifeng Zhu

#include <atomic>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <yaml-cpp/yaml.h>

#include <spdlog/spdlog.h>

#include "utils/common_utils.h"
#include "utils/cartesian_velocity_control_utils.h"
#include "utils/control_exception_utils.h"
#include "utils/control_utils.h"
#include "utils/log_utils.h"
#include "utils/robot_utils.h"
#include "utils/shared_memory.h"
#include "utils/zmq_utils.h"

// Callback
#include "utils/control_callbacks/joint_pos_callback.h"
#include "utils/control_callbacks/joint_velocity_callback.h"
#include "utils/control_callbacks/torque_callback.h"
#include "utils/control_callbacks/cartesian_velocity_calllback.h"

// Interpolators
#include "utils/traj_interpolators/linear_joint_position_traj_interpolator.h"
#include "utils/traj_interpolators/streaming_joint_position_traj_interpolator.h"
#include "utils/traj_interpolators/linear_pose_traj_interpolator.h"
#include "utils/traj_interpolators/linear_position_traj_interpolator.h"
#include "utils/traj_interpolators/min_jerk_joint_position_traj_interpolator.h"
#include "utils/traj_interpolators/min_jerk_pose_traj_interpolator.h"
#include "utils/traj_interpolators/smooth_joint_traj_interpolator.h"
#include "utils/traj_interpolators/cosine_cartesian_velocity_traj_interpolator.h"
#include "utils/traj_interpolators/linear_cartesian_velocity_traj_interpolator.h"

// State estimators
#include "utils/state_estimators/exponential_smoothing_estimator.h"

// Controllers
#include "controllers/joint_impedance.h"
#include "controllers/joint_position.h"
#include "controllers/joint_velocity.h"
#include "controllers/osc_impedance.h"
#include "controllers/osc_position_impedance.h"
#include "controllers/osc_yaw_impedance.h"
#include "controllers/cartesian_velocity.h"

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"

enum ControllerType {
  NO_CONTROL,
  OSC_POSE,
  OSC_POSITION,
  JOINT_POSITION,
  JOINT_IMPEDANCE,
  JOINT_VELOCITY,
  TORQUE,
  OSC_YAW,
  CARTESIAN_VELOCITY,
};

enum TrajInterpolatorType {
  NO_INTERPOLATION,
  LINEAR_POSITION,
  LINEAR_POSE,
  MIN_JERK_POSE,
  SMOOTH_JOINT_POSITION,
  MIN_JERK_JOINT_POSITION,
  LINEAR_JOINT_POSITION,
  COSINE_CARTESIAN_VELOCITY,
  LINEAR_CARTESIAN_VELOCITY,
};

enum StateEstimatorType {
  NO_ESTIMATOR,
  EXPONENTIAL_SMOOTHING_ESTIMATOR,
};

bool GetControllerType(const FrankaControlMessage &franka_control_msg,
                       ControllerType &controller_type) {
  if (franka_control_msg.controller_type() ==
      FrankaControlMessage_ControllerType_OSC_POSE) {
    controller_type = ControllerType::OSC_POSE;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_OSC_POSITION) {
    controller_type = ControllerType::OSC_POSITION;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_OSC_YAW) {
    controller_type = ControllerType::OSC_YAW;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_JOINT_POSITION) {
    controller_type = ControllerType::JOINT_POSITION;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_JOINT_IMPEDANCE) {
    controller_type = ControllerType::JOINT_IMPEDANCE;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_JOINT_VELOCITY) {
    controller_type = ControllerType::JOINT_VELOCITY;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_TORQUE) {
    controller_type = ControllerType::TORQUE;
  } else if (franka_control_msg.controller_type() ==           
             FrankaControlMessage_ControllerType_CARTESIAN_VELOCITY) {
    controller_type = ControllerType::CARTESIAN_VELOCITY;
  } else if (franka_control_msg.controller_type() ==
             FrankaControlMessage_ControllerType_NO_CONTROL) {
    controller_type = ControllerType::NO_CONTROL;
  } else {
    return false;
  }
  return true;
}

bool GetTrajInterpolatorType(const FrankaControlMessage &franka_control_msg,
                             TrajInterpolatorType &traj_interpolator_type) {
  if (franka_control_msg.traj_interpolator_type() ==
      FrankaControlMessage_TrajInterpolatorType_LINEAR_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_POSITION;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_LINEAR_POSE) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_POSE;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_MIN_JERK_POSE) {
    traj_interpolator_type = TrajInterpolatorType::MIN_JERK_POSE;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_SMOOTH_JOINT_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::SMOOTH_JOINT_POSITION;
  } else if (
      franka_control_msg.traj_interpolator_type() ==
      FrankaControlMessage_TrajInterpolatorType_MIN_JERK_JOINT_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::MIN_JERK_JOINT_POSITION;
  } else if (franka_control_msg.traj_interpolator_type() ==
             FrankaControlMessage_TrajInterpolatorType_LINEAR_JOINT_POSITION) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_JOINT_POSITION;
  } else if (franka_control_msg.traj_interpolator_type() == FrankaControlMessage_TrajInterpolatorType_COSINE_CARTESIAN_VELOCITY) {
    traj_interpolator_type = TrajInterpolatorType::COSINE_CARTESIAN_VELOCITY;
  } else if (franka_control_msg.traj_interpolator_type() == FrankaControlMessage_TrajInterpolatorType_LINEAR_CARTESIAN_VELOCITY) {
    traj_interpolator_type = TrajInterpolatorType::LINEAR_CARTESIAN_VELOCITY;
  }
  else {
    traj_interpolator_type = TrajInterpolatorType::NO_INTERPOLATION;
    return false;
  }
  return true;
}

bool GetStateEstimatorType(const FrankaControlMessage franka_control_msg,
                           StateEstimatorType &state_estimator_type) {
  if (franka_control_msg.state_estimator_msg().estimator_type() ==
      FrankaStateEstimatorMessage_EstimatorType_EXPONENTIAL_SMOOTHING_ESTIMATOR) {
    state_estimator_type = StateEstimatorType::EXPONENTIAL_SMOOTHING_ESTIMATOR;
  } else {
    state_estimator_type = StateEstimatorType::NO_ESTIMATOR;
    return false;
  }
  return true;
}

bool ParseJointPositionCommand(
    const FrankaControlMessage &control_msg, const double time_fraction,
    control_utils::JointPositionCommand &command) {
  FrankaJointPositionControllerMessage joint_position_msg;
  if (!control_msg.control_msg().UnpackTo(&joint_position_msg)) {
    return false;
  }

  const auto &goal = joint_position_msg.goal();
  command.positions = {{goal.q1(), goal.q2(), goal.q3(), goal.q4(), goal.q5(),
                        goal.q6(), goal.q7()}};
  command.velocities.fill(0.0);
  if (joint_position_msg.dq_reference_size() != 0 &&
      joint_position_msg.dq_reference_size() != 7) {
    return false;
  }
  for (int index = 0; index < joint_position_msg.dq_reference_size();
       ++index) {
    command.velocities[static_cast<std::size_t>(index)] =
        joint_position_msg.dq_reference(index);
  }
  command.is_delta = goal.is_delta();
  command.time_fraction = time_fraction;
  for (const double position : command.positions) {
    if (!std::isfinite(position)) {
      return false;
    }
  }
  for (const double velocity : command.velocities) {
    if (!std::isfinite(velocity)) {
      return false;
    }
  }
  return std::isfinite(command.time_fraction) && command.time_fraction > 0.0;
}

bool IsJointPositionInterpolator(const TrajInterpolatorType type) {
  return type == TrajInterpolatorType::LINEAR_JOINT_POSITION;
}

int main(int argc, char **argv) {
  // Load cofigs
  if (argc < 2) {
    spdlog::error("It seems that you forgot to specify a yaml config file");
    return 0;
  }

  YAML::Node config = YAML::LoadFile(argv[1]);

  // Initialize zmq sub / pub
  const std::string robot_ip = config["ROBOT"]["IP"].as<std::string>();

  // Subscribing control command
  const std::string subscriber_ip = config["PC"]["IP"].as<std::string>();
  const std::string sub_port = config["NUC"]["SUB_PORT"].as<std::string>();

  // Publishing control command
  const std::string pub_port = config["NUC"]["PUB_PORT"].as<std::string>();

  // zmq_utils::ZMQPublisher zmq_pub(pub_port);
  zmq_utils::ZMQSubscriber zmq_sub(subscriber_ip, sub_port);

  YAML::Node control_config;
  if (argc > 2) {
    control_config = YAML::LoadFile(argv[2]);
  } else {
    control_config = YAML::LoadFile("config/control_config.yml");
  }

  int loaded_state_pub_rate, loaded_policy_rate, loaded_traj_rate;
  bool zmq_noblock;

  if (config["CONTROL"]["STATE_PUBLISHER_RATE"]) {
    loaded_state_pub_rate = config["CONTROL"]["STATE_PUBLISHER_RATE"].as<int>();
  } else {
    loaded_state_pub_rate = 100;
  }
  if (config["CONTROL"]["POLICY_RATE"]) {
    loaded_policy_rate = config["CONTROL"]["POLICY_RATE"].as<int>();
  } else {
    loaded_policy_rate = 20;
  }
  if (config["CONTROL"]["TRAJ_RATE"]) {
    loaded_traj_rate = config["CONTROL"]["TRAJ_RATE"].as<int>();
  } else {
    loaded_traj_rate = 500;
  }
  if (config["CONTROL"]["ZMQ_NOBLOCK"]) {
    zmq_noblock = config["CONTROL"]["ZMQ_NOBLOCK"].as<bool>();
  } else {
    zmq_noblock = true;
  }
  const int state_pub_rate = loaded_state_pub_rate;
  const int policy_rate = loaded_policy_rate;
  const int traj_rate = loaded_traj_rate;

  // Initialize robot
  log_utils::initialize_logger(
      config["ARM_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>(),
      config["ARM_LOGGER"]["CONSOLE"]["LEVEL"].as<std::string>(),
      config["ARM_LOGGER"]["CONSOLE"]["USE"].as<bool>(),
      config["ARM_LOGGER"]["FILE"]["LOGGER_NAME"].as<std::string>(),
      config["ARM_LOGGER"]["FILE"]["LEVEL"].as<std::string>(),
      config["ARM_LOGGER"]["FILE"]["USE"].as<bool>());

  try {

    franka::Robot robot(robot_ip);
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

    // TODO(Yifeng): Read this config from yaml file
    robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    // Get initial state
    franka::RobotState init_state = robot.readOnce();
    Eigen::Affine3d init_T_EE_in_base_frame(
        Eigen::Matrix4d::Map(init_state.O_T_EE.data()));

    std::shared_ptr<SharedMemory> global_handler =
        std::make_shared<SharedMemory>();
    global_handler->logger = log_utils::get_logger(
        config["ARM_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());
    global_handler->logger->info(
        "Joint-position streaming revision: moving-reference-tracker-v4");
    if (!global_handler->joint_position_commands.IsLockFree()) {
      global_handler->logger->error(
          "Joint-position mailbox atomics are not lock-free on this platform");
      return -1;
    }
    
    // Read torque limits from the global config
    global_handler->max_torque =
        control_config["CONTROL"]["SAFETY"]["MAX_TORQUE"].as<double>();
    global_handler->min_torque =
        control_config["CONTROL"]["SAFETY"]["MIN_TORQUE"].as<double>();

    // Read speed limits from the global config
    global_handler->max_trans_speed =
        control_config["CONTROL"]["SAFETY"]["MAX_TRANS_SPEED"].as<double>();
    global_handler->min_trans_speed =
        control_config["CONTROL"]["SAFETY"]["MIN_TRANS_SPEED"].as<double>();
    global_handler->max_rot_speed =
        control_config["CONTROL"]["SAFETY"]["MAX_ROT_SPEED"].as<double>();
    global_handler->min_rot_speed =
        control_config["CONTROL"]["SAFETY"]["MIN_ROT_SPEED"].as<double>();

    std::shared_ptr<StateInfo> current_state_info =
        std::make_shared<StateInfo>();
    std::shared_ptr<StateInfo> goal_state_info = std::make_shared<StateInfo>();

    current_state_info->pos_EE_in_base_frame
        << init_T_EE_in_base_frame.translation();
    current_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(init_T_EE_in_base_frame.linear());
    goal_state_info->pos_EE_in_base_frame =
        current_state_info->pos_EE_in_base_frame;
    goal_state_info->quat_EE_in_base_frame =
        current_state_info->quat_EE_in_base_frame;
    current_state_info->joint_positions =
        Eigen::VectorXd::Map(init_state.q_d.data(), 7);
    goal_state_info->joint_positions = current_state_info->joint_positions;
    current_state_info->joint_velocities = Eigen::VectorXd::Map(init_state.dq.data(), 7);
    goal_state_info->joint_velocities.setZero();
    for (auto &target : global_handler->joint_velocity_target) target.store(0.0);

    // Log information about current arm control frequency
    global_handler->logger->info(
        "State Publisher: {0}Hz, Policy: {1}Hz, Traj Interpolation {2}Hz, ZMQ "
        "noblock receving {3}",
        state_pub_rate, policy_rate, traj_rate, zmq_noblock);
    if (!zmq_noblock) {
      global_handler->logger->warn(
          "ZMQ communication is blocking, it could lead to severe networking "
          "issue. Recommend to set true.");
    }

    struct {
      std::mutex mutex;
      FrankaControlMessage control_msg;
      ControllerType controller_type = ControllerType::NO_CONTROL;
      TrajInterpolatorType traj_interpolator_type =
          TrajInterpolatorType::NO_INTERPOLATION;
      StateEstimatorType state_estimator_type =
          StateEstimatorType::NO_ESTIMATOR;
      double timeout = -1.0; // No timeout if negative
    } control_command{};

    struct {
      std::mutex mutex;
      franka::RobotState state;
    } state_sub{};

    std::shared_ptr<robot_utils::StatePublisher> state_publisher =
        std::make_shared<robot_utils::StatePublisher>(pub_port, state_pub_rate);
    state_publisher->StartPublishing();
    state_publisher->UpdateNewState(init_state, &model);

    ControllerType controller_type = ControllerType::NO_CONTROL;
    std::atomic<ControllerType> active_controller_type(
        ControllerType::NO_CONTROL);
    TrajInterpolatorType traj_interpolator_type =
        TrajInterpolatorType::NO_INTERPOLATION;
    StateEstimatorType state_estimator_type = StateEstimatorType::NO_ESTIMATOR;

    const auto request_joint_position_stop =
        [&](const bool request_termination) {
          if (request_termination) {
            global_handler->joint_position_termination_requested.store(
                true, std::memory_order_release);
            // This stops the outer threads, but it does not interrupt a
            // blocking robot.control() call.  The active callback still sees
            // stop_requested below, holds q_d until settled, and only then
            // returns MotionFinished.
            global_handler->termination.store(true,
                                              std::memory_order_release);
          }

          while (true) {
            JointPositionSessionState state =
                global_handler->joint_position_session.load(
                    std::memory_order_acquire);
            if (state == JointPositionSessionState::ACTIVE) {
              JointPositionSessionState expected =
                  JointPositionSessionState::ACTIVE;
              if (!global_handler->joint_position_session.compare_exchange_weak(
                      expected, JointPositionSessionState::STOPPING,
                      std::memory_order_acq_rel,
                      std::memory_order_acquire)) {
                continue;
              }
              global_handler->joint_position_stop_requested.store(
                  true, std::memory_order_release);
              return;
            }
            if (state == JointPositionSessionState::STOPPING) {
              // STOPPING may concurrently advance to STOPPED and IDLE.  Never
              // write the stale STOPPING value back, or the session can move
              // backwards after its callback has already exited.
              global_handler->joint_position_stop_requested.store(
                  true, std::memory_order_release);
              return;
            }
            if (state == JointPositionSessionState::READY) {
              JointPositionSessionState expected =
                  JointPositionSessionState::READY;
              if (!global_handler->joint_position_session.compare_exchange_weak(
                      expected, JointPositionSessionState::IDLE,
                      std::memory_order_acq_rel,
                      std::memory_order_acquire)) {
                continue;
              }
              global_handler->joint_position_commands.DiscardAll();
              global_handler->running.store(false, std::memory_order_release);
              if (request_termination) {
                global_handler->termination.store(true,
                                                  std::memory_order_release);
              }
              return;
            }
            if (state == JointPositionSessionState::STARTING) {
              JointPositionSessionState expected =
                  JointPositionSessionState::STARTING;
              if (!global_handler->joint_position_session.compare_exchange_weak(
                      expected, JointPositionSessionState::IDLE,
                      std::memory_order_acq_rel,
                      std::memory_order_acquire)) {
                continue;
              }
              global_handler->joint_position_commands.DiscardAll();
            }
            global_handler->running.store(false, std::memory_order_release);
            if (request_termination) {
              global_handler->termination.store(true,
                                                std::memory_order_release);
            }
            return;
          }
        };

    // control message subscription thread
    std::thread control_msg_sub([&]() {
      while (!global_handler->termination) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(int(1. / policy_rate * 1000.)));

        // Receive message
        std::string msg;
        msg = zmq_sub.recv(zmq_noblock);
        if (msg.length() == 0) {
          global_handler->no_msg_counter += int(global_handler->start);
          global_handler->logger->debug("Counter {0}",
                                        global_handler->no_msg_counter);
          if (global_handler->no_msg_counter >= 20) {
            if (global_handler->joint_position_session.load(
                    std::memory_order_acquire) !=
                JointPositionSessionState::IDLE) {
              request_joint_position_stop(true);
            } else {
              global_handler->running = false;
              global_handler->termination = true;
            }
            global_handler->logger->debug(
                "No valid messages received in 20 steps");
          }
          continue;
        }

        FrankaControlMessage control_msg;

        if (control_msg.ParseFromString(msg)) {
          global_handler->no_msg_counter = 0;
          const ControllerType active_controller =
              active_controller_type.load(std::memory_order_acquire);
          if (control_msg.termination()) {
            if (global_handler->joint_position_session.load(
                    std::memory_order_acquire) !=
                JointPositionSessionState::IDLE) {
              request_joint_position_stop(true);
            } else {
              global_handler->running = false;
              global_handler->termination = true;
            }
            if (global_handler->termination.load(std::memory_order_acquire)) {
              continue;
            }
          }
          // Determine controller message type
          {
            // This is the non-real-time ZMQ thread. Waiting briefly for the
            // main loop to finish copying the previous command is preferable
            // to processing the new protobuf with stale controller metadata.
            std::lock_guard<std::mutex> lock(control_command.mutex);
            if (!GetControllerType(control_msg,
                                   control_command.controller_type)) {
              global_handler->logger->warn("No controller type is specified");
            };
            if (!GetTrajInterpolatorType(
                    control_msg, control_command.traj_interpolator_type)) {
              global_handler->logger->debug(
                  "No traj interpolator is specified");
            };
            if (!GetStateEstimatorType(control_msg,
                                       control_command.state_estimator_type)) {
              global_handler->logger->debug("No state estimator is specified");
            }
            if (control_msg.traj_interpolator_time_fraction() > 0.10) {
              global_handler->traj_interpolator_time_fraction =
                  control_msg.traj_interpolator_time_fraction();
            } else {
              // if the time fraction is set to less than 1/10, then we consider
              // it to be too dangerous
              global_handler->traj_interpolator_time_fraction = 0.1;
            }
            control_command.timeout = control_msg.timeout();
            control_command.control_msg = control_msg;
          }

          const JointPositionSessionState joint_position_session =
              global_handler->joint_position_session.load(
                  std::memory_order_acquire);
          if (control_command.controller_type !=
                  ControllerType::JOINT_POSITION &&
              joint_position_session != JointPositionSessionState::IDLE) {
            request_joint_position_stop(control_msg.termination());
            continue;
          }

          if (control_command.controller_type ==
              ControllerType::JOINT_POSITION) {
            if (!IsJointPositionInterpolator(
                    control_command.traj_interpolator_type)) {
              global_handler->logger->error(
                  "Streaming JOINT_POSITION requires "
                  "LINEAR_JOINT_POSITION");
              if (joint_position_session != JointPositionSessionState::IDLE) {
                request_joint_position_stop(false);
              }
              continue;
            }

            control_utils::JointPositionCommand joint_position_command;
            if (!ParseJointPositionCommand(
                    control_msg,
                    global_handler->traj_interpolator_time_fraction,
                    joint_position_command)) {
              global_handler->logger->error(
                  "Rejecting malformed or non-finite joint-position target");
              if (joint_position_session != JointPositionSessionState::IDLE) {
                request_joint_position_stop(false);
              }
              continue;
            }

            JointPositionSessionState state = joint_position_session;
            if (state == JointPositionSessionState::IDLE) {
              if (active_controller != ControllerType::NO_CONTROL) {
                global_handler->logger->error(
                    "Rejecting JOINT_POSITION until the active controller "
                    "finishes");
                continue;
              }
              JointPositionSessionState expected =
                  JointPositionSessionState::IDLE;
              if (!global_handler->joint_position_session.compare_exchange_strong(
                      expected, JointPositionSessionState::STARTING,
                      std::memory_order_acq_rel,
                      std::memory_order_acquire)) {
                state = expected;
              } else {
                global_handler->joint_position_commands.DiscardAll();
                global_handler->joint_position_stop_requested.store(
                    false, std::memory_order_release);
                global_handler->joint_position_termination_requested.store(
                    false, std::memory_order_release);
                global_handler->controller_ptr =
                    std::make_shared<controller::JointPositionController>(model);
                global_handler->traj_interpolator_ptr = std::make_shared<
                    traj_utils::StreamingJointPositionTrajInterpolator>();
                traj_interpolator_type =
                    TrajInterpolatorType::LINEAR_JOINT_POSITION;
                if (!global_handler->joint_position_commands.Push(
                        joint_position_command)) {
                  global_handler->logger->error(
                      "Failed to publish the first joint-position target");
                  global_handler->joint_position_session.store(
                      JointPositionSessionState::IDLE,
                      std::memory_order_release);
                  global_handler->running.store(false,
                                                std::memory_order_release);
                  continue;
                }
                global_handler->start.store(true, std::memory_order_release);
                global_handler->joint_position_session.store(
                    JointPositionSessionState::READY,
                    std::memory_order_release);
                global_handler->running.store(true,
                                              std::memory_order_release);
                global_handler->logger->info(
                    "Joint position session ready with paired position/velocity "
                    "reference tracker (v4)");
                continue;
              }
            }

            if (state == JointPositionSessionState::READY ||
                state == JointPositionSessionState::ACTIVE) {
              if (!global_handler->joint_position_commands.Push(
                      joint_position_command)) {
                global_handler->logger->error(
                    "Joint-position mailbox is full; requesting a safe stop");
                request_joint_position_stop(false);
              }
            }
            // STOPPING/STOPPED sessions deliberately ignore late commands.
            continue;
          }

          if (control_command.controller_type == ControllerType::OSC_POSE &&
              active_controller == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::OSCImpedanceController>(model);
            global_handler->logger->info("Initialize OSC Pose");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::OSC_POSITION &&
                     active_controller == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::OSCPositionImpedanceController>(
                    model);
            global_handler->logger->info("Initialize OSC Position");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::OSC_YAW &&
                     active_controller == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::OSCYawImpedanceController>(model);
            global_handler->logger->info("Initialize OSC Yaw");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::JOINT_IMPEDANCE &&
                     active_controller == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::JointImpedanceController>(model);
            global_handler->logger->info("Initialize Joint Impedance");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::JOINT_VELOCITY &&
                     active_controller == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::JointVelocityController>(model);
            global_handler->logger->info("Initialize Joint Velocity");
            global_handler->running = true;
          } else if (control_command.controller_type ==
                         ControllerType::CARTESIAN_VELOCITY &&
                     active_controller == ControllerType::NO_CONTROL) {
            global_handler->controller_ptr =
                std::make_shared<controller::CartesianVelocityController>(model);
            global_handler->logger->info("Initialize Cartesian Velocity");
            global_handler->running = true;            
          } else if (control_command.controller_type ==
                         ControllerType::NO_CONTROL ||
                     active_controller == ControllerType::NO_CONTROL) {
            global_handler->running = false;
            continue;
          }

          if (control_command.controller_type != ControllerType::NO_CONTROL &&
              active_controller == ControllerType::NO_CONTROL) {
            if (control_command.state_estimator_type ==
                StateEstimatorType::EXPONENTIAL_SMOOTHING_ESTIMATOR) {
              global_handler->controller_ptr->SetStateEstimator(
                  std::make_shared<
                      estimator_utils::ExponentialSmoothingEstimator>());
              global_handler->logger->info("Initialize State Estimator");
            }
            state_estimator_type = control_command.state_estimator_type;
          }

          global_handler->start = true;
          if (traj_interpolator_type !=
              control_command.traj_interpolator_type) {
            if (control_command.traj_interpolator_type ==
                TrajInterpolatorType::LINEAR_POSE) {
              global_handler->traj_interpolator_ptr =
                  std::make_shared<traj_utils::LinearPoseTrajInterpolator>();
              global_handler->logger->info("Initialize Pose interpolator!");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::LINEAR_POSITION) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::LinearPositionTrajInterpolator>();
              global_handler->logger->info("Initialize Position interpolator!");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::MIN_JERK_POSE) {
              global_handler->traj_interpolator_ptr =
                  std::make_shared<traj_utils::MinJerkPoseTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Min Jerk Pose interpolator!");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::SMOOTH_JOINT_POSITION) {
              global_handler->traj_interpolator_ptr =
                  std::make_shared<traj_utils::SmoothJointTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Smooth Joint Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::MIN_JERK_JOINT_POSITION) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::MinJerkJointPositionTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Min Jerk Joint Position Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type ==
                       TrajInterpolatorType::LINEAR_JOINT_POSITION) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::LinearJointPositionTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Linear Joint Position Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type == TrajInterpolatorType::COSINE_CARTESIAN_VELOCITY) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::CosineCartesianVelocityTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Cosine Cartesian Velocity Trajectory Interpolator");
            } else if (control_command.traj_interpolator_type == TrajInterpolatorType::LINEAR_CARTESIAN_VELOCITY) {
              global_handler->traj_interpolator_ptr = std::make_shared<
                  traj_utils::LinearCartesianVelocityTrajInterpolator>();
              global_handler->logger->info(
                  "Initialize Linear Cartesian Velocity Trajectory Interpolator");
            }else {
              global_handler->logger->error("No interpolator is specified");
            }

            traj_interpolator_type = control_command.traj_interpolator_type;
          }

          global_handler->controller_ptr->ParseMessage(control_msg);

          global_handler->controller_ptr->ComputeGoal(current_state_info,
                                                      goal_state_info);
          if (control_command.controller_type == ControllerType::JOINT_VELOCITY) {
            for (size_t i = 0; i < 7; ++i) {
              global_handler->joint_velocity_target[i].store(
                  goal_state_info->joint_velocities[i], std::memory_order_relaxed);
            }
          }
          if (control_command.controller_type == ControllerType::JOINT_VELOCITY &&
              goal_state_info->joint_velocities.cwiseAbs().maxCoeff() > 1e-6) {
            global_handler->logger->debug(
                "Joint velocity target: [{0}, {1}, {2}, {3}, {4}, {5}, {6}]",
                goal_state_info->joint_velocities[0], goal_state_info->joint_velocities[1],
                goal_state_info->joint_velocities[2], goal_state_info->joint_velocities[3],
                goal_state_info->joint_velocities[4], goal_state_info->joint_velocities[5],
                goal_state_info->joint_velocities[6]);
          }
          switch (control_command.traj_interpolator_type) {
          case TrajInterpolatorType::LINEAR_POSE:
          case TrajInterpolatorType::LINEAR_POSITION:
          case TrajInterpolatorType::MIN_JERK_POSE:
            global_handler->traj_interpolator_ptr->Reset(
                global_handler->time, current_state_info->pos_EE_in_base_frame,
                current_state_info->quat_EE_in_base_frame,
                goal_state_info->pos_EE_in_base_frame,
                goal_state_info->quat_EE_in_base_frame, policy_rate, traj_rate,
                global_handler->traj_interpolator_time_fraction);
            break;
          case TrajInterpolatorType::SMOOTH_JOINT_POSITION:
          case TrajInterpolatorType::MIN_JERK_JOINT_POSITION:
          case TrajInterpolatorType::LINEAR_JOINT_POSITION:
            global_handler->traj_interpolator_ptr->Reset(
                global_handler->time, current_state_info->joint_positions,
                goal_state_info->joint_positions, policy_rate, traj_rate,
                global_handler->traj_interpolator_time_fraction);
            break;
          case TrajInterpolatorType::COSINE_CARTESIAN_VELOCITY:
          case TrajInterpolatorType::LINEAR_CARTESIAN_VELOCITY:
            global_handler->traj_interpolator_ptr->Reset(
                global_handler->time, current_state_info->twist_trans_EE_in_base_frame,
                current_state_info->twist_rot_EE_in_base_frame,
                goal_state_info->twist_trans_EE_in_base_frame,
                goal_state_info->twist_rot_EE_in_base_frame, policy_rate, traj_rate,
                global_handler->traj_interpolator_time_fraction);
            break;
          default:
            break;
          }
        } else {
          global_handler->no_msg_counter++;
          global_handler->logger->info("Counter {0}",
                                       global_handler->no_msg_counter);
        }
      }
    });

    // Main loop
    global_handler->logger->info("Deoxys starting");
    bool control_failed = false;
    while (!global_handler->termination) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      // If controller_type changes, exit robot control loop and reinitialize.
      FrankaControlMessage control_msg;

      {
        std::lock_guard<std::mutex> lock(control_command.mutex);
        controller_type = control_command.controller_type;
        control_msg = control_command.control_msg;
      }

      if (global_handler->running) {
        const bool joint_position_control =
            controller_type == ControllerType::JOINT_POSITION;
        if (joint_position_control) {
          JointPositionSessionState expected =
              JointPositionSessionState::READY;
          if (!global_handler->joint_position_session.compare_exchange_strong(
                  expected, JointPositionSessionState::ACTIVE,
                  std::memory_order_acq_rel, std::memory_order_acquire)) {
            continue;
          }
        }
        active_controller_type.store(controller_type,
                                     std::memory_order_release);
        const auto read_result =
            control_exception_utils::RunWithShutdownOnFrankaException(
                "robot.readOnce",
                [&]() {
                  init_state = robot.readOnce();
                  state_publisher->UpdateNewState(init_state, &model);
                },
                global_handler->running,
                global_handler->termination);
        if (!read_result.succeeded) {
          active_controller_type.store(ControllerType::NO_CONTROL,
                                       std::memory_order_release);
          if (joint_position_control) {
            global_handler->joint_position_session.store(
                JointPositionSessionState::IDLE,
                std::memory_order_release);
          }
          global_handler->logger->error("{0} failed: {1}",
                                        read_result.operation_name,
                                        read_result.error_message);
          global_handler->logger->flush();
          control_failed = true;
          continue;
        }
        if (controller_type == ControllerType::NO_CONTROL)
          continue;
        // Choose which control callback functions
        const auto control_result =
            control_exception_utils::RunWithShutdownOnFrankaException(
                "robot.control",
                [&]() {
                  if (controller_type == ControllerType::OSC_POSE ||
                      controller_type == ControllerType::OSC_POSITION ||
                      controller_type == ControllerType::OSC_YAW) {
                    robot.control(
                        control_callbacks::CreateTorqueFromCartesianSpaceCallback(
                            global_handler, state_publisher, model,
                            current_state_info, goal_state_info, policy_rate,
                            traj_rate));
                  } else if (controller_type == ControllerType::JOINT_IMPEDANCE) {
                    global_handler->logger->info("Joint impedance callback");
                    robot.control(
                        control_callbacks::CreateTorqueFromJointSpaceCallback(
                            global_handler, state_publisher, model,
                            current_state_info, goal_state_info, policy_rate,
                            traj_rate));
                  } else if (controller_type == ControllerType::JOINT_POSITION) {
                    global_handler->logger->info("Joint position callback");
                    robot.control(control_callbacks::CreateJointPositionCallback(
                        global_handler, state_publisher, model,
                        current_state_info, policy_rate, traj_rate),
                        franka::ControllerMode::kJointImpedance,
                        true,
                        franka::kMaxCutoffFrequency);
                  } else if (controller_type == ControllerType::JOINT_VELOCITY) {
                    global_handler->logger->info("Joint velocity callback");
                    robot.control(control_callbacks::CreateJointVelocityCallback(
                        global_handler, state_publisher, model,
                        current_state_info, goal_state_info));
                  } else if (controller_type ==
                             ControllerType::CARTESIAN_VELOCITY) {
                    global_handler->logger->info(
                        "Cartesian velocity callback");
                    cartesian_velocity_control_utils::RunWithRateLimiting(
                        robot,
                        control_callbacks::CreateCartesianVelocitiesCallback(
                            global_handler, state_publisher, model,
                            current_state_info, goal_state_info, policy_rate,
                            traj_rate));
                  }
                },
                global_handler->running,
                global_handler->termination);
        active_controller_type.store(ControllerType::NO_CONTROL,
                                     std::memory_order_release);
        if (joint_position_control) {
          global_handler->joint_position_session.store(
              JointPositionSessionState::IDLE,
              std::memory_order_release);
          // The requester promotes termination immediately; this final check
          // also covers a callback that observed the per-session flag first.
          if (global_handler->joint_position_termination_requested.load(
                  std::memory_order_acquire)) {
            global_handler->termination.store(true,
                                              std::memory_order_release);
          }
        }

        if (!control_result.succeeded) {
          global_handler->logger->error("{0} failed: {1}",
                                        control_result.operation_name,
                                        control_result.error_message);
          global_handler->logger->flush();
          control_failed = true;
        }
      }
      global_handler->time = 0.0;
    }
    state_publisher->StopPublishing();

    if (control_msg_sub.joinable()) {
      control_msg_sub.join();
    }
    if (control_failed) {
      return -1;
    }
  } catch (franka::Exception const &e) {
    auto logger = log_utils::get_logger(
        config["ARM_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());
    logger->error(e.what());
    return -1;
  }
  return 0;
}
