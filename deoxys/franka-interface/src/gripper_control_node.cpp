// Copyright 2022 Yifeng Zhu

#include <atomic>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <yaml-cpp/yaml.h>

#include <spdlog/spdlog.h>

#include "utils/log_utils.h"
#include "utils/robot_utils.h"
#include "utils/zmq_utils.h"

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"

int main(int argc, char **argv) {

  // Load cofigs
  YAML::Node config = YAML::LoadFile(argv[1]);

  double pub_rate = 40.;
  if (config["GRIPPER"]["PUB_RATE"]) {
    pub_rate = config["GRIPPER"]["PUB_RATE"].as<double>();
  }

  const std::string robot_ip = config["ROBOT"]["IP"].as<std::string>();

  // Subscribing gripper command
  const std::string subscriber_ip = config["PC"]["IP"].as<std::string>();
  const std::string sub_port =
      config["NUC"]["GRIPPER_SUB_PORT"].as<std::string>();

  // Publishing gripper command
  const std::string pub_port =
      config["NUC"]["GRIPPER_PUB_PORT"].as<std::string>();

  zmq_utils::ZMQPublisher zmq_pub(pub_port);
  zmq_utils::ZMQSubscriber zmq_sub(subscriber_ip, sub_port);

  // Initialize robot
  log_utils::initialize_logger(
      config["GRIPPER_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>(),
      config["GRIPPER_LOGGER"]["CONSOLE"]["LEVEL"].as<std::string>(),
      config["GRIPPER_LOGGER"]["CONSOLE"]["USE"].as<bool>(),
      config["GRIPPER_LOGGER"]["FILE"]["LOGGER_NAME"].as<std::string>(),
      config["GRIPPER_LOGGER"]["FILE"]["LEVEL"].as<std::string>(),
      config["GRIPPER_LOGGER"]["FILE"]["USE"].as<bool>());

  try {
    franka::Gripper gripper(robot_ip);

    std::atomic_bool running{true};
    std::atomic_bool command_in_progress{false};

    robot_utils::FrankaGripperStateUtils gripper_state_utils;
    struct {
      std::mutex mutex;
      franka::GripperState state;
    } gripper_state{};

    struct {
      std::mutex mutex;
      FrankaGripperControlMessage control_msg;
      std::uint64_t sequence{0};
    } gripper_cmd{};

    auto gripper_logger = log_utils::get_logger(
        config["GRIPPER_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());

    // Log information about current gripper control frequency
    gripper_logger->info("Gripper state publisher: {0}Hz", pub_rate);
    // Initialize gripper subscribing / publishing thread,
    std::thread gripper_pub_thread([&]() {
      franka::GripperState current_gripper_state;
      while (running) {
        if (gripper_state.mutex.try_lock()) {
          gripper_state.state = gripper.readOnce();
          current_gripper_state = gripper_state.state;
          gripper_state.mutex.unlock();
        }

        FrankaGripperStateMessage gripper_state_msg;
        gripper_state_utils.LoadGripperStateToMsg(current_gripper_state,
                                                  gripper_state_msg);
        std::string serialized_gripper_state_msg;
        gripper_state_msg.SerializeToString(&serialized_gripper_state_msg);
        zmq_pub.send(serialized_gripper_state_msg);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(1. / pub_rate * 1000)));
      }
    });

    // Keep the newest command in a versioned mailbox. Gripper operations are
    // blocking, so a newer command must interrupt the active operation and
    // remain pending until the main loop has processed its sequence number.
    std::thread gripper_sub_thread([&]() {
      while (running) {
        std::string msg;
        msg = zmq_sub.recv(false);
        FrankaGripperControlMessage control_msg;
        if (control_msg.ParseFromString(msg)) {
          bool interrupt_active_command = false;
          {
            std::lock_guard<std::mutex> lock(gripper_cmd.mutex);
            gripper_cmd.control_msg = control_msg;
            ++gripper_cmd.sequence;
            interrupt_active_command = command_in_progress.load();
          }

          if (control_msg.termination()) {
            running = false;
            interrupt_active_command = true;
          }

          if (interrupt_active_command) {
            try {
              gripper.stop();
            } catch (const franka::Exception &e) {
              gripper_logger->error("Failed to interrupt gripper command: {0}",
                                    e.what());
            }
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    gripper.homing();
    gripper_logger->info("Gripper homing complete");
    bool has_grasped = false;
    std::uint64_t processed_sequence = 0;
    // Main loop
    while (running) {
      FrankaGripperControlMessage last_control_msg;
      std::uint64_t command_sequence = 0;
      bool has_pending_command = false;
      {
        std::lock_guard<std::mutex> lock(gripper_cmd.mutex);
        if (gripper_cmd.sequence != processed_sequence) {
          last_control_msg = gripper_cmd.control_msg;
          command_sequence = gripper_cmd.sequence;
          has_pending_command = true;
          command_in_progress = true;
        }
      }

      if (!has_pending_command) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      FrankaGripperStopMessage homing_msg;
      FrankaGripperMoveMessage move_msg;
      FrankaGripperGraspMessage grasp_msg;
      FrankaGripperStopMessage stop_msg;

      try {
        auto gripper_control = last_control_msg.control_msg();
        if (gripper_control.UnpackTo(&homing_msg)) {
          gripper.homing();
          has_grasped = false;
        } else if (gripper_control.UnpackTo(&move_msg)) {
          gripper.move(move_msg.width(), move_msg.speed());
          has_grasped = false;
        } else if (gripper_control.UnpackTo(&grasp_msg)) {
          if (!has_grasped) {
            double epsilon_inner, epsilon_outer;
            if (grasp_msg.epsilon_inner() == 0. &&
                grasp_msg.epsilon_outer() == 0.) {
              // if not defined, we will keep epsilon high so that it won't get
              // stuck
              epsilon_inner = 0.08;
              epsilon_outer = 0.08;
            } else {
              epsilon_inner = grasp_msg.epsilon_inner();
              epsilon_outer = grasp_msg.epsilon_outer();
            }

            double force;
            if (grasp_msg.force() == 0.) {
              force = 2.0;
            } else {
              force = grasp_msg.force();
            }
            has_grasped = gripper.grasp(grasp_msg.width(), grasp_msg.speed(),
                                        force, epsilon_inner, epsilon_outer);

            gripper_logger->info("Grasped? {0}", has_grasped);
          }
        } else if (gripper_control.UnpackTo(&stop_msg)) {
          gripper.stop();
          has_grasped = false;
        } else {
          gripper_logger->warn("Unpack failed");
        }
      } catch (const franka::CommandException &e) {
        bool superseded = false;
        {
          std::lock_guard<std::mutex> lock(gripper_cmd.mutex);
          superseded = gripper_cmd.sequence != command_sequence;
        }
        if (superseded) {
          gripper_logger->debug("Gripper command interrupted by newer input");
        } else {
          gripper_logger->error("Gripper command failed: {0}", e.what());
        }
        has_grasped = false;
      }
      processed_sequence = command_sequence;
      command_in_progress = false;
    }
    gripper_sub_thread.join();
    gripper_pub_thread.join();

  } catch (franka::Exception const &e) {
    auto gripper_logger = log_utils::get_logger(
        config["GRIPPER_LOGGER"]["CONSOLE"]["LOGGER_NAME"].as<std::string>());
    gripper_logger->error(e.what());
    return -1;
  }

  return 0;
}
