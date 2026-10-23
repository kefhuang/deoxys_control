"""Test joint impedance controller accuracy at 20Hz command rate.

Goes to the same target joint position N times, recording the steady-state
error each time. Between trials, moves to an offset position so each
approach is independent.
"""
import argparse
import time

import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()

COMMAND_RATE = 20  # Hz
COMMAND_PERIOD = 1.0 / COMMAND_RATE

# How long to hold the target before declaring "settled"
SETTLE_TIME = 2.0  # seconds
# Max joint error (rad) to consider the robot settled
SETTLE_THRESHOLD = 1e-3
# Number of repeated trials
NUM_TRIALS = 10

# Target joint position (a typical home-ish configuration)
TARGET_JOINTS = np.array([
    0.09162008114028396,
    -0.19826458111314524,
    -0.01990020486871322,
    -2.4732269941140346,
    -0.01307073642274261,
    2.30396583422025,
    0.8480939705504309,
])

# Offset to move to between trials (radians added to target)
OFFSET = np.array([0.15, -0.15, 0.15, 0.15, -0.15, 0.15, -0.15])


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument(
        "--controller-cfg", type=str, default="joint-impedance-controller.yml"
    )
    parser.add_argument("--num-trials", type=int, default=NUM_TRIALS)
    parser.add_argument("--settle-time", type=float, default=SETTLE_TIME)
    parser.add_argument("--command-rate", type=int, default=COMMAND_RATE)
    args = parser.parse_args()
    return args


def move_to_joints(robot_interface, controller_cfg, target, command_period, timeout=7.0):
    """Move to target joints at the given command rate. Returns when settled or timed out.

    Returns:
        final_q: actual joint positions when settled (numpy array, shape (7,))
        settled: whether the robot reached the target within threshold
    """
    action = target.tolist() + [-1.0]
    start_time = time.time()
    settled = False

    while True:
        loop_start = time.time()

        if robot_interface.received_states and robot_interface.check_nonzero_configuration():
            current_q = np.array(robot_interface.last_q)
            max_err = np.max(np.abs(current_q - target))
            if max_err < SETTLE_THRESHOLD:
                settled = True
                break

        if time.time() - start_time > timeout:
            break

        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )

        # Enforce command rate
        elapsed = time.time() - loop_start
        sleep_time = command_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Hold position briefly and read final state
    for _ in range(5):
        robot_interface.control(
            controller_type="JOINT_IMPEDANCE",
            action=action,
            controller_cfg=controller_cfg,
        )
        time.sleep(command_period)

    final_q = np.array(robot_interface.last_q)
    return final_q, settled


def main():
    args = parse_args()
    command_period = 1.0 / args.command_rate

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = YamlConfig(
        config_root + f"/{args.controller_cfg}"
    ).as_easydict()

    # Wait for robot state
    logger.info("Waiting for robot state...")
    while len(robot_interface._state_buffer) == 0:
        time.sleep(0.1)
    logger.info("Robot state received.")

    target = TARGET_JOINTS
    offset_pos = TARGET_JOINTS + OFFSET

    errors = []
    per_joint_errors = []

    logger.info(f"Running {args.num_trials} trials at {args.command_rate}Hz command rate")
    logger.info(f"Target joints: {np.round(target, 4)}")
    logger.info("-" * 70)

    for trial in range(args.num_trials):
        # Move to offset position first (so each approach is fresh)
        logger.info(f"Trial {trial + 1}/{args.num_trials}: moving to offset position...")
        move_to_joints(robot_interface, controller_cfg, offset_pos, command_period, timeout=5.0)
        time.sleep(0.5)  # brief pause

        # Now move to target and measure
        logger.info(f"Trial {trial + 1}/{args.num_trials}: moving to target...")
        final_q, settled = move_to_joints(
            robot_interface, controller_cfg, target, command_period, timeout=args.settle_time + 3.0
        )

        error = final_q - target
        abs_error = np.abs(error)
        max_error = np.max(abs_error)
        rms_error = np.sqrt(np.mean(error ** 2))

        errors.append(error)
        per_joint_errors.append(abs_error)

        logger.info(
            f"  Settled: {settled} | "
            f"Max error: {max_error:.6f} rad ({np.degrees(max_error):.4f} deg) | "
            f"RMS error: {rms_error:.6f} rad ({np.degrees(rms_error):.4f} deg)"
        )
        logger.info(f"  Per-joint error (rad): {np.round(abs_error, 6)}")

    # Summary statistics
    errors = np.array(errors)
    per_joint_errors = np.array(per_joint_errors)

    logger.info("=" * 70)
    logger.info(f"SUMMARY ({args.num_trials} trials at {args.command_rate}Hz)")
    logger.info("=" * 70)

    mean_abs = np.mean(per_joint_errors, axis=0)
    std_abs = np.std(per_joint_errors, axis=0)
    max_abs = np.max(per_joint_errors, axis=0)

    logger.info(f"Mean absolute error per joint (rad): {np.round(mean_abs, 6)}")
    logger.info(f"Std  absolute error per joint (rad): {np.round(std_abs, 6)}")
    logger.info(f"Max  absolute error per joint (rad): {np.round(max_abs, 6)}")
    logger.info("")

    overall_max = np.max(per_joint_errors)
    overall_mean = np.mean(per_joint_errors)
    overall_rms = np.sqrt(np.mean(errors ** 2))

    logger.info(f"Overall max error:  {overall_max:.6f} rad ({np.degrees(overall_max):.4f} deg)")
    logger.info(f"Overall mean error: {overall_mean:.6f} rad ({np.degrees(overall_mean):.4f} deg)")
    logger.info(f"Overall RMS error:  {overall_rms:.6f} rad ({np.degrees(overall_rms):.4f} deg)")

    # Repeatability: std of final positions across trials
    final_positions = target + errors  # shape (num_trials, 7)
    repeatability = np.std(final_positions, axis=0)
    logger.info(f"\nRepeatability (std of final positions per joint, rad): {np.round(repeatability, 6)}")
    logger.info(f"Repeatability (std of final positions per joint, deg): {np.round(np.degrees(repeatability), 4)}")

    robot_interface.close()


if __name__ == "__main__":
    main()
