"""Simple bimanual control example.

Connects to two Frankas, reads their current positions,
moves both up 10cm, then down 10cm, then opens/closes grippers together.

Usage:
    python examples/bimanual_example.py
"""
import threading
import time

import numpy as np

from deoxys import config_root
from deoxys.experimental.motion_utils import position_only_gripper_move_by
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()

CONFIG_LEFT = config_root + "/franka-two.yml"
CONFIG_RIGHT = config_root + "/franka-three.yml"


def wait_for_state(robot, name):
    """Block until the robot has received at least one state."""
    while robot.state_buffer_size == 0:
        logger.info(f"Waiting for {name} state...")
        time.sleep(0.5)


def print_ee_pos(robot, name):
    """Print the current end-effector position."""
    _, pos = robot.last_eef_rot_and_pos
    logger.info(f"{name} EE position: {pos.flatten()}")


def run_in_parallel(fn_left, fn_right):
    """Run two blocking calls in parallel and wait for both."""
    t1 = threading.Thread(target=fn_left)
    t2 = threading.Thread(target=fn_right)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


def main():
    # --- Initialize both robots ---
    logger.info("Initializing left robot (franka-two)...")
    robot_left = FrankaInterface(CONFIG_LEFT, use_visualizer=False)

    logger.info("Initializing right robot (franka-three)...")
    robot_right = FrankaInterface(CONFIG_RIGHT, use_visualizer=False)

    wait_for_state(robot_left, "left")
    wait_for_state(robot_right, "right")

    # --- Read current positions ---
    logger.info("=== Current positions ===")
    print_ee_pos(robot_left, "Left")
    print_ee_pos(robot_right, "Right")

    # --- Warmup: send zero-delta commands to settle OSC controller ---
    logger.info("Warming up controllers...")
    run_in_parallel(
        lambda: position_only_gripper_move_by(
            robot_left, delta_pos=[0, 0, 0], num_steps=40, grasp=False
        ),
        lambda: position_only_gripper_move_by(
            robot_right, delta_pos=[0, 0, 0], num_steps=40, grasp=False
        ),
    )

    # --- Move both up 10cm ---
    logger.info("=== Moving both robots UP 10cm ===")
    run_in_parallel(
        lambda: position_only_gripper_move_by(
            robot_left, delta_pos=[0, 0, 0.1], num_steps=80, grasp=False
        ),
        lambda: position_only_gripper_move_by(
            robot_right, delta_pos=[0, 0, 0.1], num_steps=80, grasp=False
        ),
    )
    print_ee_pos(robot_left, "Left")
    print_ee_pos(robot_right, "Right")

    time.sleep(1)

    # --- Move both down 10cm ---
    logger.info("=== Moving both robots DOWN 10cm ===")
    run_in_parallel(
        lambda: position_only_gripper_move_by(
            robot_left, delta_pos=[0, 0, -0.1], num_steps=80, grasp=False
        ),
        lambda: position_only_gripper_move_by(
            robot_right, delta_pos=[0, 0, -0.1], num_steps=80, grasp=False
        ),
    )
    print_ee_pos(robot_left, "Left")
    print_ee_pos(robot_right, "Right")

    time.sleep(1)

    # --- Close grippers ---
    logger.info("=== Closing both grippers ===")
    run_in_parallel(
        lambda: position_only_gripper_move_by(
            robot_left, delta_pos=[0, 0, 0], grasp=True
        ),
        lambda: position_only_gripper_move_by(
            robot_right, delta_pos=[0, 0, 0], grasp=True
        ),
    )
    logger.info("Grippers closed")

    time.sleep(2)

    # --- Open grippers ---
    logger.info("=== Opening both grippers ===")
    run_in_parallel(
        lambda: position_only_gripper_move_by(
            robot_left, delta_pos=[0, 0, 0], grasp=False
        ),
        lambda: position_only_gripper_move_by(
            robot_right, delta_pos=[0, 0, 0], grasp=False
        ),
    )
    logger.info("Grippers opened")

    # --- Cleanup ---
    robot_left.close()
    robot_right.close()
    logger.info("Done!")


if __name__ == "__main__":
    main()
