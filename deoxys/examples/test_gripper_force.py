"""Test script for gripper force from 10N to 70N.

Opens the gripper, then asks for user confirmation before closing
at each force level.
"""
import argparse
import time

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--interface-cfg", type=str, default="charmander.yml"
    )
    args = parser.parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    time.sleep(1)

    force_levels = list(range(10, 80, 10))  # 10, 20, 30, 40, 50, 60, 70

    for force in force_levels:
        # Open gripper
        logger.info("Opening gripper...")
        robot_interface.gripper_control(-1.0)
        time.sleep(2)
        logger.info("Gripper opened.")

        # Wait for user input
        user_input = input(
            f"\nReady to close gripper at {force}N. "
            f"Press Enter to proceed, or 'q' to quit: "
        )
        if user_input.strip().lower() == "q":
            logger.info("Quitting.")
            break

        # Override grasp force and close
        robot_interface._gripper_cfg.grasp_force = float(force)
        logger.info(f"Closing gripper with force={force}N ...")
        robot_interface.gripper_control(1.0)
        time.sleep(2)
        logger.info(f"Gripper closed at {force}N.")

    # Final: open gripper
    logger.info("Final: opening gripper...")
    robot_interface.gripper_control(-1.0)
    time.sleep(2)
    logger.info("Gripper opened (final state).")

    robot_interface.close()


if __name__ == "__main__":
    main()
