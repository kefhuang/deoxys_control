"""Test script for gripper open/close cycle."""
import time

from deoxys import config_root
from deoxys.experimental.motion_utils import position_only_gripper_move_by
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def main():
    robot_interface = FrankaInterface(
        config_root + "/charmander.yml", use_visualizer=False
    )

    # 1. 开
    logger.info("Opening gripper...")
    position_only_gripper_move_by(robot_interface, delta_pos=[0, 0, 0], grasp=False)
    logger.info("Gripper opened")

    # 2. 隔 5 秒关
    time.sleep(5)
    logger.info("Closing gripper...")
    position_only_gripper_move_by(robot_interface, delta_pos=[0, 0, 0], grasp=True)
    logger.info("Gripper closed")

    # 3. 再隔 5 秒开
    time.sleep(5)
    logger.info("Opening gripper...")
    position_only_gripper_move_by(robot_interface, delta_pos=[0, 0, 0], grasp=False)
    logger.info("Gripper opened")

    # 4. 再隔 5 秒关
    time.sleep(5)
    logger.info("Closing gripper...")
    position_only_gripper_move_by(robot_interface, delta_pos=[0, 0, 0], grasp=True)
    logger.info("Gripper closed")

    # 最后保持开启
    time.sleep(5)
    logger.info("Final: opening gripper...")
    position_only_gripper_move_by(robot_interface, delta_pos=[0, 0, 0], grasp=False)
    logger.info("Gripper opened (final state)")

    robot_interface.close()


if __name__ == "__main__":
    main()
