"""Move robot up and print joint positions."""
import argparse

import numpy as np

from deoxys import config_root
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.log_utils import get_deoxys_example_logger

logger = get_deoxys_example_logger()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface-cfg", type=str, default="charmander.yml")
    parser.add_argument("--distance", type=float, default=10.0, help="Distance to move up (cm)")
    args = parser.parse_args()

    robot_interface = FrankaInterface(
        config_root + f"/{args.interface_cfg}", use_visualizer=False
    )
    controller_cfg = get_default_controller_config("OSC_POSE")

    target_distance = args.distance / 100.0  # cm to m

    # Wait for state
    logger.info("Waiting for robot state...")
    while len(robot_interface._state_buffer) == 0:
        robot_interface.control(
            controller_type="OSC_POSE",
            action=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            controller_cfg=controller_cfg,
        )

    # Save initial pose
    start_pose = robot_interface.last_eef_pose
    start_pos = start_pose[:3, 3].copy()
    start_rot = start_pose[:3, :3].copy()
    start_quat = transform_utils.mat2quat(start_rot)

    logger.info(f"Start position: {np.round(start_pos, 3)}")
    logger.info(f"Start rotation matrix:\n{np.round(start_rot, 3)}")
    logger.info(f"Moving up {args.distance} cm while maintaining rotation...")

    while True:
        current_pose = robot_interface.last_eef_pose
        current_pos = current_pose[:3, 3]
        current_rot = current_pose[:3, :3]
        current_quat = transform_utils.mat2quat(current_rot)

        # Compute rotation error to maintain initial rotation
        if np.dot(start_quat, current_quat) < 0.0:
            current_quat = -current_quat
        quat_diff = transform_utils.quat_distance(start_quat, current_quat)
        rot_action = transform_utils.quat2axisangle(quat_diff).flatten()

        z_moved = current_pos[2] - start_pos[2]
        joint_pos = np.array(robot_interface.last_q)

        print(f"Z: {z_moved*100:.2f} cm | Joints: {np.round(joint_pos, 3)}")

        if z_moved >= target_distance:
            logger.info("Done!")
            logger.info(f"Final rotation matrix:\n{np.round(current_rot, 3)}")
            break

        # Move up (z+) and correct rotation to maintain initial orientation
        action = [0.0, 0.0, 0.5, rot_action[0], rot_action[1], rot_action[2], -1.0]
        robot_interface.control(
            controller_type="OSC_POSE",
            action=action,
            controller_cfg=controller_cfg,
        )

    robot_interface.close()


if __name__ == "__main__":
    main()
