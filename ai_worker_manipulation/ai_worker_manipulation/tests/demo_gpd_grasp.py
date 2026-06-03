"""
GPD grasp pose → move_to_pose + gripper close test.

카메라 없이 하드코딩된 grasp pose로 lift+arm 접근 동작을 검증하는 스크립트.
실제 GPD 연동 전 하드웨어 동작 확인용.

Usage:
  ros2 run ai_worker_manipulation demo_gpd_grasp
  ros2 run ai_worker_manipulation demo_gpd_grasp -- --skip-home
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface


# TODO: replace with actual GPD output values from live test
GRASP_POSITION    = [0.3014, -0.2542, 0.8930]
GRASP_ORIENTATION = [0.3846, -0.0636, 0.0507, 0.9195]

LIFT_POSITION   = 0.0   # lift_joint initial position (0.0 = top)
APPROACH_HEIGHT = 0.10  # arm moves to grasp_z + offset, then lift descends to reach grasp_z


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--skip-home', action='store_true',
                        help='skip move_to_home at the start')
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    rclpy.init()
    node    = rclpy.create_node('demo_gpd_grasp')
    client  = MoveItClient(node)
    log     = node.get_logger()
    gripper = GripperInterface(node=node)

    pre_grasp_pose = Pose()
    pre_grasp_pose.position.x    = GRASP_POSITION[0]
    pre_grasp_pose.position.y    = GRASP_POSITION[1]
    pre_grasp_pose.position.z    = GRASP_POSITION[2] + APPROACH_HEIGHT
    pre_grasp_pose.orientation.x = GRASP_ORIENTATION[0]
    pre_grasp_pose.orientation.y = GRASP_ORIENTATION[1]
    pre_grasp_pose.orientation.z = GRASP_ORIENTATION[2]
    pre_grasp_pose.orientation.w = GRASP_ORIENTATION[3]

    try:
        gripper.open('right')

        if not args.skip_home:
            log.info('moving to home...')
            client.move_to_home(arm=Arm.RIGHT)

        log.info(f'lift → {LIFT_POSITION} m')
        client.move_lift(LIFT_POSITION)

        log.info(f'pre-grasp move (z={pre_grasp_pose.position.z:.3f} m)')
        result = client.move_to_pose(pre_grasp_pose, arm=Arm.RIGHT)
        log.info(f'pre-grasp result: {result.value}')
        if result.value != 'succeeded':
            return 1

        # descend via lift to reach actual grasp height
        lift_grasp = LIFT_POSITION - APPROACH_HEIGHT
        log.info(f'lift descend → {lift_grasp:.3f} m')
        client.move_lift(lift_grasp)

        log.info('closing gripper')
        gripper.close('right')
        time.sleep(1.5)

        log.info('lift return → 0.0 m')
        client.move_lift(0.0)

        log.info(f'home: {client.move_to_home(arm=Arm.RIGHT).value}')
        return 0
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
