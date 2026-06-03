#!/usr/bin/env python3
"""
Move the robot into a right-wrist PCD capture pose.

Usage:
  ros2 run ai_worker_manipulation move_wrist_capture_pose
  ros2 run ai_worker_manipulation move_wrist_capture_pose -- --x 0.45 --y -0.2 --z 1.2
  ros2 run ai_worker_manipulation move_wrist_capture_pose -- --lift -0.15 --settle 3.0
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.robot_interface.moveit_client import Arm, MoveItClient


DEFAULT_LIFT_POSITION = 0.0
DEFAULT_EE_POSITION = (0.35, -0.20, 1.20)
DEFAULT_EE_ORIENTATION = (0.086, -0.173, 0.015, 0.981)


def make_pose(position: tuple[float, float, float],
              orientation: tuple[float, float, float, float]) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lift', type=float, default=DEFAULT_LIFT_POSITION,
                        help='lift_joint target in metres; -0.15 means 15 cm down')
    parser.add_argument('--x', type=float, default=DEFAULT_EE_POSITION[0])
    parser.add_argument('--y', type=float, default=DEFAULT_EE_POSITION[1])
    parser.add_argument('--z', type=float, default=DEFAULT_EE_POSITION[2])
    parser.add_argument('--qx', type=float, default=DEFAULT_EE_ORIENTATION[0])
    parser.add_argument('--qy', type=float, default=DEFAULT_EE_ORIENTATION[1])
    parser.add_argument('--qz', type=float, default=DEFAULT_EE_ORIENTATION[2])
    parser.add_argument('--qw', type=float, default=DEFAULT_EE_ORIENTATION[3])
    parser.add_argument('--settle', type=float, default=2.0,
                        help='seconds to wait after reaching the pose')
    parser.add_argument('--skip-home', action='store_true',
                        help='do not move right arm to home before the capture pose')
    parser.add_argument('--keep-gripper', action='store_true',
                        help='do not open the right gripper before moving')
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    rclpy.init()
    node = rclpy.create_node('move_wrist_capture_pose')
    client = MoveItClient(node)
    log = node.get_logger()
    gripper = GripperInterface(node=node)

    pose = make_pose(
        (args.x, args.y, args.z),
        (args.qx, args.qy, args.qz, args.qw),
    )

    try:
        log.info(
            f'right wrist PCD capture pose: '
            f'lift={args.lift:.3f} m, '
            f'ee=({args.x:.3f}, {args.y:.3f}, {args.z:.3f})'
        )

        if not args.keep_gripper:
            gripper.open('right')

        if not args.skip_home:
            result = client.move_to_home(arm=Arm.RIGHT, velocity=0.15)
            if result.value != 'succeeded':
                log.warn(f'home move result: {result.value}')

        result = client.move_lift(args.lift)
        if result.value != 'succeeded':
            log.error(f'lift move failed: {result.value}')
            return 1

        result = client.move_to_pose(pose, arm=Arm.RIGHT, velocity=0.10, acceleration=0.10)
        log.info(f'capture pose result: {result.value}')
        if result.value != 'succeeded':
            return 1

        if args.settle > 0.0:
            log.info(f'settling {args.settle:.1f}s before PCD capture')
            time.sleep(args.settle)

        log.info('ready for right-wrist PCD capture')
        return 0
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
