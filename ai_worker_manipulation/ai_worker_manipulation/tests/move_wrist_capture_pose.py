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
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from pymoveit2.moveit2 import MoveIt2State

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


def move_lift(node: Node, position: float, cb_group, timeout: float = 10.0) -> bool:
    moveit_lift = MoveIt2(
        node=node,
        joint_names=['lift_joint'],
        base_link_name='base_link',
        end_effector_name='lift_link',
        group_name='lift',
        callback_group=cb_group,
        use_move_group_action=True,
    )
    moveit_lift.max_velocity = 0.2
    moveit_lift.max_acceleration = 0.2
    moveit_lift.move_to_configuration([position])

    deadline = time.time() + timeout
    while moveit_lift.query_state() == MoveIt2State.IDLE:
        if time.time() > deadline:
            node.get_logger().warn('lift: goal never left IDLE')
            return False
        time.sleep(0.01)

    while moveit_lift.query_state() != MoveIt2State.IDLE:
        if time.time() > deadline:
            node.get_logger().warn('lift: timeout')
            return False
        time.sleep(0.05)

    ok = bool(moveit_lift.motion_suceeded)
    node.get_logger().info(
        f'lift -> {position:.3f} m: {"SUCCEEDED" if ok else "FAILED"}'
    )
    return ok


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--lift', type=float, default=DEFAULT_LIFT_POSITION,
                        help='lift_joint target in metres; -0.15 means 15 cm down')
    parser.add_argument('--x', type=float, default=DEFAULT_EE_POSITION[0],
                        help='right end-effector x in base_link frame [m]')
    parser.add_argument('--y', type=float, default=DEFAULT_EE_POSITION[1],
                        help='right end-effector y in base_link frame [m]')
    parser.add_argument('--z', type=float, default=DEFAULT_EE_POSITION[2],
                        help='right end-effector z in base_link frame [m]')
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
    node = Node('move_wrist_capture_pose')
    client = MoveItClient(node)
    log = node.get_logger()
    gripper = GripperInterface(node=node)

    pose = make_pose(
        (args.x, args.y, args.z),
        (args.qx, args.qy, args.qz, args.qw),
    )

    try:
        log.info(
            'right wrist PCD capture pose: '
            f'lift={args.lift:.3f} m, '
            f'ee=({args.x:.3f}, {args.y:.3f}, {args.z:.3f}), '
            f'quat=({args.qx:.3f}, {args.qy:.3f}, {args.qz:.3f}, {args.qw:.3f})'
        )

        if not args.keep_gripper:
            gripper.open('right')

        if not args.skip_home:
            home = client.move_to_home(arm=Arm.RIGHT, velocity=0.15)
            if home.value != 'succeeded':
                log.warn(f'right arm home move result: {home.value}')

        if not move_lift(node, args.lift, client._cb_group):
            log.error('lift move failed; aborting capture pose move')
            return 1

        result = client.move_to_pose(
            pose,
            arm=Arm.RIGHT,
            velocity=0.10,
            acceleration=0.10,
            timeout=30.0,
        )
        log.info(f'right arm capture pose result: {result.value}')
        if result.value != 'succeeded':
            return 1

        if args.settle > 0.0:
            log.info(f'settling for {args.settle:.1f}s before PCD capture')
            time.sleep(args.settle)

        log.info('ready for right-wrist PCD capture')
        return 0
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
