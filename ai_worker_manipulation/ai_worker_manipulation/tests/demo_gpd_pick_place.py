#!/usr/bin/env python3
"""
Hardcoded GPD result → lift-based pick and place.

Full workflow:
  1. ros2 run ai_worker_manipulation move_wrist_capture_pose
  2. [perception node saves PCD to wrist_outputs/]
  3. DISPLAY=:0 python3 test_gpd_wrist150.py --frame 0   (update BASE_DIR first)
  4. Copy GRASP_POSITION / GRASP_ORIENTATION from visualization output into this file
  5. ros2 run ai_worker_manipulation demo_gpd_pick_place

Press Enter at each prompt to advance to the next step.

Usage:
  ros2 run ai_worker_manipulation demo_gpd_pick_place
  ros2 run ai_worker_manipulation demo_gpd_pick_place -- --skip-home
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface


# ---------------------------------------------------------------------------
# Fill these in from test_gpd_wrist150.py visualization output
# ---------------------------------------------------------------------------
GRASP_POSITION    = [0.3014, -0.2542, 0.8930]
GRASP_ORIENTATION = [0.3846, -0.0636, 0.0507, 0.9195]  # [x, y, z, w]

# ---------------------------------------------------------------------------
# Place pose — fixed location
# ---------------------------------------------------------------------------
PLACE_POSITION    = [0.30, 0.20, 1.00]          # TODO: set actual place position
PLACE_ORIENTATION = [0.0, 0.0, 0.0, 1.0]

# ---------------------------------------------------------------------------
LIFT_POSITION   = 0.0   # lift_joint starting position (m)
APPROACH_HEIGHT = 0.10  # arm pre-grasp z offset; lift descends this to reach grasp z


def prompt(msg: str):
    input(f'\n  >> {msg} ... [Enter]')


def make_pose(pos, ori) -> Pose:
    p = Pose()
    p.position.x    = float(pos[0])
    p.position.y    = float(pos[1])
    p.position.z    = float(pos[2])
    p.orientation.x = float(ori[0])
    p.orientation.y = float(ori[1])
    p.orientation.z = float(ori[2])
    p.orientation.w = float(ori[3])
    return p


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--skip-home', action='store_true',
                        help='skip home move at start (already near capture pose)')
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    rclpy.init()
    node    = rclpy.create_node('demo_gpd_pick_place')
    client  = MoveItClient(node)
    gripper = GripperInterface(node=node)
    log     = node.get_logger()

    pre_grasp  = make_pose(
        [GRASP_POSITION[0], GRASP_POSITION[1], GRASP_POSITION[2] + APPROACH_HEIGHT],
        GRASP_ORIENTATION,
    )
    place_pose = make_pose(PLACE_POSITION, PLACE_ORIENTATION)

    log.info(f'grasp  pos={GRASP_POSITION}  ori={GRASP_ORIENTATION}')
    log.info(f'place  pos={PLACE_POSITION}')
    log.info(f'pre-grasp z={pre_grasp.position.z:.3f} m  (grasp_z + {APPROACH_HEIGHT} m)')

    try:
        # ── 1. open gripper + home ─────────────────────────────────────
        prompt('open gripper + home')
        gripper.open('right')
        if not args.skip_home:
            result = client.move_to_home(arm=Arm.RIGHT)
            log.info(f'home: {result.value}')

        # ── 2. lift to start position + arm to pre-grasp ───────────────
        prompt(f'lift → {LIFT_POSITION} m  +  arm → pre-grasp z={pre_grasp.position.z:.3f} m')
        client.move_lift(LIFT_POSITION)
        result = client.move_to_pose(pre_grasp, arm=Arm.RIGHT)
        log.info(f'pre-grasp: {result.value}')
        if result.value != 'succeeded':
            log.error('pre-grasp failed — aborting')
            return

        # ── 3. lift descend to grasp height ────────────────────────────
        lift_grasp = LIFT_POSITION - APPROACH_HEIGHT
        prompt(f'lift descend → {lift_grasp:.3f} m  (reach grasp z)')
        client.move_lift(lift_grasp)

        # ── 4. close gripper ───────────────────────────────────────────
        prompt('close gripper')
        gripper.close('right')
        time.sleep(1.5)

        # ── 5. lift ascend ─────────────────────────────────────────────
        prompt(f'lift ascend → {LIFT_POSITION} m')
        client.move_lift(LIFT_POSITION)

        # ── 6. move to place ───────────────────────────────────────────
        prompt(f'arm → place pos={PLACE_POSITION}')
        result = client.move_to_pose(place_pose, arm=Arm.RIGHT)
        log.info(f'place move: {result.value}')
        if result.value != 'succeeded':
            log.error('place move failed — releasing at current position')

        # ── 7. open gripper ────────────────────────────────────────────
        prompt('open gripper (release)')
        gripper.open('right')
        time.sleep(1.0)

        # ── 8. home ────────────────────────────────────────────────────
        prompt('home')
        client.move_to_home(arm=Arm.RIGHT)
        log.info('demo complete')

    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
