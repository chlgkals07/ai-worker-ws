#!/usr/bin/env python3
"""
Standalone pick and place test with hardcoded GPD output.
No Action Server or perception team needed.

Update GRASP_POSITION / GRASP_ORIENTATION from test_gpd_wrist_live.py output.

Usage:
  ros2 run ai_worker_manipulation test_pick_and_place_orchestrator
"""

import rclpy
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill
from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill
from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill

# ---------------------------------------------------------------------------
# Update these from test_gpd_wrist_live.py visualization output
# ---------------------------------------------------------------------------
GRASP_POSITION    = [0.3014, -0.2542, 0.8930]
GRASP_ORIENTATION = [0.3846, -0.0636, 0.0507, 0.9195]  # [x, y, z, w]

PLACE_POSITION    = [0.30, 0.20, 1.00]
PLACE_ORIENTATION = [0.0, 0.0, 0.0, 1.0]
# ---------------------------------------------------------------------------


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


def main():
    rclpy.init()
    node       = rclpy.create_node('test_pick_and_place_orchestrator')
    log        = node.get_logger()

    moveit     = MoveItClient(node)
    gripper    = GripperInterface(node=node)
    assessment = GraspAssessment(node)
    grasp      = GraspSkill(node, gripper, assessment)
    pick       = PickSkill(node, moveit, gripper, grasp)
    place      = PlaceSkill(node, moveit, gripper)

    grasp_pose = make_pose(GRASP_POSITION, GRASP_ORIENTATION)
    place_pose = make_pose(PLACE_POSITION, PLACE_ORIENTATION)

    log.info('--- Testing PickSkill ---')
    pick_result = pick.pick(grasp_pose, arm=Arm.RIGHT, object_name='ETC')
    log.info(f'Pick result: {pick_result.value}')

    if pick_result.value == 'success':
        log.info('--- Testing PlaceSkill ---')
        place_result = place.place(place_pose, arm=Arm.RIGHT)
        log.info(f'Place result: {place_result.value}')

    moveit.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
