#!/usr/bin/env python3
"""
pick_and_place_server.py
-------------------------
Thin ROS 2 Action Server wrapping PickAndPlaceOrchestrator.
All logic lives in the orchestrator — this node only handles ROS interfaces.
"""

import yaml

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import ament_index_python.packages as ament

from ai_worker_manipulation_msgs.action import PickAndPlace

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill
from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill
from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill
from ai_worker_manipulation.skill_primitives.pick_and_place_orchestrator import (
    PickAndPlaceOrchestrator,
    OrchestratorResult,
)


def _load_config() -> dict:
    pkg_dir = ament.get_package_share_directory('ai_worker_manipulation')
    path    = f'{pkg_dir}/config/pick_and_place.yaml'
    with open(path) as f:
        return yaml.safe_load(f)['pick_and_place']


class PickAndPlaceServer(Node):

    def __init__(self):
        super().__init__('pick_and_place_server')

        config     = _load_config()
        moveit     = MoveItClient(self)
        gripper    = GripperInterface(node=self)
        assessment = GraspAssessment(self)
        grasp      = GraspSkill(self, gripper, assessment)
        pick       = PickSkill(self, moveit, gripper, grasp)
        place      = PlaceSkill(self, moveit, gripper)

        self._orchestrator = PickAndPlaceOrchestrator(
            node=self,
            moveit=moveit,
            gripper=gripper,
            pick_skill=pick,
            place_skill=place,
            config=config,
        )

        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            self._execute_cb,
        )
        self.get_logger().info('[PickAndPlaceServer] ready')

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request

        def _feedback(phase: str):
            fb       = PickAndPlace.Feedback()
            fb.phase = phase
            goal_handle.publish_feedback(fb)

        self._orchestrator._feedback_cb = _feedback

        result_enum = self._orchestrator.run(
            object_class=goal.object_class,
            place_pose=goal.place_pose.pose,
        )

        result                = PickAndPlace.Result()
        result.success        = result_enum == OrchestratorResult.SUCCESS
        result.failure_reason = '' if result.success else result_enum.value

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main():
    rclpy.init()
    node = PickAndPlaceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
