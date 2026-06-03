import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, MoveResult
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.environment import setup_environment

INTERMEDIATE_JOINTS = [0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0]


def main():
    rclpy.init()
    node   = Node('demo_0513')
    client = MoveItClient(node)
    log    = node.get_logger()

    setup_environment(client)
    log.info('Environment Ready')

    log.info(f'Initial joints: {client.get_joints()}')
    log.info(f'Initial pose:   {client.get_pose()}')

    gripper = GripperInterface(node=node)
    gripper.open('right')

    dummy_pose = Pose()
    dummy_pose.position.x = 0.35
    dummy_pose.position.y = -0.25
    dummy_pose.position.z = 0.8
    dummy_pose.orientation.w = 1.0

    log.info(f'move_to_home:   {client.move_to_home().value}')
    log.info(f'move_to_pose:   {client.move_to_pose(dummy_pose).value}')
    dummy_pose.position.z = 0.75
    log.info(f'move_cartesian: {client.move_cartesian(dummy_pose).value}')

    log.info('Opening gripper')
    gripper.open('right')
    time.sleep(1.5)
    log.info('Closing gripper')
    gripper.close('right')
    time.sleep(1.5)

    log.info(f'move_to_home:   {client.move_to_home().value}')
    log.info('YAY')

    client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
