import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.environment import setup_environment
from ai_worker_manipulation.skill_primitives.pick_and_place import wait_for_grasp, pick, place

DUMMY_MODE = False

DUMMY_GRASP = Pose()
DUMMY_GRASP.position.x = 0.35
DUMMY_GRASP.position.y = -0.25
DUMMY_GRASP.position.z = 0.80
DUMMY_GRASP.orientation.w = 1.0

DUMMY_PLACE = Pose()
DUMMY_PLACE.position.x = 0.35
DUMMY_PLACE.position.y = 0.0
DUMMY_PLACE.position.z = 0.80
DUMMY_PLACE.orientation.w = 1.0


def main():
    rclpy.init()
    node    = Node('demo_0520')
    client  = MoveItClient(node)
    log     = node.get_logger()
    gripper = GripperInterface(node=node)
    setup_environment(client)

    gripper.open('right')
    client.move_to_home()

    try:
        if DUMMY_MODE:
            log.info("DUMMY_MODE: using hardcoded grasp and place poses")
            grasp_pose = DUMMY_GRASP
            place_pose = DUMMY_PLACE
        else:
            grasp_pose = wait_for_grasp(client)
            if grasp_pose is None:
                log.error("Failed to receive grasp result.")
                return
            place_pose = DUMMY_PLACE

        pick(client, gripper, grasp_pose)
        place(client, gripper, place_pose)

    finally:
        client.move_to_home()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
