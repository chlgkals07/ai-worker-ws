from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.ai_worker_manipulation.mission_control.environment import setup_environment
from ai_worker_manipulation.robot_interface.gripper_controller import GripperController
from ai_worker_manipulation.skill_primitives.pick_and_place import wait_for_grasp, pick, place
from geometry_msgs.msg import Pose
import rclpy

# Set True to skip GPD and use hardcoded poses for testing in RViz
DUMMY_MODE = False

# Front approach grasp (identity orientation -> approach vector along +X)
# Coordinates verified working in demo_0513
# pick() will auto-compute pre_grasp 15cm back along X: (0.20, -0.25, 0.80)
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
    client = MoveItClient()
    log = client.node.get_logger()
    gripper = GripperController(node=client.node)
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()