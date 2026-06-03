import time

from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface

GRASP_TOPIC = "/gpd/grasp_poses"


def wait_for_grasp(client: MoveItClient, timeout: float = 90.0) -> Pose | None:
    """GPD에서 grasp pose publish된거 기다리고, 첫 번째 후보를 반환."""
    log = client.node.get_logger()
    log.info("Waiting for grasp result...")
    all_poses = []

    def callback(msg: PoseArray):
        if not all_poses:
            log.info(f"Received {len(msg.poses)} grasp candidates.")
            all_poses.extend(msg.poses)

    sub = client.node.create_subscription(PoseArray, GRASP_TOPIC, callback, 10)

    deadline = time.time() + timeout
    while not all_poses and time.time() < deadline:
        time.sleep(0.1)

    client.node.destroy_subscription(sub)

    if not all_poses:
        log.error("No grasp candidates received within timeout")
        return None

    log.info("Selected grasp candidate 0")
    return all_poses[0]


def pre_grasp_of(pose: Pose, offset: float = 0.15) -> Pose:
    q = pose.orientation
    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    approach_vector = r[:, 0]

    pre = Pose()
    pre.position.x = pose.position.x - offset * approach_vector[0]
    pre.position.y = pose.position.y - offset * approach_vector[1]
    pre.position.z = pose.position.z - offset * approach_vector[2]
    pre.orientation = pose.orientation
    return pre


def pick(client: MoveItClient, gripper: GripperInterface, grasp: Pose):
    log = client.node.get_logger()
    pre = pre_grasp_of(grasp)

    log.info("Moving to pre-grasp")
    client.move_to_pose(pre)
    log.info("Cartesian move to grasp")
    client.move_cartesian(grasp)
    log.info("Closing gripper")
    gripper.close('right')
    log.info("Retracting to pre-grasp")
    client.move_cartesian(pre)


def place(client: MoveItClient, gripper: GripperInterface, place_pose: Pose):
    log = client.node.get_logger()
    pre = pre_grasp_of(place_pose)

    log.info("Moving to pre-place")
    client.move_to_pose(pre)
    log.info("Cartesian move to place")
    client.move_cartesian(place_pose)
    log.info("Opening gripper")
    gripper.open('right')
    log.info("Retracting from place")
    client.move_cartesian(pre)
