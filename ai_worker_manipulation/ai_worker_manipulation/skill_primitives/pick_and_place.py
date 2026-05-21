from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.robot_interface.gripper_controller import GripperController
from geometry_msgs.msg import PoseArray, Pose
import rclpy
import time
from scipy.spatial.transform import Rotation

GRASP_TOPIC = "/grasp_poses" # 나중에 이름 바뀌면 여기서 바꾸자

def wait_for_grasp(client: MoveItClient, timeout: float = 30.0) -> Pose | None:
    "GPD에서 grasp pose publish된거 기다리고, 그중 가장 높은 점수 + kinematically reachable한 하나의 Pose 반환"
    log = client.node.get_logger()
    log.info("Waiting for grasp result...")
    all_poses = []

    def callback(msg: PoseArray):
        if not all_poses:  # take only the first message, ignore duplicate publishes
            log.info(f"Received {len(msg.poses)} grasp candidates.")
            all_poses.extend(msg.poses)

    sub = client.node.create_subscription(PoseArray, GRASP_TOPIC, callback, 10)

    deadline = time.time() + timeout
    while not all_poses and time.time() < deadline:
        rclpy.spin_once(client.node, timeout_sec=0.1)

    client.node.destroy_subscription(sub)

    if not all_poses:
        log.error("No grasp candidates received within timeout")
        return None

    for i, pose in enumerate(all_poses):
        if client.check_reachable(pose):
            log.info(f"Selected grasp candidate {i} (best reachable)")
            return pose
        log.warn(f"Grasp candidate {i} is not reachable, trying next")

    log.error("No reachable grasp candidates found")
    return None



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

    #offset을 어떻게 할지 trial and error 필요. 너무 멀면 경로 생성 실패, 너무 가까우면 충돌 위험. 15cm 정도가 적당할 것으로 예상


def pick(client: MoveItClient, gripper: GripperController, grasp: Pose) -> bool:
    log = client.node.get_logger()
    pre = pre_grasp_of(grasp)

    log.info("Moving to pre-grasp")
    if not client.move_to_pose(pre):
        return False
    log.info("Cartesian move to grasp")
    if not client.cartesian_move(grasp):
        return False
    log.info("Closing gripper")
    try:
        gripper.close('right')
    except:
        return False
    #여기에 grasp stability 추가
    log.info("Retracting to pre-grasp")
    if not client.cartesian_move(pre):
        return False

    return True    

    #더 나은 방법이 있는지 검토 필요


def place(client: MoveItClient, gripper: GripperController, place_pose: Pose) -> bool:
    log = client.node.get_logger()
    pre = pre_grasp_of(place_pose)

    log.info("Moving to pre-place")
    if not client.move_to_pose(pre):
        return False
    log.info("Cartesian move to place")
    if not client.cartesian_move(place_pose):
        return False
    log.info("Opening gripper")
    try:
        gripper.open('right')
    except:
        return False
    log.info("Retracting from place")
    if not client.cartesian_move(pre):
        return False

    return True    
