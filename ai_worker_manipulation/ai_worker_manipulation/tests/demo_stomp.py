import rclpy
from geometry_msgs.msg import Pose
from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, MoveResult

def _pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p

# Target: safely inside workspace
# Start with identity orientation to confirm planner works,
# then swap in the desired orientation once baseline passes.
TARGET = _pose(0.35, -0.25, 0.80)  # identity: qw=1.0

def main():
    rclpy.init()
    client = MoveItClient()
    log = client.node.get_logger()

    log.info(
        f"Testing move_pose_with_stomp_smoothing → "
        f"pos=({TARGET.position.x}, {TARGET.position.y}, {TARGET.position.z}) "
        f"quat=({TARGET.orientation.x:.4f}, {TARGET.orientation.y:.4f}, "
        f"{TARGET.orientation.z:.4f}, {TARGET.orientation.w:.4f})"
    )

    try:
        result = client.move_pose_with_stomp_smoothing(TARGET)
        if result == MoveResult.SUCCEEDED:
            log.info("SUCCESS: move_pose_with_stomp_smoothing reached target")
        else:
            log.error(f"FAILED: move_pose_with_stomp_smoothing returned {result}")
    finally:
        client.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
