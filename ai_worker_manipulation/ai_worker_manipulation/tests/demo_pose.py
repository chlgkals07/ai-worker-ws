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
TARGET = _pose(0.35, -0.25, 0.80)

def main():
    rclpy.init()
    client = MoveItClient()
    log = client.node.get_logger()

    log.info(f"Testing move_to_pose → ({TARGET.position.x}, {TARGET.position.y}, {TARGET.position.z})")

    try:
        result = client.move_to_pose(TARGET)
        if result == MoveResult.SUCCEEDED:
            log.info("SUCCESS: move_to_pose reached target")
        else:
            log.error(f"FAILED: move_to_pose returned {result}")
    finally:
        client.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
