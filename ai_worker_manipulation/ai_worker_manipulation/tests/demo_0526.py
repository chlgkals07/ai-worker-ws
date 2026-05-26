from geometry_msgs.msg import Pose
import rclpy
import time

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.robot_interface.gripper_controller_demo import GripperController
from ai_worker_manipulation.mission_control.environment import (
    setup_zone_a, 
    allow_zone_objects, 
    attach_object, 
    detach_object, 
    clear_all_objects,
    EnvironmentVisualizer
)

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

# Target coordinates requested
_qx, _qy, _qz, _qw = 0.6665, 0.2937, -0.5976, 0.3353
_x, _y, _z = 0.55, 0.0313, 0.9292  # X lowered to 0.55 to be closer

PRE_GRASP = _pose(_x, _y, _z + 0.05, _qx, _qy, _qz, _qw)
GRASP     = _pose(_x, _y, _z, _qx, _qy, _qz, _qw)
PRE_PLACE = _pose(0.40, -0.30, _z + 0.05, _qx, _qy, _qz, _qw)
PLACE     = _pose(0.40, -0.30, _z, _qx, _qy, _qz, _qw)

TARGET_OBJECT = 'zone_a_part_flange_0'

def main():
    rclpy.init()
    client = MoveItClient()
    log = client.node.get_logger()
    gripper = GripperController(node=client.node)
    viz = EnvironmentVisualizer(client.node)

    # ── Phase 1: Initialization & Setup ──
    log.info("=== Phase 1: Setup ===")
    setup_zone_a(client)
    allow_zone_objects(client, 'A')
    
    log.info("Clearing planning scene...")
    clear_all_objects(client)
    
    gripper.open('right')
    client.move_to_home()

    try:
        # ── Phase 2: Approach (Pick) ──
        log.info("=== Phase 2: Approach ===")
        log.info("Moving to PRE_GRASP (Free-space OMPL/STOMP)")
        stomp_result = client.move_pose_with_stomp_smoothing(PRE_GRASP)
        if stomp_result.value != "succeeded":
            log.error("Failed to move to PRE_GRASP")
            return

        log.info("Plunging to GRASP (Cartesian Pilz LIN)")
        if client.cartesian_move(GRASP).value != "succeeded":
            log.error("Failed to cartesian move to GRASP")
            return

        # ── Phase 3: Intelligent Grasp & Attach ──
        log.info("=== Phase 3: Intelligent Grasp ===")
        # Use 'ETC' for LUT fallback. Will handle the 1-second effort stability check
        # success = gripper.grasp('right', 'ETC')
        gripper.close('right')
        
        # if not success:
        #     log.error("Grasp assessment failed! Object slipped or missed. Aborting.")
        #     return
            
        log.info(f"Grasp successful. Attaching {TARGET_OBJECT} to MoveIt scene.")
        attach_object(client, TARGET_OBJECT, 'end_effector_r_link', viz)
        
        log.info("Retracting to PRE_GRASP")
        client.cartesian_move(PRE_GRASP)

        # ── Phase 4: Transport & Place ──
        log.info("=== Phase 4: Transport & Place ===")
        log.info("Moving to PRE_PLACE")
        stomp_result = client.move_pose_with_stomp_smoothing(PRE_PLACE)
        if stomp_result.value != "succeeded":
            log.error("Failed to move to PRE_PLACE")
            return
        #result = client.move_to_pose(PRE_PLACE)
        #if result.value != "succeeded":
        #    log.error("Failed to move to PRE_PLACE")
        #    return
            
        log.info("Plunging to PLACE")
        if client.cartesian_move(PLACE).value != "succeeded":
            log.error("Failed to cartesian move to PLACE")
            return
            
        log.info("Releasing object")
        gripper.open('right') # This automatically stops the moving slip-detection thread
        
        # Detach and update RViz to render it at the drop location
        drop_position = (PLACE.position.x, PLACE.position.y, PLACE.position.z)
        detach_object(client, TARGET_OBJECT, 'end_effector_r_link', viz, drop_pos=drop_position)
        
        log.info("Retracting to PRE_PLACE")
        client.cartesian_move(PRE_PLACE)
        
        log.info("=== DEMO COMPLETE ===")

    finally:
        if rclpy.ok():
            log.info("Cleaning up...")
            gripper.open('right')
            client.move_to_home()
            client.destroy()
            rclpy.shutdown()

if __name__ == '__main__':
    main()