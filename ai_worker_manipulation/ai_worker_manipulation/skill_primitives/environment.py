from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient

TABLE_ID       = "table"
TABLE_SIZE     = (0.8, 1.2, 0.70)
TABLE_POSITION = (3.0, 0.0, 0.75)

def setup_environment(client: MoveItClient):
    client.add_collision_box(
        id=TABLE_ID,
        size=TABLE_SIZE,
        position=TABLE_POSITION,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    client.node.get_logger().info('Environment setup complete')
