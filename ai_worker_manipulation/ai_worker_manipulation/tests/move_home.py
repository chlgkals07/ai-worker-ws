#!/usr/bin/env python3
"""
Move the right arm to the home position (all joints = 0.0).

Usage:
  ros2 run ai_worker_manipulation move_home
"""

import rclpy
from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient


def main():
    rclpy.init()
    client = MoveItClient()
    log = client.node.get_logger()
    log.info('Moving to home position')
    success = client.move_to_home()
    log.info(f'Result: {"SUCCESS" if success else "FAILED"}')
    client.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
