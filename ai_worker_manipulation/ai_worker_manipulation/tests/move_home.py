#!/usr/bin/env python3
"""
Move the right arm to the home position (all joints = 0.0).

Usage:
  ros2 run ai_worker_manipulation move_home
"""

import rclpy
from rclpy.node import Node

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, MoveResult


def main():
    rclpy.init()
    node   = Node('move_home')
    client = MoveItClient(node)
    log    = node.get_logger()

    log.info('Moving to home position')
    result = client.move_to_home()
    log.info(f'Result: {result.value}')

    client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
