"""
Lift joint 동작 확인 테스트

Usage:
  ros2 run ai_worker_manipulation test_lift
  ros2 run ai_worker_manipulation test_lift -- --pos -0.15
  ros2 run ai_worker_manipulation test_lift -- --pos 0.0

방법 1: MoveIt (move_group) 경유
방법 2: /lift_controller/follow_joint_trajectory 직접 전송
둘 다 시도하고 결과 출력.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


TARGET_POS = -0.15   # 기본값: 15cm 내리기


class LiftTester(Node):

    def __init__(self, target_pos: float):
        super().__init__('test_lift')
        self._target = target_pos
        self._cb_group = ReentrantCallbackGroup()

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/lift_controller/follow_joint_trajectory',
            callback_group=self._cb_group,
        )

    def run(self):
        log = self.get_logger()
        log.info(f'목표 위치: {self._target:.3f} m  (범위: -0.5 ~ 0.0)')
        log.info('Action server 대기 중...')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            log.error('/lift_controller/follow_joint_trajectory 서버 없음')
            log.error('lift_controller 가 실행 중인지 확인하세요')
            return False

        log.info('Action server 연결됨. 목표 전송...')

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['lift_joint']

        pt = JointTrajectoryPoint()
        pt.positions = [self._target]
        pt.velocities = [0.0]
        pt.time_from_start = Duration(sec=3, nanosec=0)
        traj.points = [pt]
        goal.trajectory = traj

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or not future.result().accepted:
            log.error('Goal rejected or timeout')
            return False

        log.info('Goal accepted. 실행 대기 중...')
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        if not result_future.done():
            log.error('Timeout — 10초 내 완료 안 됨')
            return False

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            log.info(f'SUCCESS — lift → {self._target:.3f} m')
            return True
        else:
            log.error(f'FAILED — error_code={result.error_code}  "{result.error_string}"')
            return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pos', type=float, default=TARGET_POS,
                        help='목표 lift 위치 (m), 범위: -0.5 ~ 0.0')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = LiftTester(args.pos)
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    main()
