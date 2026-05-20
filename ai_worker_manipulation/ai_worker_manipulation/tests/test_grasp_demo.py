#!/usr/bin/env python3
"""
test_grasp_demo.py
------------------
왼팔을 목표 joint 위치로 이동 → 파지 시도 최대 3회 → 성공 시 위로 들어올리기

사용법:
  ros2 run ai_worker_manipulation test_grasp_demo
"""

import time
import math
from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm
from ai_worker_manipulation.robot_interface.gripper_controller import GripperController

SIDE        = 'left'
OBJECT_NAME = 'bottle'
MAX_RETRIES = 3


def deg(d: float) -> float:
    """도(degree) → 라디안 변환"""
    return math.radians(d)


# ── Joint 목표값 (도 → 라디안) ────────────────────────────────────────────────
#
#          joint :  현재(도)  →  목표(도)   목표(rad)
#  arm_l_joint1  :   -46     →   -25     = -0.4363
#  arm_l_joint2  :     8     →    21     =  0.3665
#  arm_l_joint3  :   -27     →   -24     = -0.4189
#  arm_l_joint4  :   -58     →  -107     = -1.8675
#  arm_l_joint5  :    22     →    -4     = -0.0698
#  arm_l_joint6  :     3     →   -31     = -0.5411
#  arm_l_joint7  :    18     →    33     =  0.5760

JOINTS_GRASP = [
    deg(-25),   # arm_l_joint1
    deg( 21),   # arm_l_joint2
    deg(-24),   # arm_l_joint3
    deg(-107),  # arm_l_joint4
    deg( -4),   # arm_l_joint5
    deg(-31),   # arm_l_joint6
    deg( 33),   # arm_l_joint7
]

# 파지 성공 후 들어올리는 위치 (joint4만 올려서 팔을 위로)
JOINTS_LIFT = [
    deg(-25),   # arm_l_joint1
    deg( 21),   # arm_l_joint2
    deg(-24),   # arm_l_joint3
    deg(-60),   # arm_l_joint4  ← -107 → -60 으로 올림
    deg( -4),   # arm_l_joint5
    deg(-31),   # arm_l_joint6
    deg( 33),   # arm_l_joint7
]


def main():
    # MoveItClient 가 rclpy.init() + node 담당
    client = MoveItClient()
    log    = client.node.get_logger()

    # GripperController 는 같은 node 재사용
    gc = GripperController(node=client.node)

    log.info('=' * 50)
    log.info('test_grasp_demo 시작')
    log.info('=' * 50)

    # ── Step 0: 홈 포지션 ─────────────────────────────────────────────
    log.info('[Step 0] 홈 포지션')
    client.move_to_home(arm=Arm.LEFT)
    gc.open(SIDE)
    time.sleep(0.5)

    # ── Step 1: 파지 위치로 이동 ──────────────────────────────────────
    log.info('[Step 1] 파지 위치로 이동')
    log.info(f'  목표 joints (deg): -25, 21, -24, -107, -4, -31, 33')
    result = client.move_to_joints(JOINTS_GRASP, arm=Arm.LEFT)
    log.info(f'  결과: {result}')

    if 'succeeded' not in str(result).lower():
        log.error('파지 위치 이동 실패. 중단.')
        client.shutdown()
        return

    time.sleep(0.5)

    # ── Step 2: 파지 시도 최대 3회 ────────────────────────────────────
    log.info(f'[Step 2] 파지 시도 (최대 {MAX_RETRIES}회, {OBJECT_NAME})')
    grasp_success = False

    for attempt in range(1, MAX_RETRIES + 1):
        log.info(f'  [시도 {attempt}/{MAX_RETRIES}]')
        success = gc.grip(SIDE, OBJECT_NAME, stable_duration=1.0)

        if success:
            log.info(f'  ✅ 파지 성공! ({attempt}회)')
            grasp_success = True
            break
        else:
            log.warn(f'  ❌ 파지 실패 ({attempt}회)')
            if attempt < MAX_RETRIES:
                gc.open(SIDE)
                time.sleep(1.0)

    # ── Step 3: 결과 분기 ─────────────────────────────────────────────
    if grasp_success:
        log.info('[Step 3] 파지 성공 → 위로 들어올림')
        result = client.move_to_joints(JOINTS_LIFT, arm=Arm.LEFT)
        log.info(f'  결과: {result}')
        print('grasp success')
        gc.open(SIDE)
    else:
        log.error('[Step 3] 3회 모두 실패 → 홈 복귀')
        gc.open(SIDE)
        client.move_to_home(arm=Arm.LEFT)
        print('grasp failed after 3 attempts')

    client.shutdown()


if __name__ == '__main__':
    main()