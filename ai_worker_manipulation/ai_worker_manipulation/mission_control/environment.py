# [수정 사항]
# 1. _apply_to_both_arms 제거
#    - moveit2_r, moveit2_l 두 publisher 동시 사용 시 race condition 발생
#    - remove 메시지(l)가 add 메시지(r)보다 늦게 도착해 오브젝트가 삭제되는 버그
# 2. _add_box, _add_cylinder, _remove_obj 인자 변경: moveit2 → client
#    - 내부에서 client.moveit2_r 하나만 사용 (planning scene은 전역 공유)
# 3. 모든 zone setup/remove 함수에서 _apply_to_both_arms 호출 → 헬퍼 직접 호출로 교체

"""
대회 환경 CollisionObject 설정 모듈.
규정집 제12~15조 기준 (2026 Humanoid Challenge).

좌표 기준:
  - 로봇 base_link = (0, 0, 0)
  - +x : 로봇 정면, +y : 로봇 좌측, +z : 위
  - z 공식: 테이블_상면_z + 물체_높이/2
"""

from math import sqrt

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

FRAME_ID = "base_link"

# ══════════════════════════════════════════════
# A구간 — 부품 선별 (규정집 제12조)
# 작업대: 1600×800mm 상판, 높이 800mm
# 혼재 부품 20개(5종), 노란 상자, 파란 트레이
# ══════════════════════════════════════════════

# 작업대 상판 (상면 z = 0.79 + 0.01 = 0.80) — 가로(y) 길게, 세로(x) 짧게
ZONE_A_TABLE_TOP_ID       = "zone_a_table_top"
ZONE_A_TABLE_TOP_SIZE     = (0.8, 1.6, 0.02)   # x=800mm(깊이), y=1600mm(폭) (규정집 제12조)
ZONE_A_TABLE_TOP_POSITION = (0.9, 0.0, 0.79)   # 중심. 앞 x=0.50, 뒤 x=1.30, 좌우 y=±0.80

# 작업대 다리 4개 (상판 모서리 안쪽 25mm)
ZONE_A_TABLE_LEG_SIZE      = (0.05, 0.05, 0.78)
ZONE_A_TABLE_LEG_POSITIONS = [
    (0.525,  0.775, 0.39),   # 앞-좌
    (0.525, -0.775, 0.39),   # 앞-우
    (1.275,  0.775, 0.39),   # 뒤-좌
    (1.275, -0.775, 0.39),   # 뒤-우
]

# 노란 혼재 부품 상자 — 테이블 25% 크기, 우측(y>0) 배치 / collision 없음(시각화 전용)
ZONE_A_PARTS_BOX_ID       = "zone_a_parts_box"
ZONE_A_PARTS_BOX_SIZE     = (0.40, 0.80, 0.06)   # 테이블(0.8×1.6)의 25% ≈ 0.40×0.80, 파란 트레이와 동일 높이
ZONE_A_PARTS_BOX_POSITION = (0.90, 0.40, 0.83)   # 테이블 우측 절반 중앙 (바닥 0.80, 상단 0.86)

# 파란 트레이 — 테이블 25% 크기, 노란 박스 옆(y<0) 배치
ZONE_A_TRAY_ID       = "zone_a_tray"
ZONE_A_TRAY_SIZE     = (0.40, 0.80, 0.06)
ZONE_A_TRAY_POSITION = (0.90, -0.40, 0.83)   # 테이블 좌측 절반 중앙

# 컨테이너 목록 — allow_collisions 등록 필요 (팔이 상자 안으로 진입 가능해야 함)
ZONE_A_CONTAINER_IDS = [ZONE_A_PARTS_BOX_ID, ZONE_A_TRAY_ID]

# 32인치 모니터 (작업대 뒤쪽 엣지, 지령 수신 화면)
ZONE_A_MONITOR_ID       = "zone_a_monitor"
ZONE_A_MONITOR_SIZE     = (0.72, 0.05, 0.42)   # 32인치 = 약 720×420mm, 두께 50mm (규정집 제12조)
ZONE_A_MONITOR_POSITION = (1.12, 0.0, 1.01)    # 0.80 + 0.42/2 + 0.02(베젤)

# 혼재 부품 20개 (5종 × 4개), 노란 상자 안 4×5 격자 (규정집 제12조)
# 노란 상자 x [0.70~1.10], y [0.00~0.80] / 부품 지름 3cm
ZONE_A_PART_RADIUS = 0.015   # 링 형태 근사, 외경 30mm
ZONE_A_PART_HEIGHT = 0.05    # 두께 50mm
ZONE_A_PART_Z      = 0.885   # 노란 상자 상단(0.86) + 0.05/2
ZONE_A_PART_XS     = [0.75, 0.85, 0.95, 1.05]           # 4열, 간격 0.10m (상자 안쪽)
ZONE_A_PART_YS     = [0.10, 0.25, 0.40, 0.55, 0.70]     # 5열, 간격 0.15m (상자 안쪽)
ZONE_A_PART_TYPES  = ['flange', 'gear', 'spacer', 'hex', 'dome']  # 5종

# ══════════════════════════════════════════════
# B구간 — 부품 운반 (규정집 제13조)
# 컨베이어(A): 높이 400~600mm, 선반폭 350mm 이상
# 박스: 우체국 4호 (410×310×280mm)
# 목적지 테이블(B): 높이 800~1,000mm
# ══════════════════════════════════════════════

# 컨베이어 본체 (출발지 A, 로봇 정면 우측)
ZONE_B_CONVEYOR_ID       = "zone_b_conveyor"
ZONE_B_CONVEYOR_SIZE     = (0.6, 0.35, 0.5)    # 높이 500mm 중간값 (규정집 제13조: 400~600mm)
ZONE_B_CONVEYOR_POSITION = (0.8, 0.0, 0.25)    # 목적지 테이블(x=1.8)과 직선 배치

# 컨베이어 선반 상판 (상면 z = 0.25 + 0.25 + 0.01 = 0.51)
ZONE_B_CONVEYOR_TOP_ID       = "zone_b_conveyor_top"
ZONE_B_CONVEYOR_TOP_SIZE     = (0.6, 0.35, 0.02)
ZONE_B_CONVEYOR_TOP_POSITION = (0.8, 0.0, 0.51)

# 우체국 4호 박스 (파지 대상, 컨베이어 상판 위)
ZONE_B_BOX_ID       = "zone_b_box"
ZONE_B_BOX_SIZE     = (0.41, 0.31, 0.28)   # 410×310×280mm (규정집 제13조)
ZONE_B_BOX_POSITION = (0.8, 0.0, 0.66)     # 0.51 + 0.01 + 0.28/2 = 0.66 / 파지 후 attached object로 전환 필요

# 정지선 (컨베이어 앞 바닥, 선반 앞 도착 위치 마킹)
ZONE_B_STOPLINE_ID       = "zone_b_stopline"
ZONE_B_STOPLINE_SIZE     = (0.6, 0.05, 0.005)   # 폭 5cm (규정집 제13조)
ZONE_B_STOPLINE_POSITION = (0.5, 0.0, 0.003)    # 바닥 위 (컨베이어→목적지 직선상)

# 목적지 테이블 본체 (B, 로봇 이동 후 1.5m 앞)
ZONE_B_DEST_TABLE_ID       = "zone_b_dest_table"
ZONE_B_DEST_TABLE_SIZE     = (0.8, 0.6, 0.9)    # 높이 900mm 중간값 (규정집 제13조: 800~1,000mm)
ZONE_B_DEST_TABLE_POSITION = (1.8, 0.0, 0.45)   # 상면 z = 0.90

# 목적지 테이블 박스 하차 위치선
ZONE_B_DROPOFF_MARKER_ID       = "zone_b_dropoff_marker"
ZONE_B_DROPOFF_MARKER_SIZE     = (0.5, 0.4, 0.01)   # 하차 위치선 (규정집 제13조)
ZONE_B_DROPOFF_MARKER_POSITION = (1.8, 0.0, 0.905)  # 테이블 상면

# ══════════════════════════════════════════════
# C구간 — 순차 조립 (규정집 제14조)
# peg 4개: 지름 19/25/32/38mm, 간격 15cm 이상
# 볼트 홀 지름: 50mm
# 완료 버튼: 지름 60mm 이상
# ══════════════════════════════════════════════

# 조립대 몸체 (상면 z = 0.39 + 0.39 = 0.78)
ZONE_C_BENCH_BODY_ID       = "zone_c_bench_body"
ZONE_C_BENCH_BODY_SIZE     = (1.0, 0.6, 0.78)
ZONE_C_BENCH_BODY_POSITION = (0.85, 0.0, 0.39)

# 조립대 상판 (상면 z = 0.79 + 0.01 = 0.80)
ZONE_C_BENCH_TOP_ID       = "zone_c_bench_top"
ZONE_C_BENCH_TOP_SIZE     = (1.0, 0.6, 0.02)
ZONE_C_BENCH_TOP_POSITION = (0.85, 0.0, 0.79)

# peg 4개 (지름 각각 다름, 규정집 붙임1 / 간격 15cm)
ZONE_C_PEG_RADII       = [0.0095, 0.0125, 0.016, 0.019]  # 지름 19/25/32/38mm
ZONE_C_PEG_HEIGHT      = 0.15
ZONE_C_PEG_X           = 0.60
ZONE_C_PEG_Z           = 0.875  # 0.80 + 0.15/2
ZONE_C_PEG_Y_POSITIONS = [-0.225, -0.075, 0.075, 0.225]  # 간격 15cm (규정집 제14조)

# 볼트 4개 (삽입 대상, 각 peg 앞에 분리 배치)
# 볼트 홀 지름 50mm (규정집 제14조)
ZONE_C_BOLT_RADIUS     = 0.025   # 홀 지름 50mm → 반지름 25mm
ZONE_C_BOLT_HEIGHT     = 0.04
ZONE_C_BOLT_X          = 0.40
ZONE_C_BOLT_Z          = 0.825   # 0.80 + 0.04/2 + 보정
ZONE_C_BOLT_Y_POSITIONS = [-0.225, -0.075, 0.075, 0.225]  # peg와 동일 Y

# 조립 완료 버튼 (조립대 앞쪽 중앙)
ZONE_C_BUTTON_ID       = "zone_c_button"
ZONE_C_BUTTON_RADIUS   = 0.03    # 지름 60mm 이상, PUSH버튼 (규정집 제14조)
ZONE_C_BUTTON_HEIGHT   = 0.06
ZONE_C_BUTTON_POSITION = (0.75, 0.0, 0.83)

# 지령 모니터 (조립대 뒤쪽, 삽입 순서 무작위 표시)
ZONE_C_MONITOR_ID       = "zone_c_monitor"
ZONE_C_MONITOR_SIZE     = (0.72, 0.05, 0.42)   # 32인치 (규정집 제14조)
ZONE_C_MONITOR_POSITION = (1.05, 0.0, 1.01)

# ══════════════════════════════════════════════
# D구간 — 휠 장착 체결 (규정집 제15조)
# 타이어 거치 테이블: 로봇 우측, 높이 80cm
# 체결 홀 기둥: 바닥 1m 높이, 지름 32mm
# 공구함: 로봇 좌측, 높이 80cm
# ══════════════════════════════════════════════

# 타이어 거치 테이블 몸체 (로봇 우측, 높이 80cm)
ZONE_D_TIRE_TABLE_ID       = "zone_d_tire_table"
ZONE_D_TIRE_TABLE_SIZE     = (0.5, 0.5, 0.78)    # 규정집 제15조: 바닥~80cm
ZONE_D_TIRE_TABLE_POSITION = (0.7, -0.7, 0.39)   # 우측 (y 음수)

# 타이어 거치 테이블 상판 (상면 z = 0.78 + 0.02 = 0.80)
ZONE_D_TIRE_TABLE_TOP_ID       = "zone_d_tire_table_top"
ZONE_D_TIRE_TABLE_TOP_SIZE     = (0.5, 0.5, 0.02)
ZONE_D_TIRE_TABLE_TOP_POSITION = (0.7, -0.7, 0.79)

# 타이어 (거치 테이블 상판 위, 눕혀서 근사)
ZONE_D_TIRE_ID       = "zone_d_tire"
ZONE_D_TIRE_RADIUS   = 0.15     # 지름 300mm (규정집 제15조)
ZONE_D_TIRE_HEIGHT   = 0.05     # 두께 50mm
ZONE_D_TIRE_POSITION = (0.7, -0.7, 0.825)   # 0.80 + 0.05/2

# 공구함 몸체 (로봇 좌측, 높이 80cm)
ZONE_D_TOOLBOX_ID       = "zone_d_toolbox"
ZONE_D_TOOLBOX_SIZE     = (0.4, 0.4, 0.78)    # 규정집 제15조: 바닥~80cm
ZONE_D_TOOLBOX_POSITION = (0.7, 0.7, 0.39)    # 좌측 (y 양수)

# 공구함 상판 (상면 z = 0.80)
ZONE_D_TOOLBOX_TOP_ID       = "zone_d_toolbox_top"
ZONE_D_TOOLBOX_TOP_SIZE     = (0.4, 0.4, 0.02)
ZONE_D_TOOLBOX_TOP_POSITION = (0.7, 0.7, 0.79)

# 체결 볼트 (공구함 상판 위)
ZONE_D_BOLT_ID       = "zone_d_bolt"
ZONE_D_BOLT_RADIUS   = 0.03     # 지름 60mm (규정집 제15조)
ZONE_D_BOLT_HEIGHT   = 0.06     # 길이 60mm
ZONE_D_BOLT_POSITION = (0.7, 0.55, 0.833)   # 0.80 + 0.06/2

# 전동 드릴 (공구함 상판 위, 볼트 옆)
ZONE_D_DRILL_ID       = "zone_d_drill"
ZONE_D_DRILL_SIZE     = (0.25, 0.08, 0.12)
ZONE_D_DRILL_POSITION = (0.7, 0.78, 0.86)   # 공구함 상판 중앙 (볼트 옆)

# 타이어 체결 홀 기둥 (로봇 정면, 바닥에서 1m)
ZONE_D_HOLE_POST_ID       = "zone_d_hole_post"
ZONE_D_HOLE_POST_RADIUS   = 0.016    # 지름 32mm 결합부 (규정집 제15조)
ZONE_D_HOLE_POST_HEIGHT   = 1.0      # 바닥에서 1m
ZONE_D_HOLE_POST_POSITION = (1.2, 0.0, 0.5)   # 기둥 중심 z=0.5 (0~1m) / 로봇에서 충분히 이격

# 휠 허브 (기둥 끝, 높이 1m 지점)
ZONE_D_WHEEL_HUB_ID       = "zone_d_wheel_hub"
ZONE_D_WHEEL_HUB_RADIUS   = 0.019   # 스텐파이프 38mm (규정집 제15조)
ZONE_D_WHEEL_HUB_HEIGHT   = 0.05
ZONE_D_WHEEL_HUB_POSITION = (1.2, 0.0, 1.0)   # 기둥 끝

# 기둥 바닥 고정부
ZONE_D_HOLE_BASE_ID       = "zone_d_hole_base"
ZONE_D_HOLE_BASE_SIZE     = (0.15, 0.15, 0.05)
ZONE_D_HOLE_BASE_POSITION = (1.2, 0.0, 0.025)  # 바닥 위


# ══════════════════════════════════════════════
# 공통 헬퍼 함수 (moveit2_r 단일 publisher 사용)
# ══════════════════════════════════════════════

def _add_box(client: MoveItClient, obj_id: str, size: tuple, position: tuple):
    client.moveit2_r.add_collision_box(
        id=obj_id,
        size=size,
        position=position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _add_cylinder(client: MoveItClient, obj_id: str, radius: float, height: float, position: tuple):
    client.moveit2_r.add_collision_cylinder(
        id=obj_id,
        radius=radius,
        height=height,
        position=position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _remove_obj(client: MoveItClient, obj_id: str):
    client.moveit2_r.remove_collision_object(id=obj_id)


def _zone_a_part_entries():
    positions = [(x, y, ZONE_A_PART_Z) for x in ZONE_A_PART_XS for y in ZONE_A_PART_YS]
    entries = []
    for i, part_type in enumerate(ZONE_A_PART_TYPES):
        for j in range(4):
            entries.append((f"zone_a_part_{part_type}_{j}", positions[i * 4 + j]))
    return entries


# ══════════════════════════════════════════════
# A구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_a(client: MoveItClient):
    _remove_obj(client, ZONE_A_MONITOR_ID)  # 이전 세션 레거시 정리
    _add_box(client, ZONE_A_TABLE_TOP_ID, ZONE_A_TABLE_TOP_SIZE, ZONE_A_TABLE_TOP_POSITION)
    for i, pos in enumerate(ZONE_A_TABLE_LEG_POSITIONS):
        _add_box(client, f"zone_a_table_leg_{i}", ZONE_A_TABLE_LEG_SIZE, pos)
    _add_box(client, ZONE_A_PARTS_BOX_ID, ZONE_A_PARTS_BOX_SIZE, ZONE_A_PARTS_BOX_POSITION)
    _add_box(client, ZONE_A_TRAY_ID, ZONE_A_TRAY_SIZE, ZONE_A_TRAY_POSITION)
    for part_id, pos in _zone_a_part_entries():
        _add_cylinder(client, part_id, ZONE_A_PART_RADIUS, ZONE_A_PART_HEIGHT, pos)


def remove_zone_a(client: MoveItClient):
    _remove_obj(client, ZONE_A_MONITOR_ID)  # 레거시 포함 제거
    _remove_obj(client, ZONE_A_TABLE_TOP_ID)
    for i in range(len(ZONE_A_TABLE_LEG_POSITIONS)):
        _remove_obj(client, f"zone_a_table_leg_{i}")
    _remove_obj(client, ZONE_A_PARTS_BOX_ID)
    _remove_obj(client, ZONE_A_TRAY_ID)
    for part_id, _ in _zone_a_part_entries():
        _remove_obj(client, part_id)


# ══════════════════════════════════════════════
# B구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_b(client: MoveItClient):
    _add_box(client, ZONE_B_CONVEYOR_ID, ZONE_B_CONVEYOR_SIZE, ZONE_B_CONVEYOR_POSITION)
    _add_box(client, ZONE_B_CONVEYOR_TOP_ID, ZONE_B_CONVEYOR_TOP_SIZE, ZONE_B_CONVEYOR_TOP_POSITION)
    _add_box(client, ZONE_B_BOX_ID, ZONE_B_BOX_SIZE, ZONE_B_BOX_POSITION)
    _add_box(client, ZONE_B_STOPLINE_ID, ZONE_B_STOPLINE_SIZE, ZONE_B_STOPLINE_POSITION)
    _add_box(client, ZONE_B_DEST_TABLE_ID, ZONE_B_DEST_TABLE_SIZE, ZONE_B_DEST_TABLE_POSITION)
    _add_box(client, ZONE_B_DROPOFF_MARKER_ID, ZONE_B_DROPOFF_MARKER_SIZE, ZONE_B_DROPOFF_MARKER_POSITION)


def remove_zone_b(client: MoveItClient):
    for obj_id in (
        ZONE_B_CONVEYOR_ID,
        ZONE_B_CONVEYOR_TOP_ID,
        ZONE_B_BOX_ID,
        ZONE_B_STOPLINE_ID,
        ZONE_B_DEST_TABLE_ID,
        ZONE_B_DROPOFF_MARKER_ID,
    ):
        _remove_obj(client, obj_id)


# ══════════════════════════════════════════════
# C구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_c(client: MoveItClient):
    _remove_obj(client, ZONE_C_MONITOR_ID)  # 이전 세션 레거시 정리
    _add_box(client, ZONE_C_BENCH_BODY_ID, ZONE_C_BENCH_BODY_SIZE, ZONE_C_BENCH_BODY_POSITION)
    _add_box(client, ZONE_C_BENCH_TOP_ID, ZONE_C_BENCH_TOP_SIZE, ZONE_C_BENCH_TOP_POSITION)
    for i, (r, y) in enumerate(zip(ZONE_C_PEG_RADII, ZONE_C_PEG_Y_POSITIONS)):
        _add_cylinder(client, f"zone_c_peg_{i}", r, ZONE_C_PEG_HEIGHT,
                      (ZONE_C_PEG_X, y, ZONE_C_PEG_Z))
    for i, y in enumerate(ZONE_C_BOLT_Y_POSITIONS):
        _add_cylinder(client, f"zone_c_bolt_{i}", ZONE_C_BOLT_RADIUS, ZONE_C_BOLT_HEIGHT,
                      (ZONE_C_BOLT_X, y, ZONE_C_BOLT_Z))
    _add_cylinder(client, ZONE_C_BUTTON_ID, ZONE_C_BUTTON_RADIUS, ZONE_C_BUTTON_HEIGHT,
                  ZONE_C_BUTTON_POSITION)


def remove_zone_c(client: MoveItClient):
    for obj_id in (
        ZONE_C_BENCH_BODY_ID,
        ZONE_C_BENCH_TOP_ID,
        ZONE_C_BUTTON_ID,
        ZONE_C_MONITOR_ID,  # 레거시 포함 제거
    ):
        _remove_obj(client, obj_id)
    for i in range(len(ZONE_C_PEG_Y_POSITIONS)):
        _remove_obj(client, f"zone_c_peg_{i}")
    for i in range(len(ZONE_C_BOLT_Y_POSITIONS)):
        _remove_obj(client, f"zone_c_bolt_{i}")


# ══════════════════════════════════════════════
# D구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_d(client: MoveItClient):
    _add_box(client, ZONE_D_TIRE_TABLE_ID, ZONE_D_TIRE_TABLE_SIZE, ZONE_D_TIRE_TABLE_POSITION)
    _add_box(client, ZONE_D_TIRE_TABLE_TOP_ID, ZONE_D_TIRE_TABLE_TOP_SIZE, ZONE_D_TIRE_TABLE_TOP_POSITION)
    _add_cylinder(client, ZONE_D_TIRE_ID, ZONE_D_TIRE_RADIUS, ZONE_D_TIRE_HEIGHT,
                  ZONE_D_TIRE_POSITION)
    _add_box(client, ZONE_D_TOOLBOX_ID, ZONE_D_TOOLBOX_SIZE, ZONE_D_TOOLBOX_POSITION)
    _add_box(client, ZONE_D_TOOLBOX_TOP_ID, ZONE_D_TOOLBOX_TOP_SIZE, ZONE_D_TOOLBOX_TOP_POSITION)
    _add_cylinder(client, ZONE_D_BOLT_ID, ZONE_D_BOLT_RADIUS, ZONE_D_BOLT_HEIGHT,
                  ZONE_D_BOLT_POSITION)
    _add_box(client, ZONE_D_DRILL_ID, ZONE_D_DRILL_SIZE, ZONE_D_DRILL_POSITION)
    _add_cylinder(client, ZONE_D_HOLE_POST_ID, ZONE_D_HOLE_POST_RADIUS, ZONE_D_HOLE_POST_HEIGHT,
                  ZONE_D_HOLE_POST_POSITION)
    _add_cylinder(client, ZONE_D_WHEEL_HUB_ID, ZONE_D_WHEEL_HUB_RADIUS, ZONE_D_WHEEL_HUB_HEIGHT,
                  ZONE_D_WHEEL_HUB_POSITION)
    _add_box(client, ZONE_D_HOLE_BASE_ID, ZONE_D_HOLE_BASE_SIZE, ZONE_D_HOLE_BASE_POSITION)


def remove_zone_d(client: MoveItClient):
    for obj_id in (
        ZONE_D_TIRE_TABLE_ID,
        ZONE_D_TIRE_TABLE_TOP_ID,
        ZONE_D_TIRE_ID,
        ZONE_D_TOOLBOX_ID,
        ZONE_D_TOOLBOX_TOP_ID,
        ZONE_D_BOLT_ID,
        ZONE_D_DRILL_ID,
        ZONE_D_HOLE_POST_ID,
        ZONE_D_WHEEL_HUB_ID,
        ZONE_D_HOLE_BASE_ID,
    ):
        _remove_obj(client, obj_id)


# ══════════════════════════════════════════════
# 전체 환경 설정 / 해제
# ══════════════════════════════════════════════

def setup_environment(client: MoveItClient):
    setup_zone_a(client)
    setup_zone_b(client)
    setup_zone_c(client)
    setup_zone_d(client)
    client.moveit2_r._node.get_logger().info(
        "환경 설정 완료: A/B/C/D 구간 collision objects 등록됨"
    )


def remove_environment(client: MoveItClient):
    remove_zone_a(client)
    remove_zone_b(client)
    remove_zone_c(client)
    remove_zone_d(client)
    client.moveit2_r._node.get_logger().info(
        "환경 해제 완료: 모든 collision objects 제거됨"
    )


def clear_all_objects(client: MoveItClient) -> None:
    """
    Planning Scene의 모든 collision object 제거.
    구간 전환 전 또는 완전 초기화 시 호출.
    ABCD 구간 이름과 무관하게 씬을 빈 상태로 만든다.
    """
    client.moveit2_r.clear_all_collision_objects()
    client.moveit2_r._node.get_logger().info(
        "씬 초기화 완료: Planning Scene의 모든 collision objects 제거됨"
    )


# ══════════════════════════════════════════════
# 시각화 레이어 (Phase 1)
# MarkerArray로 RViz 색상 시각화 — collision object와 독립적으로 동작
# ══════════════════════════════════════════════

# 규정집 기반 색상 상수 (r, g, b, a)
COLOR_TABLE      = (0.76, 0.60, 0.42, 0.85)  # 나무색, 테이블 상판
COLOR_WHITE      = (0.90, 0.90, 0.90, 0.85)  # 흰색, 테이블 다리/벤치
COLOR_YELLOW     = (1.00, 0.85, 0.00, 0.90)  # 노란색, A구간 부품 박스
COLOR_BLUE       = (0.10, 0.40, 0.90, 0.90)  # 파란색, A구간 트레이
COLOR_MONITOR    = (0.15, 0.15, 0.15, 0.90)  # 검정, 모니터
COLOR_PART_MIX   = (0.60, 0.60, 0.65, 0.90)  # 회색, A구간 혼재 부품
COLOR_CONVEYOR   = (0.40, 0.25, 0.10, 0.85)  # 갈색, 컨베이어
COLOR_BOX        = (0.82, 0.71, 0.55, 0.90)  # 박스색, 우체국 박스
COLOR_STOPLINE   = (1.00, 1.00, 0.00, 1.00)  # 노란색, 정지선
COLOR_DEST_TABLE = (0.50, 0.50, 0.50, 0.85)  # 회색, 목적지 테이블
COLOR_STEEL      = (0.65, 0.65, 0.70, 0.90)  # 금속색, peg/볼트/드릴
COLOR_GREEN      = (0.10, 0.80, 0.20, 1.00)  # 초록, C구간 완료 버튼
COLOR_TIRE       = (0.15, 0.15, 0.15, 0.90)  # 검정, 타이어
COLOR_TOOLBOX    = (0.30, 0.30, 0.35, 0.85)  # 어두운 회색, 공구함


def _make_box_marker(marker_id, ns, size, position, color, frame_id="base_link") -> Marker:
    """box 마커 생성."""
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(position[0])
    m.pose.position.y = float(position[1])
    m.pose.position.z = float(position[2])
    m.pose.orientation.w = 1.0
    m.scale = Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))
    m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
    return m


def _make_cylinder_marker(marker_id, ns, radius, height, position, color, frame_id="base_link") -> Marker:
    """cylinder 마커 생성."""
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = float(position[0])
    m.pose.position.y = float(position[1])
    m.pose.position.z = float(position[2])
    m.pose.orientation.w = 1.0
    m.scale = Vector3(x=float(radius * 2), y=float(radius * 2), z=float(height))
    m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
    return m


def get_zone_a_markers() -> MarkerArray:
    """A구간 마커 생성 (퍼블리시 없이 MarkerArray 반환)."""
    ma = MarkerArray()
    mid = 0
    ns = "zone_a"

    # 작업대 상판
    ma.markers.append(_make_box_marker(mid, ns, ZONE_A_TABLE_TOP_SIZE, ZONE_A_TABLE_TOP_POSITION, COLOR_TABLE))
    mid += 1
    # 작업대 다리 4개
    for pos in ZONE_A_TABLE_LEG_POSITIONS:
        ma.markers.append(_make_box_marker(mid, ns, ZONE_A_TABLE_LEG_SIZE, pos, COLOR_WHITE))
        mid += 1
    # 노란 부품 상자
    ma.markers.append(_make_box_marker(mid, ns, ZONE_A_PARTS_BOX_SIZE, ZONE_A_PARTS_BOX_POSITION, COLOR_YELLOW))
    mid += 1
    # 파란 트레이
    ma.markers.append(_make_box_marker(mid, ns, ZONE_A_TRAY_SIZE, ZONE_A_TRAY_POSITION, COLOR_BLUE))
    mid += 1
    # 혼재 부품 20개
    for _, pos in _zone_a_part_entries():
        ma.markers.append(_make_cylinder_marker(mid, ns, ZONE_A_PART_RADIUS, ZONE_A_PART_HEIGHT, pos, COLOR_PART_MIX))
        mid += 1

    return ma


def get_zone_b_markers() -> MarkerArray:
    """B구간 마커 생성 (퍼블리시 없이 MarkerArray 반환)."""
    ma = MarkerArray()
    mid = 0
    ns = "zone_b"

    ma.markers.append(_make_box_marker(mid, ns, ZONE_B_CONVEYOR_SIZE, ZONE_B_CONVEYOR_POSITION, COLOR_CONVEYOR))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_B_CONVEYOR_TOP_SIZE, ZONE_B_CONVEYOR_TOP_POSITION, COLOR_CONVEYOR))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_B_BOX_SIZE, ZONE_B_BOX_POSITION, COLOR_BOX))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_B_STOPLINE_SIZE, ZONE_B_STOPLINE_POSITION, COLOR_STOPLINE))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_B_DEST_TABLE_SIZE, ZONE_B_DEST_TABLE_POSITION, COLOR_DEST_TABLE))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_B_DROPOFF_MARKER_SIZE, ZONE_B_DROPOFF_MARKER_POSITION, COLOR_STOPLINE))
    mid += 1

    return ma


def get_zone_c_markers() -> MarkerArray:
    """C구간 마커 생성 (퍼블리시 없이 MarkerArray 반환)."""
    ma = MarkerArray()
    mid = 0
    ns = "zone_c"

    # 조립대 몸체/상판
    ma.markers.append(_make_box_marker(mid, ns, ZONE_C_BENCH_BODY_SIZE, ZONE_C_BENCH_BODY_POSITION, COLOR_WHITE))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_C_BENCH_TOP_SIZE, ZONE_C_BENCH_TOP_POSITION, COLOR_TABLE))
    mid += 1
    # peg 4개
    for r, y in zip(ZONE_C_PEG_RADII, ZONE_C_PEG_Y_POSITIONS):
        ma.markers.append(_make_cylinder_marker(mid, ns, r, ZONE_C_PEG_HEIGHT,
                                                (ZONE_C_PEG_X, y, ZONE_C_PEG_Z), COLOR_STEEL))
        mid += 1
    # 볼트 4개
    for y in ZONE_C_BOLT_Y_POSITIONS:
        ma.markers.append(_make_cylinder_marker(mid, ns, ZONE_C_BOLT_RADIUS, ZONE_C_BOLT_HEIGHT,
                                                (ZONE_C_BOLT_X, y, ZONE_C_BOLT_Z), COLOR_STEEL))
        mid += 1
    # 완료 버튼
    ma.markers.append(_make_cylinder_marker(mid, ns, ZONE_C_BUTTON_RADIUS, ZONE_C_BUTTON_HEIGHT,
                                            ZONE_C_BUTTON_POSITION, COLOR_GREEN))
    mid += 1

    return ma


def get_zone_d_markers() -> MarkerArray:
    """D구간 마커 생성 (퍼블리시 없이 MarkerArray 반환)."""
    ma = MarkerArray()
    mid = 0
    ns = "zone_d"

    # 타이어 거치 테이블
    ma.markers.append(_make_box_marker(mid, ns, ZONE_D_TIRE_TABLE_SIZE, ZONE_D_TIRE_TABLE_POSITION, COLOR_WHITE))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_D_TIRE_TABLE_TOP_SIZE, ZONE_D_TIRE_TABLE_TOP_POSITION, COLOR_TABLE))
    mid += 1
    # 타이어
    ma.markers.append(_make_cylinder_marker(mid, ns, ZONE_D_TIRE_RADIUS, ZONE_D_TIRE_HEIGHT,
                                            ZONE_D_TIRE_POSITION, COLOR_TIRE))
    mid += 1
    # 공구함
    ma.markers.append(_make_box_marker(mid, ns, ZONE_D_TOOLBOX_SIZE, ZONE_D_TOOLBOX_POSITION, COLOR_TOOLBOX))
    mid += 1
    ma.markers.append(_make_box_marker(mid, ns, ZONE_D_TOOLBOX_TOP_SIZE, ZONE_D_TOOLBOX_TOP_POSITION, COLOR_TABLE))
    mid += 1
    # 체결 볼트
    ma.markers.append(_make_cylinder_marker(mid, ns, ZONE_D_BOLT_RADIUS, ZONE_D_BOLT_HEIGHT,
                                            ZONE_D_BOLT_POSITION, COLOR_STEEL))
    mid += 1
    # 전동 드릴
    ma.markers.append(_make_box_marker(mid, ns, ZONE_D_DRILL_SIZE, ZONE_D_DRILL_POSITION, COLOR_STEEL))
    mid += 1
    # 체결 홀 기둥
    ma.markers.append(_make_cylinder_marker(mid, ns, ZONE_D_HOLE_POST_RADIUS, ZONE_D_HOLE_POST_HEIGHT,
                                            ZONE_D_HOLE_POST_POSITION, COLOR_STEEL))
    mid += 1
    # 휠 허브
    ma.markers.append(_make_cylinder_marker(mid, ns, ZONE_D_WHEEL_HUB_RADIUS, ZONE_D_WHEEL_HUB_HEIGHT,
                                            ZONE_D_WHEEL_HUB_POSITION, COLOR_STEEL))
    mid += 1
    # 기둥 바닥 고정부
    ma.markers.append(_make_box_marker(mid, ns, ZONE_D_HOLE_BASE_SIZE, ZONE_D_HOLE_BASE_POSITION, COLOR_STEEL))
    mid += 1

    return ma


def _build_object_marker_map() -> dict:
    """
    object_id → Marker (원래 색상·크기·위치 포함).
    get_zone_X_markers() 와 동일한 mid 할당 순서를 유지해야 함.
    highlight_object / restore_object 에서 이 dict 참조.
    """
    mapping = {}

    # ── Zone A (모니터 없음) ──────────────────────────────────────────────
    ns, mid = "zone_a", 0
    mapping[ZONE_A_TABLE_TOP_ID] = _make_box_marker(
        mid, ns, ZONE_A_TABLE_TOP_SIZE, ZONE_A_TABLE_TOP_POSITION, COLOR_TABLE); mid += 1
    for i, pos in enumerate(ZONE_A_TABLE_LEG_POSITIONS):
        mapping[f"zone_a_table_leg_{i}"] = _make_box_marker(
            mid, ns, ZONE_A_TABLE_LEG_SIZE, pos, COLOR_WHITE); mid += 1
    mapping[ZONE_A_PARTS_BOX_ID] = _make_box_marker(
        mid, ns, ZONE_A_PARTS_BOX_SIZE, ZONE_A_PARTS_BOX_POSITION, COLOR_YELLOW); mid += 1
    mapping[ZONE_A_TRAY_ID] = _make_box_marker(
        mid, ns, ZONE_A_TRAY_SIZE, ZONE_A_TRAY_POSITION, COLOR_BLUE); mid += 1
    for part_id, pos in _zone_a_part_entries():
        mapping[part_id] = _make_cylinder_marker(
            mid, ns, ZONE_A_PART_RADIUS, ZONE_A_PART_HEIGHT, pos, COLOR_PART_MIX); mid += 1

    # ── Zone B ───────────────────────────────────────────────────────────
    ns, mid = "zone_b", 0
    mapping[ZONE_B_CONVEYOR_ID] = _make_box_marker(
        mid, ns, ZONE_B_CONVEYOR_SIZE, ZONE_B_CONVEYOR_POSITION, COLOR_CONVEYOR); mid += 1
    mapping[ZONE_B_CONVEYOR_TOP_ID] = _make_box_marker(
        mid, ns, ZONE_B_CONVEYOR_TOP_SIZE, ZONE_B_CONVEYOR_TOP_POSITION, COLOR_CONVEYOR); mid += 1
    mapping[ZONE_B_BOX_ID] = _make_box_marker(
        mid, ns, ZONE_B_BOX_SIZE, ZONE_B_BOX_POSITION, COLOR_BOX); mid += 1
    mapping[ZONE_B_STOPLINE_ID] = _make_box_marker(
        mid, ns, ZONE_B_STOPLINE_SIZE, ZONE_B_STOPLINE_POSITION, COLOR_STOPLINE); mid += 1
    mapping[ZONE_B_DEST_TABLE_ID] = _make_box_marker(
        mid, ns, ZONE_B_DEST_TABLE_SIZE, ZONE_B_DEST_TABLE_POSITION, COLOR_DEST_TABLE); mid += 1
    mapping[ZONE_B_DROPOFF_MARKER_ID] = _make_box_marker(
        mid, ns, ZONE_B_DROPOFF_MARKER_SIZE, ZONE_B_DROPOFF_MARKER_POSITION, COLOR_STOPLINE); mid += 1

    # ── Zone C ───────────────────────────────────────────────────────────
    ns, mid = "zone_c", 0
    mapping[ZONE_C_BENCH_BODY_ID] = _make_box_marker(
        mid, ns, ZONE_C_BENCH_BODY_SIZE, ZONE_C_BENCH_BODY_POSITION, COLOR_WHITE); mid += 1
    mapping[ZONE_C_BENCH_TOP_ID] = _make_box_marker(
        mid, ns, ZONE_C_BENCH_TOP_SIZE, ZONE_C_BENCH_TOP_POSITION, COLOR_TABLE); mid += 1
    for i, (r, y) in enumerate(zip(ZONE_C_PEG_RADII, ZONE_C_PEG_Y_POSITIONS)):
        mapping[f"zone_c_peg_{i}"] = _make_cylinder_marker(
            mid, ns, r, ZONE_C_PEG_HEIGHT, (ZONE_C_PEG_X, y, ZONE_C_PEG_Z), COLOR_STEEL); mid += 1
    for i, y in enumerate(ZONE_C_BOLT_Y_POSITIONS):
        mapping[f"zone_c_bolt_{i}"] = _make_cylinder_marker(
            mid, ns, ZONE_C_BOLT_RADIUS, ZONE_C_BOLT_HEIGHT,
            (ZONE_C_BOLT_X, y, ZONE_C_BOLT_Z), COLOR_STEEL); mid += 1
    mapping[ZONE_C_BUTTON_ID] = _make_cylinder_marker(
        mid, ns, ZONE_C_BUTTON_RADIUS, ZONE_C_BUTTON_HEIGHT, ZONE_C_BUTTON_POSITION, COLOR_GREEN); mid += 1

    # ── Zone D ───────────────────────────────────────────────────────────
    ns, mid = "zone_d", 0
    mapping[ZONE_D_TIRE_TABLE_ID] = _make_box_marker(
        mid, ns, ZONE_D_TIRE_TABLE_SIZE, ZONE_D_TIRE_TABLE_POSITION, COLOR_WHITE); mid += 1
    mapping[ZONE_D_TIRE_TABLE_TOP_ID] = _make_box_marker(
        mid, ns, ZONE_D_TIRE_TABLE_TOP_SIZE, ZONE_D_TIRE_TABLE_TOP_POSITION, COLOR_TABLE); mid += 1
    mapping[ZONE_D_TIRE_ID] = _make_cylinder_marker(
        mid, ns, ZONE_D_TIRE_RADIUS, ZONE_D_TIRE_HEIGHT, ZONE_D_TIRE_POSITION, COLOR_TIRE); mid += 1
    mapping[ZONE_D_TOOLBOX_ID] = _make_box_marker(
        mid, ns, ZONE_D_TOOLBOX_SIZE, ZONE_D_TOOLBOX_POSITION, COLOR_TOOLBOX); mid += 1
    mapping[ZONE_D_TOOLBOX_TOP_ID] = _make_box_marker(
        mid, ns, ZONE_D_TOOLBOX_TOP_SIZE, ZONE_D_TOOLBOX_TOP_POSITION, COLOR_TABLE); mid += 1
    mapping[ZONE_D_BOLT_ID] = _make_cylinder_marker(
        mid, ns, ZONE_D_BOLT_RADIUS, ZONE_D_BOLT_HEIGHT, ZONE_D_BOLT_POSITION, COLOR_STEEL); mid += 1
    mapping[ZONE_D_DRILL_ID] = _make_box_marker(
        mid, ns, ZONE_D_DRILL_SIZE, ZONE_D_DRILL_POSITION, COLOR_STEEL); mid += 1
    mapping[ZONE_D_HOLE_POST_ID] = _make_cylinder_marker(
        mid, ns, ZONE_D_HOLE_POST_RADIUS, ZONE_D_HOLE_POST_HEIGHT,
        ZONE_D_HOLE_POST_POSITION, COLOR_STEEL); mid += 1
    mapping[ZONE_D_WHEEL_HUB_ID] = _make_cylinder_marker(
        mid, ns, ZONE_D_WHEEL_HUB_RADIUS, ZONE_D_WHEEL_HUB_HEIGHT,
        ZONE_D_WHEEL_HUB_POSITION, COLOR_STEEL); mid += 1
    mapping[ZONE_D_HOLE_BASE_ID] = _make_box_marker(
        mid, ns, ZONE_D_HOLE_BASE_SIZE, ZONE_D_HOLE_BASE_POSITION, COLOR_STEEL); mid += 1

    return mapping


_OBJECT_MARKER_MAP: dict = _build_object_marker_map()


class EnvironmentVisualizer:
    """
    /competition_markers 토픽으로 MarkerArray를 퍼블리시하는 시각화 클래스.
    transient_local QoS로 RViz 구독 시 즉시 표시됨.
    """

    _ZONE_FN = {
        'A': get_zone_a_markers,
        'B': get_zone_b_markers,
        'C': get_zone_c_markers,
        'D': get_zone_d_markers,
    }

    def __init__(self, node):
        # RViz MarkerArray 구독자는 volatile — transient_local이면 통신 자체 불가
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = node.create_publisher(MarkerArray, '/competition_markers', qos)
        self._highlights: dict = {}   # object_id → (r, g, b, a)
        self._floating: set = set()   # attach된 물체: publish_zone이 건너뜀, move_highlight로만 위치 갱신
        self._overrides: dict = {}    # object_id → (x, y, z): drop_object 낙하 위치 (publish_zone이 이 위치 사용)

    def publish_zone(self, zone: str):
        """zone: 'A'/'B'/'C'/'D'/'ALL' — 해당 구간 마커 퍼블리시 (highlight 반영).
        highlight된(=attached) 물체는 스킵 — move_highlight가 단독으로 위치 관리."""
        if zone == 'ALL':
            ma = MarkerArray()
            for fn in self._ZONE_FN.values():
                ma.markers.extend(fn().markers)
        elif zone in self._ZONE_FN:
            ma = self._ZONE_FN[zone]()
        else:
            return
        filtered = []
        for m in ma.markers:
            key = next(
                (oid for oid, mk in _OBJECT_MARKER_MAP.items()
                 if mk.ns == m.ns and mk.id == m.id),
                None,
            )
            if key and key in self._floating:
                continue   # attach된 물체 — move_highlight가 위치·색상 담당
            if key and key in self._overrides:
                ox, oy, oz = self._overrides[key]
                m.pose.position.x = float(ox)
                m.pose.position.y = float(oy)
                m.pose.position.z = float(oz)
            if key and key in self._highlights:
                c = self._highlights[key]
                m.color = ColorRGBA(r=float(c[0]), g=float(c[1]),
                                    b=float(c[2]), a=float(c[3]))
            filtered.append(m)
        ma.markers = filtered
        self._pub.publish(ma)

    def clear(self):
        """DELETEALL 마커로 /competition_markers 전체 삭제 + 내부 highlight 상태 초기화."""
        self._highlights.clear()
        self._floating.clear()
        self._overrides.clear()
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self._pub.publish(ma)

    def highlight_object(self, object_id: str, color: tuple) -> None:
        """object_id 마커를 지정 색상으로 변경. color: (r, g, b, a)"""
        self._highlights[object_id] = color
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        m = Marker()
        m.header.frame_id = orig.header.frame_id
        m.ns = orig.ns
        m.id = orig.id
        m.type = orig.type
        m.action = Marker.ADD
        if object_id in self._overrides:
            ox, oy, oz = self._overrides[object_id]
            m.pose = Pose(
                position=Point(x=float(ox), y=float(oy), z=float(oz)),
                orientation=orig.pose.orientation,
            )
        else:
            m.pose = orig.pose
        m.scale = orig.scale
        m.color = ColorRGBA(
            r=float(color[0]), g=float(color[1]),
            b=float(color[2]), a=float(color[3]),
        )
        ma = MarkerArray()
        ma.markers.append(m)
        self._pub.publish(ma)

    def set_floating(self, object_id: str) -> None:
        """attach된 물체를 floating으로 등록 — publish_zone이 건너뜀."""
        self._floating.add(object_id)
        self._overrides.pop(object_id, None)

    def restore_object(self, object_id: str) -> None:
        """object_id 마커를 원래 위치·색상으로 복구."""
        self._highlights.pop(object_id, None)
        self._floating.discard(object_id)
        self._overrides.pop(object_id, None)
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        ma = MarkerArray()
        ma.markers.append(orig)
        self._pub.publish(ma)

    def drop_object(self, object_id: str, x: float, y: float, z: float = 0.01) -> None:
        """detach 시 원래 위치로 복귀 대신 지정 위치에 떨어뜨림. publish_zone도 이 위치 사용."""
        self._highlights.pop(object_id, None)
        self._floating.discard(object_id)
        self._overrides[object_id] = (x, y, z)
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        m = Marker()
        m.header.frame_id = orig.header.frame_id
        m.ns = orig.ns
        m.id = orig.id
        m.type = orig.type
        m.action = Marker.ADD
        m.pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=orig.pose.orientation,
        )
        m.scale = orig.scale
        m.color = orig.color
        ma = MarkerArray()
        ma.markers.append(m)
        self._pub.publish(ma)

    def move_highlight(self, object_id: str, x: float, y: float, z: float) -> None:
        """attach된 object 마커를 현재 위치로 이동하여 퍼블리시 (하이라이트 색상 유지)."""
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        color = self._highlights.get(object_id)
        if color is None:
            return
        m = Marker()
        m.header.frame_id = orig.header.frame_id
        m.ns = orig.ns
        m.id = orig.id
        m.type = orig.type
        m.action = Marker.ADD
        m.pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=orig.pose.orientation,
        )
        m.scale = orig.scale
        m.color = ColorRGBA(
            r=float(color[0]), g=float(color[1]),
            b=float(color[2]), a=float(color[3]),
        )
        ma = MarkerArray()
        ma.markers.append(m)
        self._pub.publish(ma)


# ══════════════════════════════════════════════
# 파지 가능 물체 목록 (Phase 2 attach/detach 공용)
# ══════════════════════════════════════════════

GRASPABLE_OBJECTS = {
    'A': [
        'zone_a_part_flange_0', 'zone_a_part_flange_1', 'zone_a_part_flange_2', 'zone_a_part_flange_3',
        'zone_a_part_gear_0',   'zone_a_part_gear_1',   'zone_a_part_gear_2',   'zone_a_part_gear_3',
        'zone_a_part_spacer_0', 'zone_a_part_spacer_1', 'zone_a_part_spacer_2', 'zone_a_part_spacer_3',
        'zone_a_part_hex_0',    'zone_a_part_hex_1',    'zone_a_part_hex_2',    'zone_a_part_hex_3',
        'zone_a_part_dome_0',   'zone_a_part_dome_1',   'zone_a_part_dome_2',   'zone_a_part_dome_3',
    ],
    'B': ['zone_b_box'],
    'C': ['zone_c_bolt_0', 'zone_c_bolt_1', 'zone_c_bolt_2', 'zone_c_bolt_3'],
    'D': ['zone_d_tire', 'zone_d_bolt', 'zone_d_drill'],
}

# collision object_id → GraspAssessment LUT 키.
# Zone A 부품은 실측 캘리브레이션 전까지 'ETC' 폴백 사용.
# 실물 하드웨어 파라미터 확보 시 object_lut.json에 종류별 항목 추가 후 여기서 키 변경.
OBJECT_LUT_NAME: dict[str, str] = {
    **{f'zone_a_part_{t}_{i}': 'ETC'
       for t in ZONE_A_PART_TYPES for i in range(4)},
    'zone_b_box':    'ETC',
    'zone_c_bolt_0': 'ETC',
    'zone_c_bolt_1': 'ETC',
    'zone_c_bolt_2': 'ETC',
    'zone_c_bolt_3': 'ETC',
    'zone_d_tire':   'ETC',
    'zone_d_bolt':   'ETC',
    'zone_d_drill':  'ETC',
}

# 각 구간의 낙하 표면 object_id 목록 (detach 시 z 계산용)
# 값: _OBJECT_MARKER_MAP에서 pose.z + scale.z/2 = 표면 top-z
SURFACE_OBJECTS = {
    'A': [ZONE_A_TABLE_TOP_ID],
    'B': [ZONE_B_CONVEYOR_TOP_ID, ZONE_B_DEST_TABLE_ID],
    'C': [ZONE_C_BENCH_TOP_ID],
    'D': [ZONE_D_TIRE_TABLE_TOP_ID, ZONE_D_TOOLBOX_TOP_ID],
}


# ══════════════════════════════════════════════
# Attach / Detach (Phase 2)
# pymoveit2 API: attach_collision_object / detach_collision_object
# ══════════════════════════════════════════════

_TOUCH_LINKS_R = [
    'gripper_r_rh_p12_rn_base',
    'gripper_r_rh_p12_rn_l1', 'gripper_r_rh_p12_rn_l2',
    'gripper_r_rh_p12_rn_r1', 'gripper_r_rh_p12_rn_r2',
]
_TOUCH_LINKS_L = [
    'gripper_l_rh_p12_rn_base',
    'gripper_l_rh_p12_rn_l1', 'gripper_l_rh_p12_rn_l2',
    'gripper_l_rh_p12_rn_r1', 'gripper_l_rh_p12_rn_r2',
]


def attach_object(client: MoveItClient, object_id: str,
                  link_name: str = 'end_effector_r_link',
                  viz=None) -> None:
    left = 'end_effector_l' in link_name
    moveit2     = client.moveit2_l if left else client.moveit2_r
    touch_links = _TOUCH_LINKS_L   if left else _TOUCH_LINKS_R
    moveit2.attach_collision_object(
        id=object_id,
        link_name=link_name,
        touch_links=touch_links,
    )
    if viz is not None:
        viz.highlight_object(object_id, (1.0, 0.0, 0.0, 1.0))
        viz.set_floating(object_id)


def detach_object(client: MoveItClient, object_id: str,
                  link_name: str = 'end_effector_r_link',
                  viz=None, drop_pos: tuple = None) -> None:
    left = 'end_effector_l' in link_name
    moveit2 = client.moveit2_l if left else client.moveit2_r
    moveit2.detach_collision_object(id=object_id)
    if viz is not None:
        if drop_pos is not None:
            viz.drop_object(object_id, drop_pos[0], drop_pos[1], drop_pos[2])
        else:
            viz.restore_object(object_id)


# ══════════════════════════════════════════════
# Smart Grasp / Release (Phase 2 자동화)
# ══════════════════════════════════════════════

def find_nearest_graspable(client: MoveItClient, zone: str,
                           threshold: float = 0.08, arm=None,
                           pos: tuple = None, overrides: dict = None) -> tuple:
    """
    가장 가까운 파지 가능 물체를 탐색.
    pos: (x, y, z) 직접 지정 시 사용. None이면 EEF FK로 계산.
    overrides: EnvironmentVisualizer._overrides — drop된 물체의 현재 위치 반영.
    """
    if pos is not None:
        ex, ey, ez = pos
    else:
        from ai_worker_manipulation.robot_interface.moveit_client import Arm
        pose = client.get_current_pose(arm if arm is not None else Arm.RIGHT)
        if pose is None:
            return (None, float('inf'))
        ex, ey, ez = pose.position.x, pose.position.y, pose.position.z

    best_id, best_dist = None, float('inf')
    for obj_id in GRASPABLE_OBJECTS.get(zone, []):
        if overrides and obj_id in overrides:
            px, py, pz = overrides[obj_id]
        else:
            marker = _OBJECT_MARKER_MAP.get(obj_id)
            if marker is None:
                continue
            px, py, pz = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
        dist = sqrt((ex - px) ** 2 + (ey - py) ** 2 + (ez - pz) ** 2)
        if dist < best_dist:
            best_dist = dist
            best_id = obj_id

    if best_dist <= threshold:
        return (best_id, best_dist)
    return (None, best_dist)


def allow_zone_objects(client: MoveItClient, zone: str) -> None:
    """
    zone의 파지 대상 물체 + 컨테이너 충돌 허용 (양팔 공통).
    competition_manager._allow_graspable 및 demo 스크립트 공용.
    setup_zone_X 호출 후 반드시 실행해야 팔이 물체 근처로 이동 가능.
    """
    import time as _t
    import rclpy as _rclpy
    objs = list(GRASPABLE_OBJECTS.get(zone, []))
    if zone == 'A':
        objs.extend(ZONE_A_CONTAINER_IDS)
    for obj_id in objs:
        for mv in (client.moveit2_r, client.moveit2_l):
            f = mv.allow_collisions(obj_id, True)
            if f is None:
                continue
            deadline = _t.time() + 2.0
            while not f.done() and _t.time() < deadline:
                _rclpy.spin_once(client.node, timeout_sec=0.05)


def smart_close(client: MoveItClient, gripper, zone: str,
                viz=None, threshold: float = 0.08, side: str = 'right') -> str | None:
    """
    그리퍼를 닫고, EEF 기준 threshold(m) 이내 물체가 있으면 자동 attach.
    파지 품질 검증(GraspAssessment)은 competition_manager가 담당.
    demo 스크립트에서 수동 호출용.

    Parameters
    ----------
    gripper : GripperController — Close(side) / Open(side) 메서드 사용
    side    : 'left' | 'right'

    Returns
    -------
    str | None : attach된 object_id, 없으면 None
    """
    from ai_worker_manipulation.robot_interface.moveit_client import Arm
    arm = Arm.LEFT if side == 'left' else Arm.RIGHT
    overrides = viz._overrides if viz is not None else None
    obj_id, dist = find_nearest_graspable(client, zone, threshold, arm=arm, overrides=overrides)
    log = client.node.get_logger()
    if not obj_id:
        log.info(f'smart_close: 조건 미달 — 가장 가까운 물체 dist={dist:.3f}m (threshold={threshold}m)')
        gripper.close(side)
        return None
    gripper.close(side)
    link = f'end_effector_{side[0]}_link'
    attach_object(client, obj_id, link_name=link, viz=viz)
    log.info(f'smart_close: attached [{obj_id}] dist={dist:.3f}m side={side}')
    return obj_id


def smart_open(client: MoveItClient, gripper, object_id: str,
               viz=None, side: str = 'right') -> None:
    """
    그리퍼를 열고, attach된 물체를 자동 detach.

    Parameters
    ----------
    gripper   : GripperController — Open(side) 호출로 assess_moving 스레드 자동 중단
    object_id : smart_close() 반환값. None이면 detach 생략.
    side      : 'left' | 'right'
    """
    gripper.open(side)
    if object_id is not None:
        link = f'end_effector_{side[0]}_link'
        detach_object(client, object_id, link_name=link, viz=viz)
        client.node.get_logger().info(f'smart_open: detached [{object_id}]')