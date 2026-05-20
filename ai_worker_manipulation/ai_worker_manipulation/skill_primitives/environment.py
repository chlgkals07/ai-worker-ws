"""
대회 환경 CollisionObject 설정 모듈.
규정집 제12~15조 기준 (2026 Humanoid Challenge).

좌표 기준:
  - 로봇 base_link = (0, 0, 0)
  - +x : 로봇 정면, +y : 로봇 좌측, +z : 위
  - z 공식: 테이블_상면_z + 물체_높이/2
"""

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient

FRAME_ID = "base_link"

# ══════════════════════════════════════════════
# A구간 — 부품 선별 (규정집 제12조)
# 작업대: 1600×800mm 상판, 높이 800mm
# 혼재 부품 20개(5종), 노란 상자, 파란 트레이, 32인치 모니터
# ══════════════════════════════════════════════

# 작업대 상판 (상면 z = 0.79 + 0.01 = 0.80)
ZONE_A_TABLE_TOP_ID       = "zone_a_table_top"
ZONE_A_TABLE_TOP_SIZE     = (1.6, 0.8, 0.02)   # 1600×800mm 상판, 두께 20mm (규정집 제12조)
ZONE_A_TABLE_TOP_POSITION = (0.9, 0.0, 0.79)   # 중심. 앞 가장자리 x=0.10, 뒤 x=1.70

# 작업대 다리 4개 (상판 모서리 안쪽 25mm)
ZONE_A_TABLE_LEG_SIZE      = (0.05, 0.05, 0.78)
ZONE_A_TABLE_LEG_POSITIONS = [
    (0.20,  0.375, 0.39),    # 앞-우 (로봇 충돌 방지를 위해 x=0.125→0.20)
    (0.20, -0.375, 0.39),    # 앞-좌
    (1.65,  0.375, 0.39),    # 뒤-우
    (1.65, -0.375, 0.39),    # 뒤-좌
]

# 노란 혼재 부품 상자 (작업대 상판 뒤쪽 우측)
ZONE_A_PARTS_BOX_ID       = "zone_a_parts_box"
ZONE_A_PARTS_BOX_SIZE     = (0.35, 0.25, 0.12)   # 노란색 플라스틱 박스 (규정집 제12조)
ZONE_A_PARTS_BOX_POSITION = (1.45, -0.30, 0.86)  # 작업대 뒤쪽 우측 끝 (부품 영역 회피)

# 파란 트레이 (작업대 상판 앞쪽 좌측, 부품 적재 목적지)
ZONE_A_TRAY_ID       = "zone_a_tray"
ZONE_A_TRAY_SIZE     = (0.35, 0.25, 0.06)   # 파란색 플라스틱 박스 (규정집 제12조)
ZONE_A_TRAY_POSITION = (0.45, 0.28, 0.83)   # 작업대 앞쪽 좌측 끝 (부품 영역 회피)

# 32인치 모니터 (작업대 뒤쪽 엣지, 지령 수신 화면)
ZONE_A_MONITOR_ID       = "zone_a_monitor"
ZONE_A_MONITOR_SIZE     = (0.72, 0.05, 0.42)   # 32인치 = 약 720×420mm, 두께 50mm (규정집 제12조)
ZONE_A_MONITOR_POSITION = (1.12, 0.0, 1.01)    # 0.80 + 0.42/2 + 0.02(베젤)

# 혼재 부품 20개 (5종 × 4개), 작업대 상판 중앙 4×5 격자 (규정집 제12조)
# 부품상자/트레이 영역 회피: x 0.70~1.06, y -0.16~0.16
ZONE_A_PART_RADIUS = 0.05    # 링 형태 근사, 외경 100mm
ZONE_A_PART_HEIGHT = 0.05    # 두께 50mm
ZONE_A_PART_Z      = 0.825   # 0.80 + 0.05/2
ZONE_A_PART_XS     = [0.65, 0.78, 0.91, 1.04]           # 4열, 간격 0.13m (상자/트레이 영역 제외)
ZONE_A_PART_YS     = [-0.16, -0.08, 0.00, 0.08, 0.16]   # 5열, 간격 0.08m
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
ZONE_C_PEG_X           = 0.85   # 조립대 중앙
ZONE_C_PEG_Z           = 0.875  # 0.80 + 0.15/2
ZONE_C_PEG_Y_POSITIONS = [-0.225, -0.075, 0.075, 0.225]  # 간격 15cm (규정집 제14조)

# 볼트 4개 (삽입 대상, peg 앞에 대기)
# 볼트 홀 지름 50mm (규정집 제14조)
ZONE_C_BOLT_RADIUS     = 0.025   # 홀 지름 50mm → 반지름 25mm
ZONE_C_BOLT_HEIGHT     = 0.04
ZONE_C_BOLT_X          = 0.85    # peg와 같은 x (조립대 상판 위, peg 근처)
ZONE_C_BOLT_Z          = 0.825   # 0.80 + 0.04/2 + 보정
ZONE_C_BOLT_Y_POSITIONS = [-0.225, -0.075, 0.075, 0.225]  # peg와 동일 Y

# 조립 완료 버튼 (조립대 앞쪽 중앙)
ZONE_C_BUTTON_ID       = "zone_c_button"
ZONE_C_BUTTON_RADIUS   = 0.03    # 지름 60mm 이상, PUSH버튼 (규정집 제14조)
ZONE_C_BUTTON_HEIGHT   = 0.06
ZONE_C_BUTTON_POSITION = (0.65, 0.0, 0.83)   # 0.80 + 0.06/2

# 지령 모니터 (조립대 뒤쪽, 삽입 순서 무작위 표시)
ZONE_C_MONITOR_ID       = "zone_c_monitor"
ZONE_C_MONITOR_SIZE     = (0.72, 0.05, 0.42)   # 32인치 (규정집 제14조)
ZONE_C_MONITOR_POSITION = (1.05, 0.0, 1.01)    # 조립대 뒤 가장자리

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
# 공통 헬퍼 함수
# ══════════════════════════════════════════════

def _add_box(moveit2, obj_id: str, size: tuple, position: tuple):
    """box collision object를 씬에 추가."""
    moveit2.add_collision_box(
        id=obj_id,
        size=size,
        position=position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _add_cylinder(moveit2, obj_id: str, radius: float, height: float, position: tuple):
    """cylinder collision object를 씬에 추가."""
    moveit2.add_collision_cylinder(
        id=obj_id,
        radius=radius,
        height=height,
        position=position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _remove_obj(moveit2, obj_id: str):
    """collision object를 씬에서 제거."""
    moveit2.remove_collision_object(id=obj_id)


def _apply_to_both_arms(client: MoveItClient, fn, *args, **kwargs):
    """moveit2_r, moveit2_l 양쪽 씬에 동일하게 적용. 없는 인터페이스는 무시."""
    for arm_attr in ("moveit2_r", "moveit2_l"):
        try:
            arm = getattr(client, arm_attr)
            fn(arm, *args, **kwargs)
        except AttributeError:
            pass
        except Exception as exc:
            try:
                client.moveit2_r._node.get_logger().warn(
                    f"[environment] {arm_attr} 씬 업데이트 실패 "
                    f"({kwargs.get('id', args[0] if args else '?')}): {exc}"
                )
            except Exception:
                pass


def _zone_a_part_entries():
    """A구간 부품 20개 (id, position) 쌍 반환. setup/remove 공통 사용."""
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
    """A구간 작업대/부품상자/트레이/모니터/부품 20개 씬 등록 (규정집 제12조)."""
    # 1. 작업대 상판
    _apply_to_both_arms(client, _add_box, ZONE_A_TABLE_TOP_ID, ZONE_A_TABLE_TOP_SIZE, ZONE_A_TABLE_TOP_POSITION)
    # 2. 작업대 다리 4개
    for i, pos in enumerate(ZONE_A_TABLE_LEG_POSITIONS):
        _apply_to_both_arms(client, _add_box, f"zone_a_table_leg_{i}", ZONE_A_TABLE_LEG_SIZE, pos)
    # 3. 노란 혼재 부품 상자
    _apply_to_both_arms(client, _add_box, ZONE_A_PARTS_BOX_ID, ZONE_A_PARTS_BOX_SIZE, ZONE_A_PARTS_BOX_POSITION)
    # 4. 파란 트레이
    _apply_to_both_arms(client, _add_box, ZONE_A_TRAY_ID, ZONE_A_TRAY_SIZE, ZONE_A_TRAY_POSITION)
    # 5. 32인치 모니터
    _apply_to_both_arms(client, _add_box, ZONE_A_MONITOR_ID, ZONE_A_MONITOR_SIZE, ZONE_A_MONITOR_POSITION)
    # 6. 혼재 부품 5종 × 4개 = 20개
    for part_id, pos in _zone_a_part_entries():
        _apply_to_both_arms(client, _add_cylinder, part_id, ZONE_A_PART_RADIUS, ZONE_A_PART_HEIGHT, pos)


def remove_zone_a(client: MoveItClient):
    """A구간 모든 collision objects 씬 제거."""
    _apply_to_both_arms(client, _remove_obj, ZONE_A_TABLE_TOP_ID)
    for i in range(len(ZONE_A_TABLE_LEG_POSITIONS)):
        _apply_to_both_arms(client, _remove_obj, f"zone_a_table_leg_{i}")
    for obj_id in (ZONE_A_PARTS_BOX_ID, ZONE_A_TRAY_ID, ZONE_A_MONITOR_ID):
        _apply_to_both_arms(client, _remove_obj, obj_id)
    for part_id, _ in _zone_a_part_entries():
        _apply_to_both_arms(client, _remove_obj, part_id)


# ══════════════════════════════════════════════
# B구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_b(client: MoveItClient):
    """B구간 컨베이어/박스/목적지 테이블 씬 등록 (규정집 제13조)."""
    # 1. 컨베이어 본체
    _apply_to_both_arms(client, _add_box, ZONE_B_CONVEYOR_ID, ZONE_B_CONVEYOR_SIZE, ZONE_B_CONVEYOR_POSITION)
    # 2. 컨베이어 선반 상판
    _apply_to_both_arms(client, _add_box, ZONE_B_CONVEYOR_TOP_ID, ZONE_B_CONVEYOR_TOP_SIZE, ZONE_B_CONVEYOR_TOP_POSITION)
    # 3. 우체국 4호 박스 (파지 대상)
    _apply_to_both_arms(client, _add_box, ZONE_B_BOX_ID, ZONE_B_BOX_SIZE, ZONE_B_BOX_POSITION)
    # 4. 정지선
    _apply_to_both_arms(client, _add_box, ZONE_B_STOPLINE_ID, ZONE_B_STOPLINE_SIZE, ZONE_B_STOPLINE_POSITION)
    # 5. 목적지 테이블
    _apply_to_both_arms(client, _add_box, ZONE_B_DEST_TABLE_ID, ZONE_B_DEST_TABLE_SIZE, ZONE_B_DEST_TABLE_POSITION)
    # 6. 하차 위치선
    _apply_to_both_arms(client, _add_box, ZONE_B_DROPOFF_MARKER_ID, ZONE_B_DROPOFF_MARKER_SIZE, ZONE_B_DROPOFF_MARKER_POSITION)


def remove_zone_b(client: MoveItClient):
    """B구간 모든 collision objects 씬 제거."""
    for obj_id in (
        ZONE_B_CONVEYOR_ID,
        ZONE_B_CONVEYOR_TOP_ID,
        ZONE_B_BOX_ID,
        ZONE_B_STOPLINE_ID,
        ZONE_B_DEST_TABLE_ID,
        ZONE_B_DROPOFF_MARKER_ID,
    ):
        _apply_to_both_arms(client, _remove_obj, obj_id)


# ══════════════════════════════════════════════
# C구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_c(client: MoveItClient):
    """C구간 조립대/peg/볼트/버튼/모니터 씬 등록 (규정집 제14조)."""
    # 1. 조립대 몸체
    _apply_to_both_arms(client, _add_box, ZONE_C_BENCH_BODY_ID, ZONE_C_BENCH_BODY_SIZE, ZONE_C_BENCH_BODY_POSITION)
    # 2. 조립대 상판
    _apply_to_both_arms(client, _add_box, ZONE_C_BENCH_TOP_ID, ZONE_C_BENCH_TOP_SIZE, ZONE_C_BENCH_TOP_POSITION)
    # 3. peg 4개 — 지름 각각 다름 (규정집 붙임1)
    for i, (r, y) in enumerate(zip(ZONE_C_PEG_RADII, ZONE_C_PEG_Y_POSITIONS)):
        _apply_to_both_arms(client, _add_cylinder,
                            f"zone_c_peg_{i}", r, ZONE_C_PEG_HEIGHT,
                            (ZONE_C_PEG_X, y, ZONE_C_PEG_Z))
    # 4. 볼트 4개 (삽입 대상, peg 앞 대기)
    for i, y in enumerate(ZONE_C_BOLT_Y_POSITIONS):
        _apply_to_both_arms(client, _add_cylinder,
                            f"zone_c_bolt_{i}", ZONE_C_BOLT_RADIUS, ZONE_C_BOLT_HEIGHT,
                            (ZONE_C_BOLT_X, y, ZONE_C_BOLT_Z))
    # 5. 조립 완료 버튼
    _apply_to_both_arms(client, _add_cylinder,
                        ZONE_C_BUTTON_ID, ZONE_C_BUTTON_RADIUS, ZONE_C_BUTTON_HEIGHT,
                        ZONE_C_BUTTON_POSITION)
    # 6. 지령 모니터
    _apply_to_both_arms(client, _add_box, ZONE_C_MONITOR_ID, ZONE_C_MONITOR_SIZE, ZONE_C_MONITOR_POSITION)


def remove_zone_c(client: MoveItClient):
    """C구간 모든 collision objects 씬 제거."""
    for obj_id in (
        ZONE_C_BENCH_BODY_ID,
        ZONE_C_BENCH_TOP_ID,
        ZONE_C_BUTTON_ID,
        ZONE_C_MONITOR_ID,
    ):
        _apply_to_both_arms(client, _remove_obj, obj_id)
    for i in range(len(ZONE_C_PEG_Y_POSITIONS)):
        _apply_to_both_arms(client, _remove_obj, f"zone_c_peg_{i}")
    for i in range(len(ZONE_C_BOLT_Y_POSITIONS)):
        _apply_to_both_arms(client, _remove_obj, f"zone_c_bolt_{i}")


# ══════════════════════════════════════════════
# D구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_d(client: MoveItClient):
    """D구간 거치 테이블/타이어/공구함/볼트/드릴/홀 기둥 씬 등록 (규정집 제15조)."""
    # 1. 타이어 거치 테이블 (우측)
    _apply_to_both_arms(client, _add_box, ZONE_D_TIRE_TABLE_ID, ZONE_D_TIRE_TABLE_SIZE, ZONE_D_TIRE_TABLE_POSITION)
    _apply_to_both_arms(client, _add_box, ZONE_D_TIRE_TABLE_TOP_ID, ZONE_D_TIRE_TABLE_TOP_SIZE, ZONE_D_TIRE_TABLE_TOP_POSITION)
    # 2. 타이어
    _apply_to_both_arms(client, _add_cylinder,
                        ZONE_D_TIRE_ID, ZONE_D_TIRE_RADIUS, ZONE_D_TIRE_HEIGHT,
                        ZONE_D_TIRE_POSITION)
    # 3. 공구함 (좌측)
    _apply_to_both_arms(client, _add_box, ZONE_D_TOOLBOX_ID, ZONE_D_TOOLBOX_SIZE, ZONE_D_TOOLBOX_POSITION)
    _apply_to_both_arms(client, _add_box, ZONE_D_TOOLBOX_TOP_ID, ZONE_D_TOOLBOX_TOP_SIZE, ZONE_D_TOOLBOX_TOP_POSITION)
    # 4. 체결 볼트
    _apply_to_both_arms(client, _add_cylinder,
                        ZONE_D_BOLT_ID, ZONE_D_BOLT_RADIUS, ZONE_D_BOLT_HEIGHT,
                        ZONE_D_BOLT_POSITION)
    # 5. 전동 드릴
    _apply_to_both_arms(client, _add_box, ZONE_D_DRILL_ID, ZONE_D_DRILL_SIZE, ZONE_D_DRILL_POSITION)
    # 6. 타이어 체결 홀 기둥
    _apply_to_both_arms(client, _add_cylinder,
                        ZONE_D_HOLE_POST_ID, ZONE_D_HOLE_POST_RADIUS, ZONE_D_HOLE_POST_HEIGHT,
                        ZONE_D_HOLE_POST_POSITION)
    # 7. 휠 허브 (기둥 끝)
    _apply_to_both_arms(client, _add_cylinder,
                        ZONE_D_WHEEL_HUB_ID, ZONE_D_WHEEL_HUB_RADIUS, ZONE_D_WHEEL_HUB_HEIGHT,
                        ZONE_D_WHEEL_HUB_POSITION)
    # 8. 기둥 바닥 고정부
    _apply_to_both_arms(client, _add_box, ZONE_D_HOLE_BASE_ID, ZONE_D_HOLE_BASE_SIZE, ZONE_D_HOLE_BASE_POSITION)


def remove_zone_d(client: MoveItClient):
    """D구간 모든 collision objects 씬 제거."""
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
        _apply_to_both_arms(client, _remove_obj, obj_id)


# ══════════════════════════════════════════════
# 전체 환경 설정 / 해제
# ══════════════════════════════════════════════

def setup_environment(client: MoveItClient):
    """모든 구간 CollisionObject 씬 등록 (기존 인터페이스 유지)."""
    setup_zone_a(client)
    setup_zone_b(client)
    setup_zone_c(client)
    setup_zone_d(client)
    try:
        client.moveit2_r._node.get_logger().info(
            "환경 설정 완료: A/B/C/D 구간 collision objects 등록됨"
        )
    except Exception:
        pass


def remove_environment(client: MoveItClient):
    """모든 구간 CollisionObject 씬 제거."""
    remove_zone_a(client)
    remove_zone_b(client)
    remove_zone_c(client)
    remove_zone_d(client)
    try:
        client.moveit2_r._node.get_logger().info(
            "환경 해제 완료: 모든 collision objects 제거됨"
        )
    except Exception:
        pass
