"""
GPD grasp orientation 분석 스크립트 (ROS 불필요, numpy/scipy 불필요)

GPD 쿼터니언을 분석해서:
  1) RPY 변환
  2) approach / binormal / axis 벡터 확인
  3) EE frame 컨벤션 불일치 여부 확인
  4) 위치 도달 가능 범위 체크

Usage:
    python3 analyze_grasp_orient.py
"""

import math


# ─── 분석할 GPD grasp 데이터 입력 ─────────────────────────────────────────────
# (label, x, y, z, qx, qy, qz, qw)  ← base_link 프레임 기준
GRASPS = [
    ('Grasp0  score=-1495', 0.5280, -0.1689, 0.9305, 0.7122,  0.6307, -0.2970,  0.0824),
    ('Grasp1  score=-1557', 0.5702, -0.2625, 0.8995, 0.7299,  0.1949, -0.5875, -0.2901),
    ('Grasp2  score=-2316', 0.5377, -0.2334, 0.9242, 0.6582, -0.0942, -0.6347, -0.3937),
    ('GT pos  (ground)',    0.5710, -0.2130, 0.8690, None, None, None, None),
]

# ─── SG2 arm_l 기구학 파라미터 (ffw_sg2_follower.urdf 기준) ──────────────────
# base_link 기준 arm_base 위치 (lift=0 기준)
ARM_BASE_X = -0.0199
ARM_BASE_Y =  0.1045   # arm_l_joint1 의 y offset
ARM_BASE_Z =  1.4316

# arm_l 링크 offset 합산으로 구한 최대 직선 도달 거리 (m)
# j3(-0.165) + j4(-0.135) + j5(-0.1489) + j6(-0.1041) + j7(-0.0885) + ee(-0.215)
MAX_REACH = 0.857

# arm_l 마지막 2 관절 제한
J6_LIMIT_DEG = 90.0    # ±90°  (y-axis)
J7_LIMIT_DEG = (104.3, 90.6)  # [-104°, +91°]  (x-axis)
# ──────────────────────────────────────────────────────────────────────────────


def quat_to_matrix(qx, qy, qz, qw):
    return [
        [1-2*(qy**2+qz**2),  2*(qx*qy-qw*qz),  2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz),  1-2*(qx**2+qz**2),  2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),  1-2*(qx**2+qy**2)],
    ]


def quat_to_rpy(qx, qy, qz, qw):
    """ZYX (intrinsic) convention → roll/pitch/yaw in degrees"""
    roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
    pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
    yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def col(R, i):
    return [R[0][i], R[1][i], R[2][i]]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def norm(v):
    return math.sqrt(sum(x**2 for x in v))


def sep():
    print('─' * 65)


# ─────────────────────────────────────────────────────────────────────────────
print('=' * 65)
print('  GPD Grasp Orientation Analyzer — SG2 arm_l')
print('=' * 65)

# ── 1) RPY + approach/binormal/axis ──────────────────────────────────────────
sep()
print('[1] RPY 변환 및 GPD 프레임 벡터 (base_link 기준)')
sep()
print(f'  GPD 컨벤션:  R = [approach | binormal | axis]')
print(f'    +x col  = approach  (그리퍼가 물체 쪽으로 이동하는 방향)')
print(f'    +y col  = binormal  (그리퍼 닫힘 방향)')
print(f'    +z col  = axis      (핑거 대칭축)')
print()

for label, x, y, z, qx, qy, qz, qw in GRASPS:
    if qx is None:
        continue
    r, p, yaw = quat_to_rpy(qx, qy, qz, qw)
    R = quat_to_matrix(qx, qy, qz, qw)
    approach = col(R, 0)
    binormal = col(R, 1)
    axis_v   = col(R, 2)

    j6_warn = '  ← ⚠ |roll|>90° — j6 한계 초과 가능성' if abs(r) > J6_LIMIT_DEG else ''

    print(f'  [{label}]')
    print(f'    roll={r:+7.1f}°  pitch={p:+7.1f}°  yaw={yaw:+7.1f}°{j6_warn}')
    print(f'    approach (+x): ({approach[0]:+.3f}, {approach[1]:+.3f}, {approach[2]:+.3f})')
    print(f'    binormal (+y): ({binormal[0]:+.3f}, {binormal[1]:+.3f}, {binormal[2]:+.3f})')
    print(f'    axis     (+z): ({axis_v[0]:+.3f}, {axis_v[1]:+.3f}, {axis_v[2]:+.3f})')
    print()

# ── 2) 위치 도달 가능성 ───────────────────────────────────────────────────────
sep()
print('[2] 위치 도달 가능성 검사 (arm_l, lift=0 기준)')
sep()
print(f'  arm_base 위치 (base_link 기준): '
      f'({ARM_BASE_X:.4f}, {ARM_BASE_Y:.4f}, {ARM_BASE_Z:.4f})')
print(f'  최대 직선 도달 거리: {MAX_REACH:.3f} m')
print()

for label, x, y, z, qx, qy, qz, qw in GRASPS:
    dx = x - ARM_BASE_X
    dy = y - ARM_BASE_Y
    dz = z - ARM_BASE_Z
    dist = math.sqrt(dx**2 + dy**2 + dz**2)
    pct  = dist / MAX_REACH * 100

    if dist < MAX_REACH * 0.85:
        status = '✓ OK'
    elif dist <= MAX_REACH:
        status = '⚠ NEAR LIMIT'
    else:
        status = '✗ OUT OF REACH'

    print(f'  [{label}]')
    print(f'    base_link pos: ({x:.3f}, {y:.3f}, {z:.3f})')
    print(f'    arm_base 기준: ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})')
    print(f'    거리: {dist:.3f} m / max {MAX_REACH:.3f} m  ({pct:.0f}%)  {status}')
    print()

# ── 3) frame convention 불일치 진단 ──────────────────────────────────────────
sep()
print('[3] EE 프레임 컨벤션 불일치 진단')
sep()
print("""
  SG2 URDF 구조:
    arm_l_link7
      └─ end_effector_l_joint (xyz=0 0 -0.215, rpy=0 0 0)
            └─ end_effector_l_link  ← MoveIt IK target frame
      └─ gripper_l_joint     (xyz=0 0 -0.078, rpy=0 π π)
            └─ gripper_l_rh_p12_rn_base

  Zero config 에서 end_effector_l_link 의 +z축 = 위쪽 (+Z world)
  → EE 가 물체 위에서 내려올 때 EE +z 는 아래를 가리켜야 함
    = quat(1,0,0,0) 에서 Rx(180°) → (0,1,0,0) 또는 Ry(180°) → (1,0,0,0)

  GPD 출력 컨벤션:
    R = [approach | binormal | axis]
    코드에서 이 R 을 그대로 quaternion 으로 변환 → EE +x = approach
    하지만 MoveIt 은 end_effector_l_link 의 전체 orientation 을 받음
    → GPD approach 가 EE +x 에 맞게 들어가 있어 방향 불일치 가능성 있음

  문제 증상:
    - roll 이 ±90° 초과 (arm_l_joint6 한계: ±90°)
    - 특히 Grasp0: roll = -162° → arm_l_joint6 로 구현 불가
""")

# ── 4) 권장 수정 방향 ─────────────────────────────────────────────────────────
sep()
print('[4] 권장 수정 / 확인 사항')
sep()
print("""
  ① [높음] roll 이 ±90° 초과하는 grasp 는 IK 해 없음
     → gpd_grasp_publisher.py 에서 filtering 추가:
        roll = abs(rpy_from_quat(q)[0]) <= 90° 인 것만 통과

  ② [높음] EE frame 컨벤션 확인
     → MoveIt move_to_pose 에 넣기 전에 end_effector_l_link 의 실제
       approach 축(+x? +z?)을 실험적으로 확인:
         a) quat=(0,1,0,0) 으로 move_to_pose 실행 → 그리퍼가 어디를 향하는지 확인
         b) approach 축이 +z 이면 GPD orientation 에 Ry(-90°) 보정 필요
              R_corrected = R_gpd @ R_y(−90°)

  ③ [중간] 위치가 최대 도달 거리의 90%+ 에 위치
     → Grasp1(87.5cm), GT(87.5cm) 은 최대 도달 범위(85.7cm) 초과
     → lift 를 올려서 arm_base_z 를 높이거나, 더 가까운 grasp 선택

  ④ [중간] test_tf_gpd.py 의 workspace bound 불일치
     → 현재: X_MAX=0.35, Z_MAX=0.85
     → GPD 출력: x≈0.52-0.57, z≈0.87-0.93  → 전부 OUT으로 표시됨
     → 실제 arm 도달 범위에 맞게 수정 필요: X_MAX≈0.65, Z_MAX≈0.95
""")

sep()
print('done.')
