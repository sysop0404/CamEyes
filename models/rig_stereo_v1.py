"""
CamEyes Stereo Camera Rig v1 - FreeCAD Macro
==============================================
FreeCAD에서 실행: Macro > Execute Macro > 이 파일 선택

오브젝트 구성 (개별 편집 가능):
  1. MainBar       - 메인 수평 바 (카메라 2대 연결)
  2. CamCradleL    - 좌측 카메라 거치대
  3. CamCradleR    - 우측 카메라 거치대
  4. IMUPlatform   - 중앙 IMU 마운트
  5. HandleMount   - 하부 핸들/삼각대 마운트
  6. _Ghost_CamL/R - 보드 위치 참고용 (출력 안 함)

수정 방법:
  1. 아래 PARAMETERS 값을 보드 실측 후 수정
  2. FreeCAD에서 매크로 재실행 → 모델 자동 재생성

STL 내보내기:
  오브젝트 선택 > File > Export > .stl
"""

import FreeCAD
import Part
from FreeCAD import Vector

# ============================================================
# PARAMETERS - 보드 도착 후 실측하여 수정하세요
# ============================================================

# ESP32-S3-CAM 보드 치수 (mm)
CAM_W = 27.0            # 보드 폭
CAM_H = 40.5            # 보드 높이
CAM_T = 4.5             # 보드 두께 (부품 포함)
CAM_LENS_D = 8.5        # 카메라 렌즈 직경
CAM_LENS_Z = 10.0       # 렌즈 중심 높이 (보드 하단 기준)
CAM_USB_W = 8.0         # Micro USB 커넥터 폭
CAM_USB_H = 3.5         # Micro USB 커넥터 높이

# GY-9250 IMU 보드 치수 (mm)
IMU_W = 15.5            # IMU 폭
IMU_H = 21.0            # IMU 높이
IMU_T = 2.0             # IMU 두께

# 스테레오 베이스라인 (mm) - 카메라 렌즈 중심 간 거리
BASELINE = 100.0

# 리그 구조 (mm)
WALL = 2.5              # 벽 두께
TOLERANCE = 0.4         # 끼움 공차 (프린터에 따라 조정, 0.3~0.5)
BAR_H = 10.0            # 메인 바 높이
BAR_D = 14.0            # 메인 바 깊이 (전후)

# 삼각대/핸들 마운트
TRIPOD_HOLE_D = 6.35    # 1/4 inch = 6.35mm
TRIPOD_MOUNT_H = 20.0   # 마운트 기둥 높이
TRIPOD_MOUNT_W = 20.0   # 마운트 기둥 폭
TRIPOD_MOUNT_D = 16.0   # 마운트 기둥 깊이

# M2 나사
M2_D = 2.2              # M2 + 공차

# ============================================================
# 계산값 (수정 불필요)
# ============================================================
SLOT_W = CAM_W + TOLERANCE
SLOT_T = CAM_T + TOLERANCE
CRADLE_W = SLOT_W + WALL * 2
CRADLE_D = SLOT_T + WALL * 2
CRADLE_H = CAM_H * 0.55
BAR_W = BASELINE + CRADLE_W
CAM_L_X = -BASELINE / 2
CAM_R_X = BASELINE / 2

# ============================================================
# 문서 생성
# ============================================================
DOC_NAME = "CamEyes_StereoRig_v1"
if FreeCAD.ActiveDocument and FreeCAD.ActiveDocument.Name == DOC_NAME:
    FreeCAD.closeDocument(DOC_NAME)
doc = FreeCAD.newDocument(DOC_NAME)


# ============================================================
# 1. MAIN BAR (메인 수평 바)
# ============================================================
def make_main_bar():
    # 기본 바
    bar = Part.makeBox(BAR_W, BAR_D, BAR_H)
    bar.translate(Vector(-BAR_W / 2, -BAR_D / 2, 0))

    # 경량화 홀 (좌/우 대칭)
    pocket_w = (BASELINE - CRADLE_W) / 2 - 6
    if pocket_w > 8:
        for sign in [-1, 1]:
            cx = sign * BASELINE / 4
            pocket = Part.makeBox(
                pocket_w, BAR_D - WALL * 2, BAR_H + 2
            )
            pocket.translate(Vector(
                cx - pocket_w / 2,
                -BAR_D / 2 + WALL,
                -1
            ))
            bar = bar.cut(pocket)

    # 상단 모서리 필렛 느낌 (45도 챔퍼)
    # (FreeCAD Part에서 fillet이 까다로우므로 심플하게 유지)

    obj = doc.addObject("Part::Feature", "MainBar")
    obj.Shape = bar
    obj.ViewObject.ShapeColor = (0.65, 0.65, 0.68)
    return obj

main_bar = make_main_bar()


# ============================================================
# 2 & 3. CAMERA CRADLE (카메라 거치대)
# ============================================================
def make_camera_cradle(name, x_center):
    """
    U자형 거치대 - 카메라를 위에서 슬라이드 삽입
    전면에 렌즈 관통홀, 후면에 USB 포트 개구부
    """
    x0 = x_center - CRADLE_W / 2
    y0 = -CRADLE_D / 2
    z0 = BAR_H

    # 외형
    outer = Part.makeBox(CRADLE_W, CRADLE_D, CRADLE_H)
    outer.translate(Vector(x0, y0, z0))

    # 내부 슬롯 (상단 개방 - 보드 삽입)
    slot = Part.makeBox(SLOT_W, SLOT_T, CRADLE_H + 1)
    slot.translate(Vector(
        x_center - SLOT_W / 2,
        -SLOT_T / 2,
        z0 + WALL
    ))
    shape = outer.cut(slot)

    # 렌즈 관통홀 (전면)
    lens_r = CAM_LENS_D / 2 + 1.0
    lens_hole = Part.makeCylinder(
        lens_r,
        WALL + 4,
        Vector(x_center, y0 - 2, z0 + WALL + CAM_LENS_Z),
        Vector(0, 1, 0)
    )
    shape = shape.cut(lens_hole)

    # USB 포트 개구부 (후면 하단)
    usb_cut = Part.makeBox(
        CAM_USB_W + 2, WALL + 4, CAM_USB_H + 2
    )
    usb_cut.translate(Vector(
        x_center - (CAM_USB_W + 2) / 2,
        CRADLE_D / 2 - WALL - 2,
        z0 + WALL + 1
    ))
    shape = shape.cut(usb_cut)

    # M2 고정 나사홀 (좌우 관통, 보드 중간 높이)
    screw_z = z0 + CRADLE_H * 0.5
    for dy_sign in [-1, 1]:
        screw = Part.makeCylinder(
            M2_D / 2,
            CRADLE_D + 4,
            Vector(x_center, dy_sign * (CRADLE_D / 2 + 2), screw_z),
            Vector(0, -dy_sign, 0)
        )
        shape = shape.cut(screw)

    # 바닥면 보드 걸림턱 (보드가 아래로 빠지지 않도록)
    lip_h = 1.0
    lip = Part.makeBox(SLOT_W - 2, SLOT_T - 1, lip_h)
    lip.translate(Vector(
        x_center - (SLOT_W - 2) / 2,
        -(SLOT_T - 1) / 2,
        z0 + WALL - lip_h
    ))
    # lip은 슬롯 안쪽으로 살짝 돌출
    inner_lip_w = 1.5
    for dx_sign in [-1, 1]:
        lip_piece = Part.makeBox(inner_lip_w, SLOT_T - 2, lip_h)
        lx = x_center + dx_sign * (SLOT_W / 2 - inner_lip_w) - (inner_lip_w / 2 if dx_sign > 0 else -inner_lip_w / 2)
        lip_piece.translate(Vector(
            x_center + dx_sign * (SLOT_W / 2 - inner_lip_w),
            -(SLOT_T - 2) / 2,
            z0 + WALL
        ))
        shape = shape.fuse(lip_piece)

    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    obj.ViewObject.ShapeColor = (0.2, 0.5, 0.85)
    return obj

cam_cradle_l = make_camera_cradle("CamCradleL", CAM_L_X)
cam_cradle_r = make_camera_cradle("CamCradleR", CAM_R_X)


# ============================================================
# 4. IMU PLATFORM (중앙 IMU 마운트)
# ============================================================
def make_imu_platform():
    base_w = IMU_W + WALL * 2 + TOLERANCE
    base_d = IMU_H + WALL * 2 + TOLERANCE
    base_h = 3.5
    lip_h = 1.8

    x0 = -base_w / 2
    y0 = -base_d / 2
    z0 = BAR_H

    # 기본 플랫폼
    plat = Part.makeBox(base_w, base_d, base_h)
    plat.translate(Vector(x0, y0, z0))

    # IMU 안착 홈
    groove_w = IMU_W + TOLERANCE
    groove_d = IMU_H + TOLERANCE
    groove = Part.makeBox(groove_w, groove_d, IMU_T + 0.5)
    groove.translate(Vector(
        -groove_w / 2,
        -groove_d / 2,
        z0 + base_h - IMU_T - 0.3
    ))
    plat = plat.cut(groove)

    # 4면 가이드 립
    lip_configs = [
        (-base_w / 2, -base_d / 2, base_w, WALL),          # 전면
        (-base_w / 2, base_d / 2 - WALL, base_w, WALL),     # 후면
        (-base_w / 2, -base_d / 2, WALL, base_d),           # 좌측
        (base_w / 2 - WALL, -base_d / 2, WALL, base_d),     # 우측
    ]
    for lx, ly, lw, ld in lip_configs:
        lip = Part.makeBox(lw, ld, lip_h)
        lip.translate(Vector(lx, ly, z0 + base_h))
        plat = plat.fuse(lip)

    # M2 고정홀 (대각선 2개)
    hole_positions = [
        (-IMU_W / 2 + 2.5, -IMU_H / 2 + 2.5),
        (IMU_W / 2 - 2.5, IMU_H / 2 - 2.5),
    ]
    for hx, hy in hole_positions:
        hole = Part.makeCylinder(
            M2_D / 2,
            base_h + lip_h + 4,
            Vector(hx, hy, z0 - 2),
            Vector(0, 0, 1)
        )
        plat = plat.cut(hole)

    obj = doc.addObject("Part::Feature", "IMUPlatform")
    obj.Shape = plat
    obj.ViewObject.ShapeColor = (0.85, 0.45, 0.15)
    return obj

imu_platform = make_imu_platform()


# ============================================================
# 5. HANDLE MOUNT (핸들/삼각대 마운트)
# ============================================================
def make_handle_mount():
    mount_h = TRIPOD_MOUNT_H
    z_top = 0  # 메인 바 하단에 연결

    # 사각 기둥 (원통보다 출력 안정적)
    body = Part.makeBox(TRIPOD_MOUNT_W, TRIPOD_MOUNT_D, mount_h)
    body.translate(Vector(
        -TRIPOD_MOUNT_W / 2,
        -TRIPOD_MOUNT_D / 2,
        z_top - mount_h
    ))

    # 상단 플랜지 (바와 접합면 확대)
    flange_w = TRIPOD_MOUNT_W + 6
    flange_d = BAR_D
    flange_h = 4.0
    flange = Part.makeBox(flange_w, flange_d, flange_h)
    flange.translate(Vector(
        -flange_w / 2,
        -flange_d / 2,
        z_top - flange_h
    ))
    body = body.fuse(flange)

    # 1/4" 삼각대 나사홀 (하단에서 관통)
    tripod_hole = Part.makeCylinder(
        TRIPOD_HOLE_D / 2,
        mount_h + flange_h + 4,
        Vector(0, 0, z_top - mount_h - 2),
        Vector(0, 0, 1)
    )
    body = body.cut(tripod_hole)

    # 경량화 측면 홈
    if mount_h > 12:
        for dx_sign in [-1, 1]:
            side_cut = Part.makeBox(
                TRIPOD_MOUNT_W * 0.3,
                TRIPOD_MOUNT_D + 4,
                mount_h - 8
            )
            side_cut.translate(Vector(
                dx_sign * (TRIPOD_MOUNT_W / 2 - TRIPOD_MOUNT_W * 0.15) - TRIPOD_MOUNT_W * 0.15,
                -TRIPOD_MOUNT_D / 2 - 2,
                z_top - mount_h + 4
            ))
            body = body.cut(side_cut)

    obj = doc.addObject("Part::Feature", "HandleMount")
    obj.Shape = body
    obj.ViewObject.ShapeColor = (0.4, 0.4, 0.42)
    return obj

handle_mount = make_handle_mount()


# ============================================================
# 6. GHOST BOARDS (보드 위치 참고용 - 출력 안 함)
# ============================================================
def make_ghost_board(name, x_center):
    """실제 보드 크기의 투명 박스 - 위치 확인용"""
    board = Part.makeBox(CAM_W, CAM_T, CAM_H)
    board.translate(Vector(
        x_center - CAM_W / 2,
        -CAM_T / 2,
        BAR_H + WALL
    ))
    # 렌즈 표시 (작은 원통)
    lens = Part.makeCylinder(
        CAM_LENS_D / 2,
        3,
        Vector(x_center, -CAM_T / 2 - 3, BAR_H + WALL + CAM_LENS_Z),
        Vector(0, 1, 0)
    )
    board = board.fuse(lens)

    obj = doc.addObject("Part::Feature", name)
    obj.Shape = board
    obj.ViewObject.ShapeColor = (0.0, 0.9, 0.2)
    obj.ViewObject.Transparency = 75
    obj.ViewObject.Visibility = False  # 기본 숨김, 필요 시 트리에서 켜기
    return obj

ghost_l = make_ghost_board("_Ghost_CamL", CAM_L_X)
ghost_r = make_ghost_board("_Ghost_CamR", CAM_R_X)


# ============================================================
# 문서 재계산 및 뷰 설정
# ============================================================
doc.recompute()

if FreeCAD.GuiUp:
    import FreeCADGui
    FreeCADGui.activeDocument().activeView().viewIsometric()
    FreeCADGui.SendMsgToActiveView("ViewFit")

# ============================================================
# 안내 출력
# ============================================================
print()
print("=" * 55)
print("  CamEyes Stereo Rig v1 - 모델 생성 완료!")
print("=" * 55)
print()
print(f"  베이스라인:      {BASELINE:.0f}mm")
print(f"  카메라 보드:     {CAM_W} x {CAM_H} x {CAM_T}mm")
print(f"  IMU 보드:        {IMU_W} x {IMU_H}mm")
print(f"  전체 리그 폭:    {BAR_W:.1f}mm")
print(f"  공차:            {TOLERANCE}mm")
print()
print("  오브젝트 목록:")
print("    1. MainBar       메인 바          (회색)")
print("    2. CamCradleL    좌측 거치대      (파랑)")
print("    3. CamCradleR    우측 거치대      (파랑)")
print("    4. IMUPlatform   IMU 마운트       (주황)")
print("    5. HandleMount   핸들 마운트      (진회색)")
print("    6. _Ghost_Cam*   보드 위치 참고   (숨김)")
print()
print("  수정: PARAMETERS 값 변경 후 매크로 재실행")
print("  내보내기: 오브젝트 선택 > File > Export > .stl")
print("=" * 55)
