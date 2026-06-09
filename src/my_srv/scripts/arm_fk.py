#!/usr/bin/env python3
"""
arm_fk.py — 机械臂正运动学（Forward Kinematics）模块

========== IK 基座原点说明 ==========
原点位于 **#000舵机（底座旋转轴）中心，底板安装面处**：
  - z 轴：向上，z=0 即底板平面
  - y 轴：向前（机械臂主要工作方向，正值 = 远离机身）
  - x 轴：向右（右手坐标系，x = cross(y, z)）

机械臂DH链路（默认参数，与 z_move.py 完全一致）：
  L0 = 111 mm  底座旋转面到肩关节高度（z 方向偏移）
  L1 = 105 mm  大臂（肩→肘）
  L2 =  88 mm  小臂（肘→腕关节）
  L3 = 178 mm  末端执行器（腕→爪子尖）

关节定义（正向 PWM 编号 = z_move.py 中的 servo_pwm 序号）：
  servo_pwm[0] ← theta6：底座旋转（从 +y 轴量起，顺时针为正）
  servo_pwm[1] ← theta5：大臂仰角（0° = 水平，正值 = 向上抬）
  servo_pwm[2] ← theta4：肘关节内夹角（0° = 完全伸展）
  servo_pwm[3] ← theta3：腕俯仰（等于 alpha - theta5 + theta4）

========== 末端执行器（EE）坐标系 ==========
  z_ee：爪子抓取方向（从爪子向前）
  y_ee：爪子向上方向（alpha=0水平时与 -z_base 方向一致，即指向地面）
  x_ee：爪子侧向（右手坐标系）

旋转矩阵 R_base_to_ee（列 = EE坐标轴在基座系中的单位向量）：
  R = [ cos(θ6),  sin(α)·sin(θ6),  cos(α)·sin(θ6)]
      [-sin(θ6),  sin(α)·cos(θ6),  cos(α)·cos(θ6)]
      [       0,        -cos(α),           sin(α) ]
  其中 θ6 = 底座转角（rad），α = 末端俯仰角（rad，z_move 中的 best_alpha）

用法示例：
  from arm_fk import compute_T_base_to_ee
  T = compute_T_base_to_ee(x_mm=0, y_mm=200, z_mm=100)
  # T: 4×4 齐次变换矩阵（mm单位）
"""

import os
import sys
import math
import numpy as np

# ── 把 OpenCV 目录加入 PYTHONPATH 以便导入 z_move ────────────────────────────
_OPENVC_DIR = os.path.expanduser('~/OpenCV')
if _OPENVC_DIR not in sys.path:
    sys.path.insert(0, _OPENVC_DIR)

# ── 机械臂默认参数（0.1mm单位，与 z_move.py 完全相同）────────────────────────
_L0_01mm = 1110
_L1_01mm = 1050
_L2_01mm = 880
_L3_01mm = 1780

# mm 单位
L0 = _L0_01mm / 10.0   # 111 mm
L1 = _L1_01mm / 10.0   # 105 mm
L2 = _L2_01mm / 10.0   #  88 mm
L3 = _L3_01mm / 10.0   # 178 mm


def _find_best_alpha_and_theta6(x_mm: float, y_mm: float, z_mm: float):
    """
    复现 z_move.kinematics_move 的 alpha 搜索逻辑，
    返回 (best_alpha_deg, theta6_deg)。
    alpha 搜索范围：0° → -135°，取最小（最向下）有效值。
    """
    try:
        import z_move
        best_alpha = 0
        for alpha in range(0, -136, -1):
            if z_move.kinematics_analysis(x_mm, y_mm, z_mm, alpha) == 0:
                best_alpha = alpha
        # 用 best_alpha 重算一次以写入 servo_angle
        z_move.kinematics_analysis(x_mm, y_mm, z_mm, best_alpha)
        theta6 = float(z_move.servo_angle[0])
        return best_alpha, theta6
    except ImportError:
        pass

    # ── Fallback：不依赖 z_move 的解析实现 ────────────────────────────────────
    # 计算 theta6（底座旋转角）
    if x_mm == 0 and y_mm == 0:
        theta6 = 0.0
    elif x_mm > 0 and y_mm < 0:
        theta6 = 180.0 + math.degrees(math.atan(x_mm / y_mm))
    elif y_mm == 0:
        y_mm = -0.5
        theta6 = math.degrees(math.atan(x_mm / y_mm)) - 180.0
    elif y_mm > 0:
        theta6 = math.degrees(math.atan(x_mm / y_mm))
    else:
        theta6 = math.degrees(math.atan(x_mm / y_mm)) - 180.0

    y_proj = math.sqrt(x_mm * x_mm + y_mm * y_mm)
    best_alpha = 0
    for alpha in range(0, -136, -1):
        a_r = math.radians(alpha)
        y_adj = y_proj - L3 * math.cos(a_r)
        z_adj = z_mm - L0 - L3 * math.sin(a_r)
        if z_adj < -L0:
            continue
        dist2 = y_adj * y_adj + z_adj * z_adj
        if dist2 > (L1 + L2) ** 2:
            continue
        bbb = (dist2 + L1 * L1 - L2 * L2) / (2.0 * L1 * math.sqrt(dist2))
        if abs(bbb) > 1.0:
            continue
        aaa = -(dist2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if abs(aaa) > 1.0:
            continue
        ccc = math.acos(y_adj / math.sqrt(dist2))
        zf = -1 if z_adj < 0 else 1
        theta5 = math.degrees(ccc * zf + math.acos(bbb))
        if not (0 <= theta5 <= 180):
            continue
        theta4 = 180.0 - math.degrees(math.acos(aaa))
        if not (-135 <= theta4 <= 135):
            continue
        theta3 = alpha - theta5 + theta4
        best_alpha = alpha
        break

    return best_alpha, theta6


def build_rotation_matrix(theta6_deg: float, alpha_deg: float) -> np.ndarray:
    """
    构造 R_base_to_ee（3×3）。

    Args:
        theta6_deg: 底座旋转角（度）
        alpha_deg:  末端俯仰角（度，即 best_alpha）

    Returns:
        R (3×3 ndarray): 列向量为 EE 坐标轴在基座系中的方向

    推导说明：
      设 θ = theta6，α = alpha（末端与水平面夹角）
      EE 坐标轴在基座系中的表达：
        x_ee = ( cos θ,  -sin θ,    0)   ← 爪子侧向（垂直于臂平面）
        y_ee = (sin α·sin θ, sin α·cos θ, -cos α)  ← 爪子向上（alpha=0时向下）
        z_ee = (cos α·sin θ, cos α·cos θ,  sin α)  ← 爪子抓取方向
      R = [x_ee | y_ee | z_ee]
    """
    t6 = math.radians(theta6_deg)
    a  = math.radians(alpha_deg)
    ct6, st6 = math.cos(t6), math.sin(t6)
    ca,  sa  = math.cos(a),  math.sin(a)

    R = np.array([
        [ ct6,  sa * st6,  ca * st6],
        [-st6,  sa * ct6,  ca * ct6],
        [   0,       -ca,        sa],
    ], dtype=np.float64)
    return R


def compute_T_base_to_ee(
    x_mm: float,
    y_mm: float,
    z_mm: float,
    theta6_deg: float = None,
    alpha_deg:  float = None,
) -> np.ndarray:
    """
    计算末端执行器在基座坐标系中的位姿（4×4 齐次矩阵，mm 单位）。

    对应关系：若调用 kinematics_move(x_mm, y_mm, z_mm, t)，
    则本函数返回与之完全一致的 T_base_to_ee。

    Args:
        x_mm, y_mm, z_mm: 末端坐标（mm），与 kinematics_move 相同含义
        theta6_deg: 底座转角（None = 自动计算）
        alpha_deg:  末端俯仰角（None = 自动选最优 alpha）

    Returns:
        T (4×4 ndarray): T_base_to_ee，单位 mm
            T[:3,:3] = R_base_to_ee
            T[:3, 3] = [x_mm, y_mm, z_mm]（EE原点在基座系坐标）
    """
    if alpha_deg is None or theta6_deg is None:
        a, t6 = _find_best_alpha_and_theta6(x_mm, y_mm, z_mm)
        if alpha_deg  is None: alpha_deg  = a
        if theta6_deg is None: theta6_deg = t6

    R = build_rotation_matrix(theta6_deg, alpha_deg)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = np.array([x_mm, y_mm, z_mm])
    return T


def compute_T_base_to_ee_from_angles(
    theta6_deg: float,
    theta5_deg: float,
    theta4_deg: float,
    theta3_deg: float,
) -> np.ndarray:
    """
    从 4 个关节角度（度）正向计算 T_base_to_ee（4×4，mm 单位）。

    角度定义与 z_move.kinematics_analysis 完全一致：
      theta6: 底座旋转（servo_pwm[0]）
      theta5: 大臂俯仰，0°=水平、正=向上（servo_pwm[1] 对应 servo_angle[1]=theta5-90）
      theta4: 肘关节内角，180°=完全伸展（servo_pwm[2]）
      theta3: 腕俯仰（servo_pwm[3]）

    PWM↔角度（z_move.py 的 servo_pwm 公式反解）：
      theta6_deg = (1500 - pwm0) * 270 / 2000
      theta5_deg = (pwm1 - 1500) * 270 / 2000 + 90
      theta4_deg = (pwm2 - 1500) * 270 / 2000
      theta3_deg = (pwm3 - 1500) * 270 / 2000
    """
    alpha_deg = theta3_deg + theta5_deg - theta4_deg
    beta_deg  = theta5_deg - theta4_deg            # L2 前臂与水平面夹角

    t5 = math.radians(theta5_deg)
    b  = math.radians(beta_deg)
    a  = math.radians(alpha_deg)
    t6 = math.radians(theta6_deg)

    y_proj = L1 * math.cos(t5) + L2 * math.cos(b) + L3 * math.cos(a)
    z_mm   = L0 + L1 * math.sin(t5) + L2 * math.sin(b) + L3 * math.sin(a)
    x_mm   = y_proj * math.sin(t6)
    y_mm   = y_proj * math.cos(t6)

    R = build_rotation_matrix(theta6_deg, alpha_deg)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = np.array([x_mm, y_mm, z_mm])
    return T


def pwms_to_angles(pwm0: int, pwm1: int, pwm2: int, pwm3: int):
    """servo_pwm[0..3] → (theta6, theta5, theta4, theta3) 度。"""
    theta6 = (1500 - pwm0) * 270.0 / 2000.0
    theta5 = (pwm1 - 1500) * 270.0 / 2000.0 + 90.0
    theta4 = (pwm2 - 1500) * 270.0 / 2000.0
    theta3 = (pwm3 - 1500) * 270.0 / 2000.0
    return theta6, theta5, theta4, theta3


def T_mm_to_m(T: np.ndarray) -> np.ndarray:
    """将 T_base_to_ee 的平移部分从 mm 转换为 m（供 cv2.calibrateHandEye 使用）。"""
    Tm = T.copy()
    Tm[:3, 3] /= 1000.0
    return Tm


# ── 调试/验证 ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=== IK 基座原点确认 ===')
    print(f'L0={L0}mm (底板→肩关节高度)')
    print(f'L1={L1}mm (大臂), L2={L2}mm (小臂), L3={L3}mm (末端)')
    print()

    test_cases = [
        (0,   200, 100, '中心位'),
        (0,   200, 111, '中心位（z=L0肩高）'),
        (100, 200, 100, '右移100mm'),
        (0,   300, 150, '前伸远位'),
    ]
    for x, y, z, label in test_cases:
        T = compute_T_base_to_ee(x, y, z)
        a, t6 = _find_best_alpha_and_theta6(x, y, z)
        print(f'[{label}] ({x},{y},{z})mm  theta6={t6:.1f}°  alpha={a}°')
        print(f'  平移: {T[:3,3]}')
        print(f'  旋转:\n{T[:3,:3].round(4)}')
        # 正交性验证
        RtR = T[:3, :3].T @ T[:3, :3]
        err = np.linalg.norm(RtR - np.eye(3))
        print(f'  正交误差: {err:.2e}')
        print()
