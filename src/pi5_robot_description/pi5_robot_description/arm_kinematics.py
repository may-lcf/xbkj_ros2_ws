#!/usr/bin/env python3
"""
arm_kinematics.py — 独立机械臂运动学模块 (从 my_srv 移植，去除串口依赖)

DH 链路参数（小车搭载版，L0 因底盘加高）:
  L0 = 240mm  底座旋转面到肩关节高度（z 方向偏移）
  L1 = 105mm  大臂（肩→肘）
  L2 =  88mm  小臂（肘→腕关节）
  L3 = 178mm  末端执行器（腕→爪子尖）

关节定义:
  servo_pwm[0] ← theta6: 底座旋转（从 +y 轴量起）
  servo_pwm[1] ← theta5: 大臂仰角（0°=水平，正值=向上）
  servo_pwm[2] ← theta4: 肘关节内夹角（0°=完全伸展）
  servo_pwm[3] ← theta3: 腕俯仰
  servo_pwm[4] ← 夹爪 (1500=中位, >1500闭合, <1500打开)

PWM↔角度公式:
  theta6 = (1500 - pwm0) * 270 / 2000
  theta5 = (pwm1 - 1500) * 270 / 2000 + 90
  theta4 = (pwm2 - 1500) * 270 / 2000
  theta3 = (pwm3 - 1500) * 270 / 2000
"""

import math
import numpy as np

# ── 机械臂 DH 参数 (0.1mm 单位，与 z_move.py 一致) ──
_L0_01mm = 2400   # 240mm (小车底盘加高)
_L1_01mm = 1050   # 105mm
_L2_01mm = 880    #  88mm
_L3_01mm = 1780   # 178mm

# mm 单位
L0 = _L0_01mm / 10.0   # 240mm
L1 = _L1_01mm / 10.0   # 105mm
L2 = _L2_01mm / 10.0   #  88mm
L3 = _L3_01mm / 10.0   # 178mm

# 全局变量（兼容 z_move 的接口风格）
servo_angle = [0.0, 0.0, 0.0, 0.0]
servo_pwm = [0, 0, 0, 0]


def kinematics_analysis(x, y, z, alpha):
    """
    逆运动学分析 (移植自 z_move.py，去除串口依赖)

    Args:
        x, y, z: 末端坐标 (mm)
        alpha: 末端与水平面夹角 (度)

    Returns:
        0 表示成功，非0 为错误码
    """
    global servo_angle, servo_pwm
    pi = math.pi

    # 转换为 0.1mm 单位
    x10 = x * 10
    y10 = y * 10
    z10 = z * 10

    # 计算底座旋转角 theta6
    if x == 0:
        theta6 = 0.0
    elif x > 0 and y < 0:
        theta6 = math.atan(x10 / y10)
        theta6 = 180 + (theta6 * 180.0 / pi)
    else:
        if y == 0:
            y10 = -5
            theta6 = math.atan(x10 / y10)
            theta6 = theta6 * 180.0 / pi - 180.0
        elif y > 0:
            theta6 = math.atan(x10 / y10)
            theta6 = theta6 * 180.0 / pi
        else:
            theta6 = math.atan(x10 / y10)
            theta6 = theta6 * 180.0 / pi - 180.0

    # 计算投影和调整分量
    y_proj = math.sqrt(x10 * x10 + y10 * y10)
    y_adj = y_proj - _L3_01mm * math.cos(alpha * pi / 180)
    z_adj = z10 - _L0_01mm - _L3_01mm * math.sin(alpha * pi / 180)

    # 边界检查
    if z_adj < -_L0_01mm:
        return 1

    distance = math.sqrt(y_adj * y_adj + z_adj * z_adj)
    max_reach = _L1_01mm + _L2_01mm
    if distance > max_reach:
        return 2

    # theta5 (大臂)
    ccc = math.acos(y_adj / distance)
    bbb = (y_adj * y_adj + z_adj * z_adj + _L1_01mm * _L1_01mm - _L2_01mm * _L2_01mm) / (2 * _L1_01mm * distance)
    if abs(bbb) > 1:
        return 3
    zf_flag = -1 if z_adj < 0 else 1
    theta5 = math.degrees(ccc * zf_flag + math.acos(bbb))
    if not (0 <= theta5 <= 180):
        return 4

    # theta4 (肘关节)
    aaa = -(y_adj * y_adj + z_adj * z_adj - _L1_01mm * _L1_01mm - _L2_01mm * _L2_01mm) / (2 * _L1_01mm * _L2_01mm)
    if abs(aaa) > 1:
        return 5
    theta4 = 180.0 - math.degrees(math.acos(aaa))
    if not (-135 <= theta4 <= 135):
        return 6

    # theta3 (腕)
    theta3 = alpha - theta5 + theta4

    # 存储角度
    servo_angle[0] = theta6
    servo_angle[1] = theta5 - 90
    servo_angle[2] = theta4
    servo_angle[3] = theta3

    # 转换为 PWM
    servo_pwm[0] = int(1500 - 2000.0 * servo_angle[0] / 270.0)
    servo_pwm[1] = int(1500 + 2000.0 * servo_angle[1] / 270.0)
    servo_pwm[2] = int(1500 + 2000.0 * servo_angle[2] / 270.0)
    servo_pwm[3] = int(1500 - 2000.0 * servo_angle[3] / 270.0)
    servo_pwm[3] = 3000 - servo_pwm[3]

    return 0


def find_best_alpha(x, y, z, alpha_hint=None):
    """
    搜索最优 alpha 角度

    Args:
        x, y, z: 目标坐标 (mm)
        alpha_hint: 偏好 alpha (None=自动选最负有效值)

    Returns:
        (best_alpha, servo_pwm_tuple) 或 (None, None) 如果无解
    """
    best_alpha = None
    hint_matched = None
    hint_best_dist = 999

    for alpha in range(0, -136, -1):
        result = kinematics_analysis(x, y, z, alpha)
        if result == 0:
            if best_alpha is None or alpha < best_alpha:
                best_alpha = alpha
            if alpha_hint is not None:
                dist = abs(alpha - alpha_hint)
                if dist < hint_best_dist:
                    hint_best_dist = dist
                    hint_matched = alpha

    chosen = hint_matched if hint_matched is not None else best_alpha
    if chosen is None:
        return None, None

    kinematics_analysis(x, y, z, chosen)
    return chosen, tuple(servo_pwm)


def build_arm_cmd(pwms, time_ms=1000):
    """
    构建舵机控制命令字符串

    Args:
        pwms: 4 个 PWM 值 (servo_pwm[0..3])
        time_ms: 运动时间 (ms)

    Returns:
        命令字符串，如 "{#0P1500T1000!#1P1800T1000!...}"
    """
    cmd = '{'
    for i in range(4):
        pwm_val = max(500, min(2500, pwms[i]))
        cmd += f"#{i}P{pwm_val:04d}T{time_ms:04d}!"
    cmd += '}'
    return cmd


def build_arm_cmd_with_gripper(pwms, gripper_pwm, time_ms=1000):
    """
    构建包含夹爪的舵机控制命令

    Args:
        pwms: 4 个关节 PWM 值
        gripper_pwm: 夹爪 PWM (1500=中位, >1500闭合, <1500打开)
        time_ms: 运动时间

    Returns:
        命令字符串
    """
    cmd = '{'
    for i in range(4):
        pwm_val = max(500, min(2500, pwms[i]))
        cmd += f"#{i}P{pwm_val:04d}T{time_ms:04d}!"
    cmd += f"#5P{gripper_pwm:04d}T{time_ms:04d}!"
    cmd += '}'
    return cmd


# ── 正运动学 ──────────────────────────────────────────────────────────────────

def build_rotation_matrix(theta6_deg, alpha_deg):
    """构造 R_base_to_ee (3x3)"""
    t6 = math.radians(theta6_deg)
    a = math.radians(alpha_deg)
    ct6, st6 = math.cos(t6), math.sin(t6)
    ca, sa = math.cos(a), math.sin(a)
    R = np.array([
        [ct6,  sa * st6, ca * st6],
        [-st6, sa * ct6, ca * ct6],
        [0,    -ca,      sa],
    ], dtype=np.float64)
    return R


def compute_T_base_to_ee(x_mm, y_mm, z_mm, theta6_deg=None, alpha_deg=None):
    """
    计算末端执行器在基座系中的位姿 (4x4 齐次矩阵, mm 单位)

    Returns:
        T (4x4 ndarray) 或 None 如果无解
    """
    if alpha_deg is None or theta6_deg is None:
        a, t6 = find_best_alpha(x_mm, y_mm, z_mm)
        if a is None:
            return None
        if alpha_deg is None:
            alpha_deg = a
        if theta6_deg is None:
            theta6_deg = servo_angle[0]

    R = build_rotation_matrix(theta6_deg, alpha_deg)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.array([x_mm, y_mm, z_mm])
    return T


def compute_T_base_to_ee_from_angles(theta6_deg, theta5_deg, theta4_deg, theta3_deg):
    """
    从 4 个关节角度正向计算 T_base_to_ee (4x4, mm)

    Returns:
        T (4x4 ndarray)
    """
    alpha_deg = theta3_deg + theta5_deg - theta4_deg
    beta_deg = theta5_deg - theta4_deg

    t5 = math.radians(theta5_deg)
    b = math.radians(beta_deg)
    a = math.radians(alpha_deg)
    t6 = math.radians(theta6_deg)

    y_proj = L1 * math.cos(t5) + L2 * math.cos(b) + L3 * math.cos(a)
    z_mm = L0 + L1 * math.sin(t5) + L2 * math.sin(b) + L3 * math.sin(a)
    x_mm = y_proj * math.sin(t6)
    y_mm = y_proj * math.cos(t6)

    R = build_rotation_matrix(theta6_deg, alpha_deg)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.array([x_mm, y_mm, z_mm])
    return T


def pwms_to_angles(pwm0, pwm1, pwm2, pwm3):
    """servo_pwm[0..3] → (theta6, theta5, theta4, theta3) 度"""
    theta6 = (1500 - pwm0) * 270.0 / 2000.0
    theta5 = (pwm1 - 1500) * 270.0 / 2000.0 + 90.0
    theta4 = (pwm2 - 1500) * 270.0 / 2000.0
    theta3 = (pwm3 - 1500) * 270.0 / 2000.0
    return theta6, theta5, theta4, theta3


def T_mm_to_m(T):
    """将 T_base_to_ee 的平移部分从 mm 转换为 m"""
    Tm = T.copy()
    Tm[:3, 3] /= 1000.0
    return Tm


if __name__ == '__main__':
    print(f'=== arm_kinematics (L0={L0}mm) ===')
    for x, y, z, label in [(0, 200, 100, '中心'), (0, 300, 150, '远伸')]:
        a, pwms = find_best_alpha(x, y, z)
        if a is not None:
            print(f'[{label}] ({x},{y},{z}) alpha={a}° pwms={pwms}')
        else:
            print(f'[{label}] ({x},{y},{z}) 无解')
