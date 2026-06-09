#!/usr/bin/env python3
"""深度诊断：使用真实 arm_fk + depth_utils 变换链对比三种颜色"""
import os, sys, math, numpy as np, yaml
sys.path.insert(0, os.path.expanduser('~/ros2_ws/install/my_srv/lib/my_srv'))

import arm_fk

with open(os.path.expanduser('~/ros2_ws/src/my_srv/config/hand_eye_calib.yaml')) as f:
    data = yaml.safe_load(f)
R = np.array(data['R_cam2gripper'])
t = np.array(data['t_cam2gripper']).reshape(3)
T_c2g = np.eye(4); T_c2g[:3,:3] = R; T_c2g[:3,3] = t

fx, fy, cx, cy = 417.77, 418.71, 338.09, 195.22

def pixel_to_3d(u, v, d_mm):
    z = d_mm / 1000.0
    return np.array([(u-cx)*z/fx, (v-cy)*z/fy, z])

def xform_cam_to_base(p_cam, flip_y=False):
    pc = p_cam.copy()
    if flip_y:
        pc[1] = -pc[1]
    p_ee = (T_c2g @ np.append(pc, 1.0))[:3]
    T_b2e = arm_fk.compute_T_base_to_ee_from_angles(*pwms_angles)
    T_m = arm_fk.T_mm_to_m(T_b2e)
    return (T_m @ np.append(p_ee, 1.0))[:3]

# 观察位姿 PWM
pwms = (1500, 1432, 1871, 666)
pwms_angles = arm_fk.pwms_to_angles(*pwms)

# 直接 FK 验证
T_mm = arm_fk.compute_T_base_to_ee_from_angles(*pwms_angles)
ee_pos_mm = T_mm[:3, 3]
alpha = math.radians(pwms_angles[3] + pwms_angles[1] - pwms_angles[2])

print(f"PWM: {pwms}")
print(f"关节角: θ6={pwms_angles[0]:.1f}° θ5={pwms_angles[1]:.1f}° θ4={pwms_angles[2]:.1f}° θ3={pwms_angles[3]:.1f}°")
print(f"FK EE位置: ({ee_pos_mm[0]:.1f}, {ee_pos_mm[1]:.1f}, {ee_pos_mm[2]:.1f})mm")
print(f"FK α = {math.degrees(alpha):.1f}°")
print()

tests = [
    ("red",   481, 177, 159),
    ("blue",  437, 268, 165),
    ("green", 459, 291, 157),
]

print("=" * 100)
print(f"{'':6s} | {'cam (m)':>22s} | {'ee NO-flip (m)':>22s} | {'ee WITH-flip (m)':>22s} | {'base NO-flip':>20s} | {'base WITH-flip':>20s}")
print("-" * 100)

for name, u, v, d in tests:
    cam = pixel_to_3d(u, v, d)
    ee_nf = (T_c2g @ np.append(cam, 1.0))[:3]
    cam_f = cam.copy(); cam_f[1] = -cam_f[1]
    ee_wf = (T_c2g @ np.append(cam_f, 1.0))[:3]
    base_nf = xform_cam_to_base(cam, flip_y=False) * 1000
    base_wf = xform_cam_to_base(cam, flip_y=True) * 1000
    print(f"{name:6s} | {cam[0]:6.4f} {cam[1]:7.4f} {cam[2]:6.4f} | {ee_nf[0]:6.4f} {ee_nf[1]:7.4f} {ee_nf[2]:6.4f} | {ee_wf[0]:6.4f} {ee_wf[1]:7.4f} {ee_wf[2]:6.4f} | ({base_nf[0]:5.0f},{base_nf[1]:5.0f},{base_nf[2]:5.0f}) | ({base_wf[0]:5.0f},{base_wf[1]:5.0f},{base_wf[2]:5.0f})")

print("=" * 100)
print()
print("分析：")
for name, u, v, d in tests:
    cam_y = (v - cy) * d / fy
    print(f"  {name}: pix_y={v} (中心={cy}) → cam_y = {cam_y:.0f}mm" + 
          (" ← 接近0，符号翻转不影响" if abs(cam_y) < 10 else " ← 远离0，符号翻转影响显著"))
print()
print("比较 NO-flip vs WITH-flip 的 base_Y:")
for name, u, v, d in tests:
    cam = pixel_to_3d(u, v, d)
    by_nf = xform_cam_to_base(cam, flip_y=False)[1] * 1000
    by_wf = xform_cam_to_base(cam, flip_y=True)[1] * 1000
    print(f"  {name}: NO-flip base_Y={by_nf:.0f}  WITH-flip base_Y={by_wf:.0f}  (diff={by_wf-by_nf:.0f}mm)")
