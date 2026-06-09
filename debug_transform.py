#!/usr/bin/env python3
"""深度分析：三种颜色变换链对比"""
import numpy as np, yaml, os, math

with open(os.path.expanduser("~/ros2_ws/src/my_srv/config/hand_eye_calib.yaml")) as f:
    data = yaml.safe_load(f)
R = np.array(data["R_cam2gripper"])
t = np.array(data["t_cam2gripper"]).reshape(3)

fx, fy, cx, cy = 417.77, 418.71, 338.09, 195.22

def pixel_to_3d(u, v, d_mm):
    z = d_mm / 1000.0
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])

def transform_full(p_cam):
    p = np.append(p_cam, 1.0)
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = t
    return (T @ p)[:3]

# FK for observation pose [1500,1432,1871,666]
theta5 = math.radians(80.82)
theta4 = math.radians(50.085)
theta3 = math.radians(-112.59)
alpha = theta3 + theta5 - theta4

L0, L1, L2, L3 = 111.0, 105.0, 88.0, 178.0
y_plane = L1*math.cos(theta5) + L2*math.cos(theta5-theta4) + L3*math.cos(alpha)
z_mm = L0 + L1*math.sin(theta5) + L2*math.sin(theta5-theta4) + L3*math.sin(alpha)

R_b2e = np.array([
    [1, 0, 0],
    [0, math.sin(alpha), math.cos(alpha)],
    [0, -math.cos(alpha), math.sin(alpha)]
])
t_b2e = np.array([0, y_plane, z_mm]) / 1000.0
T_b2e = np.eye(4); T_b2e[:3,:3] = R_b2e; T_b2e[:3,3] = t_b2e

print("FK 观察位姿: x=0 y=%.3f z=%.3f alpha=%.1f" % (y_plane, z_mm, math.degrees(alpha)))

tests = [
    ("red",   481, 177, 159),
    ("blue",  437, 268, 165),
    ("green", 459, 291, 157),
]

print("\n" + "=" * 85)
print("%-6s %10s %6s %10s %10s %10s %10s %18s" % ("Color", "pix", "depth", "cam_x", "cam_y", "ee_x", "ee_y", "base_XYZ"))
print("-" * 85)

for name, u, v, d in tests:
    cam = pixel_to_3d(u, v, d)
    ee = transform_full(cam)
    p_base = T_b2e @ np.append(ee, 1.0)
    bx, by, bz = p_base[:3] * 1000
    print("%-6s (%3d,%3d) %4dmm  %8.4f %8.4f  %8.4f %8.4f  (%5.0f,%5.0f,%5.0f)mm" %
          (name, u, v, d, cam[0], cam[1], ee[0], ee[1], bx, by, bz))

print("=" * 85)
print()
print("【关键发现】")
print("  cam_y (相机 y 向下):  red=%.0fmm   blue=%+.0fmm   green=%+.0fmm" %
      ((177-cy)*159/fy, (268-cy)*165/fy, (291-cy)*157/fy))
print()
print("  红色 pix_y=177 接近中心 cy=195，cam_y≈0 → 无论符号正确与否，影响为0")
print("  蓝色 pix_y=268 距中心 +73px → cam_y≈+29mm")
print("  绿色 pix_y=291 距中心 +96px → cam_y≈+36mm")
print()
print("  如果 cam_y 需要取反（相机y向下 vs 夹爪y向上）：")
print("    → 红色不受影响，蓝色/绿色产生 ~60-70mm Y方向偏移")
print("    → 这会表现为 '抓取偏右/偏后'")
print()
print("  结论：(v-cy)*depth/fy 算出的 cam_y 必须确认正负号")
