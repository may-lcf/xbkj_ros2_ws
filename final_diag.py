#!/usr/bin/env python3
"""最终诊断：模拟 depth_color_sorting_node 的 _compute_world_target 完整链路"""
import os, sys, math, yaml, numpy as np
sys.path.insert(0, os.path.expanduser('~/ros2_ws/install/my_srv/lib/my_srv'))

import arm_fk

with open(os.path.expanduser('~/ros2_ws/src/my_srv/config/hand_eye_calib.yaml')) as f:
    data = yaml.safe_load(f)
R_c2g = np.array(data['R_cam2gripper'])
t_c2g = np.array(data['t_cam2gripper']).reshape(3, 1)
T_c2g = np.eye(4); T_c2g[:3,:3] = R_c2g; T_c2g[:3,3] = t_c2g.flatten()

fx, fy, cx, cy = 417.77, 418.71, 338.09, 195.22

def simulate_compute_world_target(u, v, depth_mm):
    """完全模拟 _compute_world_target 的流程"""
    # 1. pixel_to_3d
    z_m = depth_mm / 1000.0
    x_m = (u - cx) * z_m / fx
    y_m = (v - cy) * z_m / fy
    p_cam = np.array([x_m, y_m, z_m])
    
    # 2. transform_cam_to_gripper (current: NO y-flip)
    p_ee = (T_c2g @ np.append(p_cam, 1.0))[:3]
    
    # 3. compute T_gripper2base from PWM
    #    observation PWM = [1500, 1432, 1871, 666]
    angles = arm_fk.pwms_to_angles(1500, 1432, 1871, 666)
    T_mm = arm_fk.compute_T_base_to_ee_from_angles(*angles)
    T_g2b = arm_fk.T_mm_to_m(T_mm)
    
    # 4. p_base = T_g2b @ p_ee
    p_base = (T_g2b @ np.append(p_ee, 1.0))[:3]
    return p_cam, p_ee, T_g2b, p_base

print("观察位姿 FK: EE at", end=" ")
T_mm = arm_fk.compute_T_base_to_ee_from_angles(*arm_fk.pwms_to_angles(1500, 1432, 1871, 666))
print("(%.1f, %.1f, %.1f)mm" % tuple(T_mm[:3,3]))

# 目标: 实际运行结果（从日志）
# red:   world XYZ=(52,183,16)mm  实际抓取精准
# blue:  world XYZ=(30,161,5)mm   实际偏右很多
# green: world XYZ=(33,154,11)mm  实际偏右很多

for name, u, v, d in [("red",481,177,159), ("blue",437,268,165), ("green",459,291,157)]:
    p_cam, p_ee, T_g2b, p_base = simulate_compute_world_target(u, v, d)
    bx, by, bz = p_base * 1000
    
    # 分析: 如果 RED 是精准的，blue/green 偏右，意味着 blue/green 的实际 Y 应该更大
    # 实际 RED 精准抓取时 world_Y=183
    # 实际 BLUE 偏右 → 实际 Y 应该比 161 大？还是小？
    # "偏右" = x 方向偏？y 方向偏？取决于基座坐标系
    # base: x=右 y=前 z=上
    # "偏右" = base_x 太大，或 gripper 位置的 x 偏移
    # 如果物体实际在 y 方向更远，但计算出的 y 偏小 → 夹爪飞到了偏前的位置 → 从相机看物体偏右
    
    print(f"\n{name}: pix=({u},{v}) d={d}mm")
    print(f"  p_cam  = [{p_cam[0]:.4f} {p_cam[1]:.4f} {p_cam[2]:.4f}] m")
    print(f"  p_ee   = [{p_ee[0]:.4f} {p_ee[1]:.4f} {p_ee[2]:.4f}] m")
    print(f"  p_base = [{bx:.0f} {by:.0f} {bz:.0f}] mm")

# RED 基准：pix_y=177 接近 cy=195，当作"正确的基准"
# 用 RED 的 base_Y 和 BLUE/GREEN 做对比
_, _, _, red_base = simulate_compute_world_target(481, 177, 159)
print("\n" + "="*60)
print("关键对比：以 RED 为基准")
for name, u, v, d in [("blue",437,268,165), ("green",459,291,157)]:
    _, _, _, base = simulate_compute_world_target(u, v, d)
    dy = (base[1] - red_base[1]) * 1000
    dx = (base[0] - red_base[0]) * 1000
    print(f"  {name}: delta vs RED: dX={dx:.0f}mm  dY={dy:.0f}mm")

print("\n如果 BLUE/GREEN 偏右，说明计算出的 X 相对于 RED 的 X 变化了")
print("但实际三个物体在桌面上同一排的话，X 不应该有这么大差异")
print()
print("问题可能出在: pixel_to_3d 中 (v-cy)*depth/fy 计算的 cam_y")
print("相机 y 向下，物体在画面下方时 v > cy，cam_y > 0")
print("这个正 cam_y 通过 T_cam2gripper 后变成了什么方向？")
