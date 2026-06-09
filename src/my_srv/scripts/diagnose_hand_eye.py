#!/usr/bin/env python3
"""
诊断手眼标定与变换链的脚本。
用法示例（在 Pi 上运行）：
 python3 diagnose_hand_eye.py --u 424 --v 187 --depth 160 --pwms 1500 1432 1871 666

输出：将打印使用当前标定的 T_cam2gripper、物理替代值（如果提供）以及由实际 PWM 反算出的末端位姿下，像素点转换到基座系的坐标。便于对比并定位偏差来源（R、t 或 FK）。
"""
import sys
import os
import argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

try:
    import depth_utils
    import arm_fk
except Exception as e:
    print('无法导入本包内模块，尝试添加上级路径并重试...')
    parent = os.path.dirname(HERE)
    if parent not in sys.path:
        sys.path.insert(0, parent)
    try:
        import depth_utils
        import arm_fk
    except Exception as e2:
        print('导入失败:', e2)
        raise


def vec(mm):
    return np.array(mm, dtype=float)


def transform_point_cam_to_base(p_cam_m, R_c2g, t_c2g, T_gripper2base=None):
    # p_cam_m: (3,) in meters
    # R_c2g, t_c2g: camera->gripper
    p_gripper = R_c2g @ p_cam_m + t_c2g
    if T_gripper2base is None:
        return p_gripper
    # convert to base
    R_g2b = T_gripper2base[:3, :3]
    t_g2b = T_gripper2base[:3, 3]
    p_base = R_g2b @ p_gripper + t_g2b
    return p_base


def build_T_from_angles(angles_deg, origin_mm=None):
    # angles_deg = (theta6, theta5, theta4, theta3)
    # use arm_fk to compute T_base_to_ee; if arm_fk exposes compute_T_base_to_ee_from_angles use it
    th6, th5, th4, th3 = angles_deg
    if hasattr(arm_fk, 'compute_T_base_to_ee_from_angles'):
        T = arm_fk.compute_T_base_to_ee_from_angles(th6, th5, th4, th3)
        return T
    # fallback: if arm_fk.compute_T_base_to_ee accepts mm coordinates, we can't use it; raise
    raise RuntimeError('arm_fk lacks compute_T_base_to_ee_from_angles')


def pwms_to_angles(pwms):
    # expect iterable of 4 ints
    return arm_fk.pwms_to_angles(*pwms)


def load_hand_eye_cfg():
    # 直接从 YAML 读取 hand-eye 标定数据
    yaml_path = os.path.expanduser('~/ros2_ws/src/my_srv/config/hand_eye_calib.yaml')
    if os.path.isfile(yaml_path):
        import yaml
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        return {
            'R_cam2gripper': np.array(data['R_cam2gripper']),
            't_cam2gripper': np.array(data['t_cam2gripper']),
            'intrinsics': (417.8, 418.7, 338.1, 195.2)
        }

    # 退回默认值（仅用于离线诊断）
    print('⚠️ 未找到标定文件，使用内置后备 R / intrinsics 进行诊断（仅作参考）')
    R_def = np.array([[0.9939, 0.0926, -0.0602],
                      [-0.0903, 0.9951, 0.0395],
                      [0.0636, -0.0338, 0.9974]])
    t_def = np.array([0.0386, 0.1295, -0.0016])
    intrinsics_def = (417.8, 418.7, 338.1, 195.2)
    return {'R_cam2gripper': R_def, 't_cam2gripper': t_def, 'intrinsics': intrinsics_def}

    # 退回默认值（基于最近一次会话日志），仅用于离线诊断
    print('⚠️ 未找到标定文件，使用内置后备 R / intrinsics 进行诊断（仅作参考）')
    R_def = np.array([[0.9939, 0.0926, -0.0602],
                      [-0.0903, 0.9951, 0.0395],
                      [0.0636, -0.0338, 0.9974]])
    t_def = np.array([0.0386, 0.1295, -0.0016])
    intrinsics_def = (417.8, 418.7, 338.1, 195.2)
    return {'R_cam2gripper': R_def, 't_cam2gripper': t_def, 'intrinsics': intrinsics_def}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--u', type=int, required=True)
    p.add_argument('--v', type=int, required=True)
    p.add_argument('--depth', type=float, required=True, help='mm')
    p.add_argument('--pwms', nargs=4, type=int, required=True, help='4 pwm values')
    p.add_argument('--physical-t', nargs=3, type=float, help='override t_cam2gripper in meters')
    args = p.parse_args()

    u, v = args.u, args.v
    depth_mm = args.depth
    pwms = tuple(args.pwms)

    print('输入: u={}, v={}, depth={}mm, pwms={}'.format(u, v, depth_mm, pwms))

    cfg = load_hand_eye_cfg()
    R_c2g = np.array(cfg['R_cam2gripper'])
    t_c2g = np.array(cfg['t_cam2gripper'])
    print('\n当前标定 T_cam2gripper:')
    print(R_c2g)
    print('t_cam2gripper (m):', t_c2g)

    if args.physical_t:
        t_phys = np.array(args.physical_t)
        print('\n使用物理 override t_cam2gripper (m):', t_phys)
    else:
        t_phys = None

    # pixel -> camera coords (meters)
    intr = cfg.get('intrinsics')
    if intr is None:
        fx, fy, cx, cy = 417.8, 418.7, 338.1, 195.2
    else:
        fx, fy, cx, cy = intr
    z = depth_mm / 1000.0
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    p_cam_m = np.array([x, y, z], dtype=float)
    print('\n像素点在相机系 (m):', p_cam_m)

    # compute T_gripper2base from pwms
    angles = pwms_to_angles(pwms)
    print('\n由 PWM 反算关节角 (deg):', angles)
    try:
        T_g2b = build_T_from_angles(angles)
    except Exception as e:
        print('无法通过 arm_fk 生成 T_gripper2base:', e)
        T_g2b = None

    # 如果有从 arm_fk 得到的 T_g2b (mm)，先转换为米再使用
    T_g2b_m_for_transform = None
    if T_g2b is not None:
        T_g2b_m_for_transform = T_g2b.copy()
        T_g2b_m_for_transform[:3, 3] = T_g2b_m_for_transform[:3, 3] / 1000.0

    # Transform via calibrated R,t
    p_base_calib = transform_point_cam_to_base(p_cam_m, R_c2g, t_c2g, T_g2b_m_for_transform)
    print('\n通过标定 (R,t) -> 基座系 (m):', p_base_calib)

    if t_phys is not None:
        p_base_phys = transform_point_cam_to_base(p_cam_m, R_c2g, t_phys, T_g2b_m_for_transform)
        print('\n通过物理 t (与标定 R 组合) -> 基座系 (m):', p_base_phys)
        print('\n通过物理 t (与标定 R 组合) -> 基座系 (m):', p_base_phys)

    # Also show gripper origin position from T_g2b
    if T_g2b is not None:
        # arm_fk 返回的 T 单位为 mm，转换为 m
        T_g2b_m = T_g2b.copy()
        T_g2b_m[:3, 3] = T_g2b_m[:3, 3] / 1000.0
        origin_g_in_base = T_g2b_m[:3, 3]
        print('\n末端原点在基座系 (m) 从 PWM 反算:', origin_g_in_base)
        if T_g2b is not None:
            # compute vector from gripper origin to target
            vec_target = p_base_calib - origin_g_in_base
            print('末端原点 -> 目标 向量 (m):', vec_target)

    print('\n注意: 如果目标相对于末端在 x/y 上出现大偏差，说明 R 存在显著误差；如果偏差主要沿 z 轴，可能是 t 的问题或深度测量误差。')


if __name__ == '__main__':
    main()
