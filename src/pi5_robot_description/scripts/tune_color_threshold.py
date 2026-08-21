#!/usr/bin/env python3
"""
tune_color_threshold.py --颜色阈值实时调节工具 (LAB 颜色空间)

功能:
  - 实时显示 Aurora 相机画面
  - 滑条调节 LAB 6 个阈值参数
  - 实时预览 mask 和检测结果
  - 按 's' 保存阈值到文件
  - 按 'q' 退出

用法:
  python3 tune_color_threshold.py
  python3 tune_color_threshold.py --color red
  python3 tune_color_threshold.py --camera /aurora/rgb/image_raw
"""

import argparse
import os
import sys
import cv2
import numpy as np

# 添加同级目录到路径
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
_PKG_DIR = os.path.dirname(_SCRIPT_DIR)
sys.path.insert(0, os.path.join(_PKG_DIR, 'pi5_robot_description'))

# 阈值文件目录
CONFIG_DIR = os.path.join(_PKG_DIR, 'config')


def load_thresholds(filepath):
    """加载阈值文件，返回 (L_min, L_max, A_min, A_max, B_min, B_max)"""
    if not os.path.exists(filepath):
        return 0, 255, 128, 255, 128, 255  # 默认红色 LAB
    with open(filepath) as f:
        nums = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            for s in line.split():
                nums.append(int(s) if '.' not in s else float(s))
    if len(nums) >= 6:
        return int(nums[0]), int(nums[1]), int(nums[2]), int(nums[3]), int(nums[4]), int(nums[5])
    return 0, 255, 128, 255, 128, 255


def save_thresholds(filepath, lo, hi):
    """保存阈值到文件"""
    with open(filepath, 'w') as f:
        f.write(f"{lo[0]} {hi[0]} {lo[1]} {hi[1]} {lo[2]} {hi[2]}\n")
    print(f"✅ 已保存到: {filepath}")
    print(f"   L: {lo[0]}-{hi[0]}, A: {lo[1]}-{hi[1]}, B: {lo[2]}-{hi[2]}")


def main():
    parser = argparse.ArgumentParser(description='颜色阈值实时调节工具')
    parser.add_argument('--color', default='red', help='颜色名称 (red/blue/green)')
    parser.add_argument('--camera', default='/aurora/rgb/image_raw', help='相机话题')
    parser.add_argument('--min-area', type=int, default=200, help='最小检测面积')
    args = parser.parse_args()

    # 阈值文件路径
    threshold_file = os.path.join(CONFIG_DIR, f'{args.color}.txt')
    print(f"阈值文件: {threshold_file}")

    # 加载当前阈值
    L_min, L_max, A_min, A_max, B_min, B_max = load_thresholds(threshold_file)
    print(f"当前阈值: L={L_min}-{L_max}, A={A_min}-{A_max}, B={B_min}-{B_max}")

    # 尝试 ROS2 话题获取图像
    use_ros = False
    cap = None
    bridge = None
    node = None
    latest_frame = None

    # 先尝试 USB 摄像头
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        print("使用 USB 摄像头 (设备 0)")
    else:
        cap.release()
        cap = None
        # 尝试 ROS2
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            import threading

            rclpy.init()
            node = rclpy.create_node('tune_color_threshold')
            bridge = CvBridge()

            def image_callback(msg):
                nonlocal latest_frame
                try:
                    latest_frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
                except Exception:
                    pass

            _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
            sub = node.create_subscription(Image, args.camera, image_callback, _qos)

            # 后台 spin
            spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
            spin_thread.start()

            use_ros = True
            print(f"使用 ROS2 话题: {args.camera}")
            print("等待图像...")

        except Exception as e:
            print(f"无法初始化 ROS2: {e}")
            print("请确保 ROS2 环境已配置，或连接 USB 摄像头")
            sys.exit(1)

    # 创建窗口
    win_name = f'Color Tuner - {args.color}'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1200, 600)

    # 创建滑条
    cv2.createTrackbar('L_min', win_name, L_min, 255, lambda x: None)
    cv2.createTrackbar('L_max', win_name, L_max, 255, lambda x: None)
    cv2.createTrackbar('A_min', win_name, A_min, 255, lambda x: None)
    cv2.createTrackbar('A_max', win_name, A_max, 255, lambda x: None)
    cv2.createTrackbar('B_min', win_name, B_min, 255, lambda x: None)
    cv2.createTrackbar('B_max', win_name, B_max, 255, lambda x: None)
    cv2.createTrackbar('min_area', win_name, args.min_area, 5000, lambda x: None)

    print("\n操作说明:")
    print("  - 拖动滑条调节阈值")
    print("  - 按 's' 保存阈值")
    print("  - 按 'r' 重置为默认值")
    print("  - 按 'q' 或 ESC 退出")
    print()

    frame_count = 0
    while True:
        # 获取图像
        frame = None
        if cap is not None:
            ret, frame = cap.read()
            if not ret:
                frame = None
        elif use_ros and latest_frame is not None:
            frame = latest_frame.copy()

        if frame is None:
            cv2.waitKey(100)
            continue

        frame_count += 1

        # 读取滑条值
        L_min = cv2.getTrackbarPos('L_min', win_name)
        L_max = cv2.getTrackbarPos('L_max', win_name)
        A_min = cv2.getTrackbarPos('A_min', win_name)
        A_max = cv2.getTrackbarPos('A_max', win_name)
        B_min = cv2.getTrackbarPos('B_min', win_name)
        B_max = cv2.getTrackbarPos('B_max', win_name)
        min_area = cv2.getTrackbarPos('min_area', win_name)

        # LAB 颜色空间转换
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lower = np.array([L_min, A_min, B_min], dtype=np.uint8)
        upper = np.array([L_max, A_max, B_max], dtype=np.uint8)
        mask = cv2.inRange(lab, lower, upper)

        # 形态学处理
        kernel = np.ones((3, 3), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

        # 检测轮廓
        result = frame.copy()
        contours = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        detected = False
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), angle = rect
            if min(w, h) < 15:
                continue

            # 绘制检测结果
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(result, [box], -1, (0, 255, 0), 2)
            cv2.drawMarker(result, (int(cx), int(cy)), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(result, f'A={area:.0f} ({cx:.0f},{cy:.0f})',
                        (int(box[:, 0].min()), max(int(box[:, 1].min()) - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            detected = True

        # 拼接显示: 原图 | mask | 检测结果
        h, w = frame.shape[:2]
        display_w = 400
        display_h = int(h * display_w / w)

        img_orig = cv2.resize(frame, (display_w, display_h))
        img_mask = cv2.resize(cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR), (display_w, display_h))
        img_result = cv2.resize(result, (display_w, display_h))

        # 添加标签
        cv2.putText(img_orig, 'Original', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img_mask, 'Mask', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        status = 'DETECTED' if detected else 'NO DETECT'
        color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(img_result, f'Result [{status}]', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 在 mask 上显示阈值信息
        info_lines = [
            f'L: {L_min}-{L_max}',
            f'A: {A_min}-{A_max}',
            f'B: {B_min}-{B_max}',
            f'min_area: {min_area}',
        ]
        for i, line in enumerate(info_lines):
            cv2.putText(img_mask, line, (10, 55 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)

        combined = np.hstack([img_orig, img_mask, img_result])
        cv2.imshow(win_name, combined)

        # 按键处理
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or key == 27:  # q 或 ESC
            break
        elif key == ord('s'):  # 保存
            lo = (L_min, A_min, B_min)
            hi = (L_max, A_max, B_max)
            save_thresholds(threshold_file, lo, hi)
        elif key == ord('r'):  # 重置
            cv2.setTrackbarPos('L_min', win_name, 0)
            cv2.setTrackbarPos('L_max', win_name, 255)
            cv2.setTrackbarPos('A_min', win_name, 128)
            cv2.setTrackbarPos('A_max', win_name, 255)
            cv2.setTrackbarPos('B_min', win_name, 128)
            cv2.setTrackbarPos('B_max', win_name, 255)
            print("已重置为默认值")

    # 清理
    cv2.destroyAllWindows()
    if cap is not None:
        cap.release()
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    print("退出阈值调节工具")


if __name__ == '__main__':
    main()
