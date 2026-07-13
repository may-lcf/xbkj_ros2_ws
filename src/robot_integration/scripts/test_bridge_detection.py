#!/usr/bin/env python3
"""
test_bridge_detection.py — 桥面检测算法测试脚本

功能：
  1. 读取深度图像文件或从相机获取
  2. 运行桥面检测算法
  3. 可视化检测结果

用法：
  # 从文件测试
  python3 test_bridge_detection.py --image depth_image.png

  # 从相机实时测试
  python3 test_bridge_detection.py --live
"""

import os
import sys
import argparse
import numpy as np
import cv2

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def detect_bridge(depth_image, params=None):
    """
    检测桥面（独立函数版本）

    Args:
        depth_image: mono16 深度图 (mm)
        params: 参数字典

    Returns:
        dict: 检测结果
    """
    # 默认参数
    if params is None:
        params = {
            'bridge_depth_min': 200,
            'bridge_depth_max': 500,
            'roi_y_start': 200,
            'roi_y_end': 400,
            'min_bridge_area': 500,
            'max_bridge_area': 50000,
            'max_bridge_width': 200,
            'min_aspect_ratio': 2.0,
            'morph_kernel_size': 5,
            'morph_iterations': 2,
        }

    h, w = depth_image.shape

    # 提取ROI
    roi_y_start = min(params['roi_y_start'], h - 2)
    roi_y_end = min(params['roi_y_end'], h)
    roi = depth_image[roi_y_start:roi_y_end, :]

    # 高度阈值分割
    bridge_mask = np.logical_and(
        roi >= params['bridge_depth_min'],
        roi <= params['bridge_depth_max']
    ).astype(np.uint8)

    # 形态学处理
    kernel_size = params['morph_kernel_size']
    iterations = params['morph_iterations']
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    bridge_mask = cv2.morphologyEx(bridge_mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
    bridge_mask = cv2.morphologyEx(bridge_mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)

    # 连通域分析
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        bridge_mask, connectivity=8
    )

    # 筛选桥面区域
    bridge_candidates = []
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        width = stats[i, cv2.CC_STAT_WIDTH]
        height = stats[i, cv2.CC_STAT_HEIGHT]

        if area < params['min_bridge_area'] or area > params['max_bridge_area']:
            continue
        if width > params['max_bridge_width']:
            continue
        aspect_ratio = height / width if width > 0 else 0
        if aspect_ratio < params['min_aspect_ratio']:
            continue

        bridge_candidates.append({
            'label': i,
            'area': area,
            'width': width,
            'height': height,
            'centroid': centroids[i],
            'stats': stats[i]
        })

    # 如果没有找到桥面
    if not bridge_candidates:
        return {
            'detected': False,
            'centerline': [],
            'center_x': w // 2,
            'lateral_error': 0.0,
            'heading_angle': 0.0,
            'area': 0,
            'width': 0.0,
            'confidence': 0.0,
            'mask': bridge_mask,
            'candidates': []
        }

    # 选择最佳桥面
    best_bridge = max(bridge_candidates, key=lambda x: x['area'])
    best_label = best_bridge['label']

    # 提取桥面像素
    bridge_pixels = np.where(labels == best_label)

    # 计算中心线
    centerline = []
    row_min = int(bridge_pixels[0].min())
    row_max = int(bridge_pixels[0].max())

    for row in range(row_min, row_max + 1):
        cols = bridge_pixels[1][bridge_pixels[0] == row]
        if len(cols) > 0:
            center_col = int(np.mean(cols))
            centerline.append((row + roi_y_start, center_col))

    # 计算航向角
    heading_angle = 0.0
    if len(centerline) > 10:
        rows = [p[0] for p in centerline]
        cols = [p[1] for p in centerline]
        slope, _ = np.polyfit(rows, cols, 1)
        heading_angle = np.arctan(slope)

    # 计算横向偏移
    if len(centerline) > 0:
        mid_idx = len(centerline) // 2
        bridge_center_x = centerline[mid_idx][1]
        lateral_error = (bridge_center_x - w / 2) / (w / 2)
    else:
        bridge_center_x = w // 2
        lateral_error = 0.0

    # 计算置信度
    confidence = min(1.0, best_bridge['area'] / 1000.0) * 0.5 + \
                 min(1.0, len(centerline) / 50.0) * 0.5

    return {
        'detected': True,
        'centerline': centerline,
        'center_x': bridge_center_x,
        'lateral_error': lateral_error,
        'heading_angle': heading_angle,
        'area': best_bridge['area'],
        'width': best_bridge['width'],
        'confidence': confidence,
        'mask': bridge_mask,
        'candidates': bridge_candidates
    }


def visualize_result(depth_image, result):
    """可视化检测结果"""
    # 深度图可视化
    depth_vis = np.clip(depth_image, 0, 1000).astype(np.uint8)
    depth_vis = cv2.equalizeHist(depth_vis)
    debug_image = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

    h, w = depth_image.shape

    # 绘制ROI区域
    cv2.rectangle(debug_image, (0, 200), (w-1, 400), (255, 255, 0), 1)

    if result['detected']:
        # 绘制中心线
        for point in result['centerline']:
            cv2.circle(debug_image, (point[1], point[0]), 2, (0, 0, 255), -1)

        # 绘制桥面中心
        cv2.circle(debug_image, (result['center_x'], h//2), 5, (0, 255, 0), -1)

        # 绘制图像中心线
        cv2.line(debug_image, (w//2, 0), (w//2, h-1), (255, 0, 0), 1)

        # 显示信息
        cv2.putText(debug_image,
                    f"Offset: {result['lateral_error']:.3f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_image,
                    f"Conf: {result['confidence']:.2f}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_image,
                    f"Area: {result['area']}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_image,
                    f"Angle: {np.degrees(result['heading_angle']):.1f} deg",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(debug_image,
                    "No bridge detected",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return debug_image


def main():
    parser = argparse.ArgumentParser(description='桥面检测算法测试')
    parser.add_argument('--image', type=str, help='深度图像文件路径')
    parser.add_argument('--live', action='store_true', help='从相机实时测试')
    parser.add_argument('--save', type=str, help='保存结果图像路径')
    args = parser.parse_args()

    if args.image:
        # 从文件加载
        if not os.path.exists(args.image):
            print(f"错误: 文件不存在 {args.image}")
            return

        # 读取图像
        depth_image = cv2.imread(args.image, cv2.IMREAD_UNCHANGED)
        if depth_image is None:
            print(f"错误: 无法读取图像 {args.image}")
            return

        # 转换为mono16
        if len(depth_image.shape) == 3:
            depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)

        print(f"图像尺寸: {depth_image.shape}")
        print(f"深度范围: {depth_image.min()} - {depth_image.max()} mm")

        # 检测桥面
        result = detect_bridge(depth_image)

        print(f"\n检测结果:")
        print(f"  检测到桥面: {result['detected']}")
        print(f"  桥面面积: {result['area']} 像素")
        print(f"  桥面宽度: {result['width']} 像素")
        print(f"  横向偏移: {result['lateral_error']:.3f}")
        print(f"  航向角: {np.degrees(result['heading_angle']):.1f} 度")
        print(f"  置信度: {result['confidence']:.2f}")
        print(f"  候选区域数: {len(result['candidates'])}")

        # 可视化
        debug_image = visualize_result(depth_image, result)

        # 保存结果
        if args.save:
            cv2.imwrite(args.save, debug_image)
            print(f"\n结果已保存到: {args.save}")

        # 显示
        cv2.imshow('Bridge Detection', debug_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    elif args.live:
        print("实时模式需要ROS2环境，请使用以下命令启动:")
        print("  ros2 launch robot_integration balance_beam.launch.py debug:=true")

    else:
        print("请指定 --image 或 --live 参数")
        parser.print_help()


if __name__ == '__main__':
    main()
