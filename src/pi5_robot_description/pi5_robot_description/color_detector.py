#!/usr/bin/env python3
"""
color_detector.py — LAB 颜色检测模块

基于 LAB 颜色空间进行物体检测，用于识别红色正方体和红色盒子。
"""

import os
import cv2
import numpy as np


# 颜色阈值文件搜索路径
_SEARCH_DIRS = [
    os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'config'),
    os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
    os.path.expanduser('~/ros2_ws'),
    os.getcwd(),
]


def _load_thresholds(filename):
    """
    加载 LAB 颜色阈值文件

    文件格式: L_min L_max A_min A_max B_min B_max (空格分隔)

    Returns:
        (lower_L, lower_A, lower_B, upper_L, upper_A, upper_B)
    """
    for d in _SEARCH_DIRS:
        fp = os.path.join(d, filename)
        if os.path.exists(fp):
            break
    else:
        raise FileNotFoundError(f'颜色阈值文件 {filename} 未找到')

    with open(fp) as f:
        nums = []
        for line in f:
            line = line.strip()
            if not line:
                continue
            for s in line.split():
                nums.append(int(s) if '.' not in s else float(s))

    # 文件格式: L_min L_max A_min A_max B_min B_max
    lo = (int(nums[0]), int(nums[2]), int(nums[4]))
    hi = (int(nums[1]), int(nums[3]), int(nums[5]))
    return lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]


class ColorDetector:
    """LAB 颜色检测器"""

    def __init__(self, threshold_dir=None):
        """
        Args:
            threshold_dir: 阈值文件目录 (None 则自动搜索)
        """
        if threshold_dir:
            _SEARCH_DIRS.insert(0, threshold_dir)

        # 加载红色阈值
        try:
            r = _load_thresholds('red.txt')
            self.lower_red = np.array(r[0:3], dtype=np.uint8)
            self.upper_red = np.array(r[3:6], dtype=np.uint8)
        except FileNotFoundError:
            # 使用默认红色 LAB 阈值
            self.lower_red = np.array([0, 139, 122], dtype=np.uint8)
            self.upper_red = np.array([255, 255, 255], dtype=np.uint8)

        # 尝试加载蓝色和绿色 (可选)
        try:
            b = _load_thresholds('blue.txt')
            self.lower_blue = np.array(b[0:3], dtype=np.uint8)
            self.upper_blue = np.array(b[3:6], dtype=np.uint8)
        except FileNotFoundError:
            self.lower_blue = np.array([0, 0, 100], dtype=np.uint8)
            self.upper_blue = np.array([255, 120, 255], dtype=np.uint8)

        try:
            g = _load_thresholds('green.txt')
            self.lower_green = np.array(g[0:3], dtype=np.uint8)
            self.upper_green = np.array(g[3:6], dtype=np.uint8)
        except FileNotFoundError:
            self.lower_green = np.array([0, 0, 0], dtype=np.uint8)
            self.upper_green = np.array([255, 120, 120], dtype=np.uint8)

    def detect(self, frame, lower_lab, upper_lab, min_area=150, min_side=15):
        """
        检测指定颜色的物体

        Args:
            frame: BGR 图像
            lower_lab: LAB 下界 (3,)
            upper_lab: LAB 上界 (3,)
            min_area: 最小轮廓面积
            min_side: 最小边长

        Returns:
            (area, center, rect) 或 None
            - area: 轮廓面积
            - center: (cx, cy) 中心坐标
            - rect: cv2.minAreaRect 结果 ((cx,cy), (w,h), angle)
        """
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(lab, lower_lab, upper_lab)

        # 形态学处理
        k = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if not contours:
            return None

        best = None
        best_area = -1
        for c in contours:
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), _ = rect
            area = cv2.contourArea(c)
            if min(w, h) < min_side or area < min_area:
                continue
            if area > best_area:
                best_area = area
                best = (area, (cx, cy), rect)

        return best

    def detect_red(self, frame, min_area=150):
        """检测红色物体"""
        return self.detect(frame, self.lower_red, self.upper_red, min_area)

    def detect_blue(self, frame, min_area=150):
        """检测蓝色物体"""
        return self.detect(frame, self.lower_blue, self.upper_blue, min_area)

    def detect_green(self, frame, min_area=150):
        """检测绿色物体"""
        return self.detect(frame, self.lower_green, self.upper_green, min_area)

    def calc_rotation_angle(self, rect):
        """
        计算 minAreaRect 的旋转角度，用于夹爪对齐

        Returns:
            角度 (度)，0 表示不需要旋转
        """
        if rect is None:
            return 0
        _, _, angle = rect
        if angle <= 10 or angle >= 80:
            return 0
        if 10 < angle < 45:
            return -angle
        return 90 - angle

    @staticmethod
    def draw_detection(frame, rect, color_bgr, label=''):
        """在图像上绘制检测结果"""
        if rect is None:
            return
        cx, cy = int(rect[0][0]), int(rect[0][1])
        box = cv2.boxPoints(rect)
        box_i = np.intp(box)
        cv2.drawContours(frame, [box_i], -1, color_bgr, 2)
        cv2.drawMarker(frame, (cx, cy), color_bgr, cv2.MARKER_CROSS, 18, 2)
        if label:
            cv2.putText(frame, label,
                        (int(box_i[:, 0].min()), max(int(box_i[:, 1].min()) - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2)
