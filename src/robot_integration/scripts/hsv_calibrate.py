#!/usr/bin/env python3
"""
hsv_calibrate.py — HSV 黑色阈值标定工具（tkinter 版）

功能：
  1. 订阅 Aurora 930 RGB 相机图像
  2. 实时显示 HSV 黑色掩码效果
  3. 通过 tkinter 滑块调节 H/S/V 上下限
  4. 点击 Save 保存阈值到 line_follow_params.yaml
  5. 按 Ctrl+C 或关闭窗口退出

用法：
  # 先启动相机
  ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py

  # 运行标定
  python3 hsv_calibrate.py -o config/line_follow_params.yaml
"""

import os
import sys
import argparse
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tkinter as tk
from PIL import Image as PILImage, ImageTk


# ── 默认阈值 ──
DEFAULTS = {
    'H_min': 0, 'H_max': 180,
    'S_min': 0, 'S_max': 80,
    'V_min': 0, 'V_max': 80,
    'ROI_start': 200, 'ROI_end': 380,
    'ROI_left': 0, 'ROI_right': 640,
}


class HSVCalibrateNode(Node):
    def __init__(self, output_path):
        super().__init__('hsv_calibrate_node')

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.output_path = output_path
        self._img_lock = threading.Lock()

        # ── QoS（与相机发布者匹配：RELIABLE）──
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # ── 订阅 RGB ──
        self.create_subscription(
            Image, '/aurora/rgb/image_raw',
            self._rgb_callback, qos,
        )

        self.get_logger().info(
            '\033[1;36m[HSV Calibrate]\033[0m 标定工具已启动\n'
            '  等待 /aurora/rgb/image_raw ...\n'
            '  操作:\n'
            '    - 拖动滑块调节 HSV 阈值\n'
            '    - 点击 Save 保存阈值\n'
            '    - 点击 Reset 重置\n'
            '    - 关闭窗口或 Ctrl+C 退出'
        )

    def _rgb_callback(self, msg: Image):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._img_lock:
                self.latest_rgb = rgb
            if not hasattr(self, '_cb_count'):
                self._cb_count = 0
            self._cb_count += 1
            if self._cb_count <= 3 or self._cb_count % 60 == 0:
                self.get_logger().info(f'[HSV] 收到图像 #{self._cb_count}: {rgb.shape}')
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {e}')


class CalibrateGUI:
    """tkinter GUI 控制面板"""

    def __init__(self, node: HSVCalibrateNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('HSV Black Calibration')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self.running = True
        self.values = dict(DEFAULTS)

        # ── 滑块 ──
        self.sliders = {}
        slider_frame = tk.Frame(self.root)
        slider_frame.pack(side=tk.LEFT, padx=10, pady=10)

        params = [
            ('H_min', 0, 180), ('H_max', 0, 180),
            ('S_min', 0, 255), ('S_max', 0, 255),
            ('V_min', 0, 255), ('V_max', 0, 255),
            ('ROI_start', 0, 400), ('ROI_end', 0, 400),
            ('ROI_left', 0, 640), ('ROI_right', 0, 640),
        ]

        for name, lo, hi in params:
            frame = tk.Frame(slider_frame)
            frame.pack(fill=tk.X, pady=2)
            tk.Label(frame, text=name, width=10, anchor='w').pack(side=tk.LEFT)
            var = tk.IntVar(value=self.values[name])
            slider = tk.Scale(frame, from_=lo, to=hi, orient=tk.HORIZONTAL,
                              variable=var, length=250)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.sliders[name] = (var, slider)

        # ── 按钮 ──
        btn_frame = tk.Frame(slider_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        tk.Button(btn_frame, text='Save', command=self._save,
                  bg='green', fg='white', width=10).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text='Reset', command=self._reset,
                  bg='orange', width=10).pack(side=tk.LEFT, padx=5)

        # ── 信息标签 ──
        self.info_label = tk.Label(slider_frame, text='', font=('Courier', 10),
                                   anchor='w', justify=tk.LEFT)
        self.info_label.pack(fill=tk.X, pady=5)

        # ── 图像显示 ──
        self.img_label = tk.Label(self.root)
        self.img_label.pack(side=tk.RIGHT, padx=10, pady=10)

    def _get_values(self):
        """读取所有滑块当前值"""
        for name, (var, _) in self.sliders.items():
            self.values[name] = var.get()
        return self.values

    def _save(self):
        """保存阈值到 YAML"""
        import yaml
        v = self._get_values()

        params = {
            'line_follow_node': {
                'ros__parameters': {
                    'hsv_black_h_min': v['H_min'],
                    'hsv_black_h_max': v['H_max'],
                    'hsv_black_s_min': v['S_min'],
                    'hsv_black_s_max': v['S_max'],
                    'hsv_black_v_min': v['V_min'],
                    'hsv_black_v_max': v['V_max'],
                    'roi_y_start': v['ROI_start'],
                    'roi_y_end': v['ROI_end'],
                    'roi_x_left': v['ROI_left'],
                    'roi_x_right': v['ROI_right'],
                    'ground_depth_min_mm': 150,
                    'ground_depth_max_mm': 600,
                    'morph_kernel_size': 5,
                    'morph_open_iter': 2,
                    'morph_close_iter': 2,
                    'scan_rows': 10,
                    'min_road_ratio': 0.10,
                    'pid_kp': 0.30,
                    'pid_ki': 0.01,
                    'pid_kd': 0.15,
                    'max_steering': 0.10,
                    'integral_limit': 1.0,
                    'move_speed': 0.15,
                    'max_lost_frames': 30,
                    'publish_debug_image': True,
                    'auto_start': False,
                }
            }
        }

        out = os.path.abspath(self.node.output_path)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            yaml.dump(params, f, default_flow_style=False, allow_unicode=True)

        print(f'\n\033[1;32m[Saved]\033[0m 阈值已保存到: {out}')
        print(f'  H: [{v["H_min"]}, {v["H_max"]}]')
        print(f'  S: [{v["S_min"]}, {v["S_max"]}]')
        print(f'  V: [{v["V_min"]}, {v["V_max"]}]')
        print(f'  ROI Y: [{v["ROI_start"]}, {v["ROI_end"]}]')
        print(f'  ROI X: [{v["ROI_left"]}, {v["ROI_right"]}]')

    def _reset(self):
        """重置滑块"""
        for name, (var, _) in self.sliders.items():
            var.set(DEFAULTS[name])

    def _on_close(self):
        self.running = False
        self.root.destroy()

    def update_image(self):
        """定时更新图像（每 50ms）"""
        if not self.running:
            return

        with self.node._img_lock:
            rgb = self.node.latest_rgb

        if rgb is not None:
            v = self._get_values()
            display = self._process(rgb, v)
            # 转为 tkinter 可显示格式
            img_rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            img_pil = PILImage.fromarray(img_rgb)
            img_tk = ImageTk.PhotoImage(image=img_pil)
            self.img_label.configure(image=img_tk)
            self.img_label.image = img_tk  # 保持引用

        if self.running:
            self.root.after(50, self.update_image)

    def _process(self, rgb, v):
        """处理图像：HSV检测 + 叠加显示"""
        h, w = rgb.shape[:2]

        # HSV 黑色掩码
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower = np.array([v['H_min'], v['S_min'], v['V_min']], dtype=np.uint8)
        upper = np.array([v['H_max'], v['S_max'], v['V_max']], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # ROI 裁剪（上下 + 左右）
        roi_s = min(v['ROI_start'], h - 10)
        roi_e = min(v['ROI_end'], h - 1)
        roi_e = max(roi_e, roi_s + 10)
        roi_l = max(0, min(v['ROI_left'], w - 10))
        roi_r = max(roi_l + 10, min(v['ROI_right'], w))
        mask[:roi_s, :] = 0
        mask[roi_e:, :] = 0
        mask[:, :roi_l] = 0
        mask[:, roi_r:] = 0

        # 形态学
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 叠加显示（绿色高亮道路区域）
        overlay = rgb.copy()
        m = mask > 0
        blended = cv2.addWeighted(rgb, 0.5, np.full_like(rgb, (0, 200, 0)), 0.5, 0)
        overlay[m] = blended[m]

        # ROI 线（上下+左右）
        cv2.line(overlay, (0, roi_s), (w, roi_s), (255, 255, 0), 1)
        cv2.line(overlay, (0, roi_e), (w, roi_e), (255, 255, 0), 1)
        cv2.line(overlay, (roi_l, 0), (roi_l, h), (255, 255, 0), 1)
        cv2.line(overlay, (roi_r, 0), (roi_r, h), (255, 255, 0), 1)
        cv2.line(overlay, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

        # 中心线扫描
        center_pts = []
        for y in np.linspace(roi_s, roi_e, 10, dtype=int):
            row = mask[y, :]
            px = np.where(row > 0)[0]
            if len(px) > w * 0.05:
                cx = int(np.mean(px))
                center_pts.append((cx, y))
                cv2.circle(overlay, (cx, y), 4, (0, 0, 255), -1)

        if len(center_pts) > 1:
            for i in range(len(center_pts) - 1):
                cv2.line(overlay, center_pts[i], center_pts[i + 1], (0, 0, 255), 2)

        # 横向偏差
        lateral_err = 0.0
        if center_pts:
            cx_img = w / 2.0
            tw = 0.0
            we = 0.0
            for i, (cx, y) in enumerate(center_pts):
                wt = i + 1
                we += ((cx - cx_img) / cx_img) * wt
                tw += wt
            lateral_err = we / tw if tw > 0 else 0.0

        road_ratio = np.sum(m) / m.size if m.size > 0 else 0.0

        # 信息叠加
        cv2.putText(overlay, f"Road: {road_ratio:.1%}  Error: {lateral_err:+.3f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 更新信息标签
        info = (f"H: [{v['H_min']}, {v['H_max']}]\n"
                f"S: [{v['S_min']}, {v['S_max']}]\n"
                f"V: [{v['V_min']}, {v['V_max']}]\n"
                f"ROI Y: [{roi_s}, {roi_e}]\n"
                f"ROI X: [{roi_l}, {roi_r}]\n"
                f"Road: {road_ratio:.1%}\n"
                f"Error: {lateral_err:+.3f}\n"
                f"Center pts: {len(center_pts)}")
        self.info_label.configure(text=info)

        return overlay

    def run(self):
        """启动 GUI 主循环"""
        self.root.after(50, self.update_image)
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description='HSV 黑色阈值标定工具')
    parser.add_argument('--output', '-o', default='config/line_follow_params.yaml',
                        help='输出 YAML 文件路径')
    args = parser.parse_args()

    rclpy.init()
    node = HSVCalibrateNode(args.output)

    # ROS2 spin 在后台线程
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        gui = CalibrateGUI(node)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
