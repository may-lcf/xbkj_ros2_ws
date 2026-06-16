#!/usr/bin/env python3
"""
lab_tuner.py — LAB 色彩空间颜色阈值调节工具

支持两种模式:
  1. 单目 USB 摄像头 (默认):  python3 lab_tuner.py
  2. 深度相机 ROS2 话题:     python3 lab_tuner.py --depth
     （需先 source ROS2 环境 + deptrum 驱动运行中）

功能:
  - 选择要调节的颜色（红/蓝/绿）
  - 滑动条实时调节 LAB 上下阈值
  - 按 Enter 保存到对应颜色的 .txt 文件
  - 实时显示叠加掩膜效果
"""

import os, sys, time, threading, argparse
import numpy as np, cv2

# ── 阈值文件路径 ──
def find_target_dir():
    dirs = [
        os.path.expanduser('~/ros2_ws/src/my_srv/scripts'),
        os.path.dirname(os.path.abspath(__file__)),
        os.getcwd(),
    ]
    for d in dirs:
        if os.path.isdir(d):
            return d
    return dirs[-1]

TARGET_DIR = find_target_dir()
COLORS = ['red', 'blue', 'green']
COLOR_FILES = {c: f'{c}.txt' for c in COLORS}
DEFAULT_THRESHOLDS = {
    'red':   (0, 145, 100, 255, 255, 200),
    'blue':  (0,  80,   0, 255, 135, 115),
    'green': (0,  84,   0, 255, 116, 255),
}


def load_t(color):
    fp = os.path.join(TARGET_DIR, COLOR_FILES[color])
    if os.path.exists(fp):
        try:
            with open(fp) as f:
                p = f.readline().strip().split()
                if len(p) == 6:
                    return tuple(int(x) for x in p)
        except Exception:
            pass
    return DEFAULT_THRESHOLDS[color]


def save_t(color, t):
    fp = os.path.join(TARGET_DIR, COLOR_FILES[color])
    os.makedirs(TARGET_DIR, exist_ok=True)
    with open(fp, 'w') as f:
        f.write(' '.join(str(int(x)) for x in t) + '\n')
    print(f'\033[1;32m✅ [{color}]\033[0m 已保存 → {fp}')
    print(f'   L={t[0]}-{t[1]}  A={t[2]}-{t[3]}  B={t[4]}-{t[5]}')


# ═══════════════════════════════════════════════════════════════════════════════

class LABTuner:
    def __init__(self, win='LAB Tuner', use_depth=False, camera_idx=0):
        self.use_depth = use_depth
        self.current = 'red'
        self.thresholds = list(load_t(self.current))
        self.win = win
        self._setup_capture(camera_idx)
        self._setup_ros(use_depth)
        self._setup_gui()
        self._wait_frame()
        self._print_info()

    def _setup_capture(self, camera_idx):
        if not self.use_depth:
            self.cap = cv2.VideoCapture(camera_idx)
            if not self.cap.isOpened():
                raise RuntimeError(f'无法打开摄像头 /dev/video{camera_idx}')

    def _setup_ros(self, use_depth):
        self.node, self.pub, self.spin_thread = None, None, None
        if not use_depth:
            return
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        from rclpy.executors import MultiThreadedExecutor
        rclpy.init(args=[])
        self.node = Node('lab_tuner')
        self.bridge = CvBridge()
        self._frame = None
        self._lock = threading.Lock()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.node.create_subscription(Image, '/aurora/rgb/image_raw', self._ros_cb, qos)
        self.pub = self.node.create_publisher(Image, '/lab_tuner/image_debug', 10)
        exec_ = MultiThreadedExecutor()
        exec_.add_node(self.node)
        self.spin_thread = threading.Thread(target=exec_.spin, daemon=True)
        self.spin_thread.start()

    def _ros_cb(self, msg):
        try:
            with self._lock:
                self._frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _wait_frame(self):
        if self.use_depth:
            print('[等待] 深度相机 RGB 帧...')
            for _ in range(150):
                with self._lock:
                    if self._frame is not None:
                        print('[就绪] 深度相机帧已接收')
                        return
                time.sleep(0.1)
            raise RuntimeError('无法获取相机图像，请确保 deptrum 驱动在运行')

    def _print_info(self):
        src = 'Aurora 930 Depth' if self.use_depth else 'USB Camera'
        print(f'\n{"="*55}')
        print(f'\033[1;36m  LAB 颜色阈值调节 — {src}\033[0m')
        print(f'{"="*55}')
        print(f'  当前: \033[1;33m{self.current}\033[0m  |  目标目录: {TARGET_DIR}')
        print(f'  [r]Red  [b]Blue  [g]Green  [Enter]保存  [q]退出\n')

    def _setup_gui(self):
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win, 900, 520)
        for c in ('L', 'A', 'B'):
            for mn in ('min', 'max'):
                cv2.createTrackbar(f'{c}_{mn}', self.win, 0, 255, lambda x: None)
        self._update_sliders()

    def _read_sliders(self):
        return [cv2.getTrackbarPos(f'{c}_{mn}', self.win)
                for c in ('L', 'A', 'B') for mn in ('min', 'max')]

    def _update_sliders(self):
        for i, (c, mn) in enumerate([(c, mn) for c in ('L', 'A', 'B') for mn in ('min', 'max')]):
            cv2.setTrackbarPos(f'{c}_{mn}', self.win, self.thresholds[i])

    def _switch(self, color):
        self.current = color
        self.thresholds = list(load_t(color))
        self._update_sliders()
        t = self.thresholds
        print(f'\033[1;33m  → {color}\033[0m  '
              f'L={t[0]}-{t[1]}  A={t[2]}-{t[3]}  B={t[4]}-{t[5]}')

    def _save(self):
        self.thresholds = self._read_sliders()
        save_t(self.current, self.thresholds)

    def get_frame(self):
        if self.use_depth:
            with self._lock:
                return self._frame.copy() if self._frame is not None else None
        else:
            ret, frame = self.cap.read()
            return frame if ret else None

    def run(self):
        while True:
            frame = self.get_frame()
            if frame is None:
                time.sleep(0.03)
                continue

            frame = cv2.flip(frame, -1)
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            t = self._read_sliders()
            lo = np.array(t[0::2], dtype=np.uint8)
            hi = np.array(t[1::2], dtype=np.uint8)
            mask = cv2.inRange(lab, lo, hi)

            k = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

            # HUD overlay
            display = frame.copy()
            h, w = display.shape[:2]
            ovl = display.copy()
            cv2.rectangle(ovl, (0, 0), (w, 72), (25, 25, 25), -1)
            display = cv2.addWeighted(ovl, 0.5, display, 0.5, 0)

            txt = (f'{self.current.upper()}  |  '
                   f'L:{t[0]}-{t[1]}  A:{t[2]}-{t[3]}  B:{t[4]}-{t[5]}')
            cv2.putText(display, txt, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            cv2.putText(display, '[R]Red [B]Blue [G]Green [Enter]Save [Q]Quit',
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

            # 掩膜叠加
            mrg = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mrg[:, :, 2] = mask
            display = cv2.addWeighted(display, 0.55, mrg, 0.45, 0)

            cv2.imshow(self.win, display)

            # 发布调试图到 ROS2 话题
            if self.pub:
                try:
                    self.pub.publish(self.bridge.cv2_to_imgmsg(display, 'bgr8'))
                except Exception:
                    pass

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                self._switch('red')
            elif key == ord('b'):
                self._switch('blue')
            elif key == ord('g'):
                self._switch('green')
            elif key == 13:
                self._save()

    def close(self):
        cv2.destroyAllWindows()
        if self.use_depth:
            if self.node:
                self.node.destroy_node()
            import rclpy
            rclpy.shutdown()
        else:
            if hasattr(self, 'cap') and self.cap:
                self.cap.release()


# ═══════════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(description='LAB Color Threshold Tuner')
    p.add_argument('--depth', action='store_true', help='使用深度相机 (Aurora 930)')
    p.add_argument('--camera', type=int, default=0, help='USB 摄像头设备编号')
    args = p.parse_args()

    try:
        tuner = LABTuner(win='LAB Tuner', use_depth=args.depth, camera_idx=args.camera)
        tuner.run()
    except RuntimeError as e:
        print(f'\033[1;31m[错误]\033[0m {e}')
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if 'tuner' in locals():
            tuner.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
