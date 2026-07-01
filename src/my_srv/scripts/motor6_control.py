#!/usr/bin/env python3
"""motor6_control.py — 6号电机串口控制脚本

协议格式: #<设备号>P<目标值>B<执行时间>!
  设备号: 006（6号电机）
  目标值: 1500=停止/中位, >1500正转, <1500反转
  执行时间: 毫秒

用法:
  python3 motor6_control.py forward <速度> [时间ms]    # 正转（速度: 1501~2000）
  python3 motor6_control.py reverse <速度> [时间ms]    # 反转（速度: 1000~1499）
  python3 motor6_control.py stop                       # 停止
  python3 motor6_control.py run <目标值> [时间ms]       # 直接发送目标值

示例:
  python3 motor6_control.py forward 1520 5000          # 正转，目标值1520，持续5秒
  python3 motor6_control.py reverse 1400 3000          # 反转，目标值1400，持续3秒
  python3 motor6_control.py stop                       # 立即停止
"""

import serial
import sys
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
MOTOR_ID = "006"
STOP_VALUE = 1500
DEFAULT_TIME_MS = 5000


def send_command(ser, value, duration_ms):
    """发送电机控制命令: #006P<value>B<duration>!"""
    cmd = f"#{MOTOR_ID}P{value}B{duration_ms:04d}!"
    ser.write(cmd.encode("ascii"))
    print(f"发送: {cmd}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return

    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    try:
        action = sys.argv[1]

        if action == "forward":
            value = int(sys.argv[2]) if len(sys.argv) > 2 else 1520
            duration = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TIME_MS
            if value <= STOP_VALUE:
                print(f"错误: forward 速度应 > {STOP_VALUE}，收到 {value}")
                return
            send_command(ser, value, duration)

        elif action == "reverse":
            value = int(sys.argv[2]) if len(sys.argv) > 2 else 1480
            duration = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TIME_MS
            if value >= STOP_VALUE:
                print(f"错误: reverse 速度应 < {STOP_VALUE}，收到 {value}")
                return
            send_command(ser, value, duration)

        elif action == "stop":
            send_command(ser, STOP_VALUE, 0000)

        elif action == "run":
            value = int(sys.argv[2])
            duration = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_TIME_MS
            send_command(ser, value, duration)

        else:
            print(f"未知命令: {action}")
            print(__doc__)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
