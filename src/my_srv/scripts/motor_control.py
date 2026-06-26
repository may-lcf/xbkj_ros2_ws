#!/usr/bin/env python3
"""步进电机 TTL 串口控制脚本

支持位置模式(FD)、速度模式(F6)、立即停止(FE 98)、解除保护(0E 52)、回零(9A)、修改回零参数(4C)。
通讯: 115200, 8N1
"""

import serial
import sys
import time


class StepperMotor:
    """步进电机控制器"""

    # 一圈所需脉冲数: 1.8°步距角, 16细分
    PULSES_PER_REV = 3200

    def __init__(self, port="/dev/ttyUSB0", addr=0x01):
        self.ser = serial.Serial(port, baudrate=115200, timeout=1)
        self.addr = addr

    def close(self):
        self.ser.close()

    def _send(self, cmd: bytes) -> bytes:
        """发送命令并读取返回"""
        self.ser.write(cmd)
        time.sleep(0.05)
        return self.ser.read(4)

    def _parse_response(self, resp: bytes, func_code: int) -> str:
        """解析返回数据"""
        if len(resp) < 4:
            return f"错误: 返回数据不足({len(resp)}字节)"
        status = resp[2]
        status_map = {
            0x02: "命令执行成功",
            0x12: "已在零点或限位触发",
            0x22: "另一侧限位触发",
            0xE2: "参数错误或保护触发",
            0xEE: "命令格式错误",
            0x9F: "动作执行完成",
        }
        return status_map.get(status, f"未知状态码: 0x{status:02X}")

    # ---- 位置模式 (FD) ----
    def move(self, direction: int, speed_rpm: int, pulses: int,
             acc: int = 0, mode: int = 0, sync: int = 0) -> str:
        """位置模式运动

        Args:
            direction: 0=CW, 1=CCW
            speed_rpm: 速度 0~3000 RPM
            pulses: 脉冲数 (3200脉冲=1圈)
            acc: 加速度 0=直接启动, 1~255
            mode: 0=相对上一目标, 1=绝对位置, 2=相对当前位置
            sync: 0=立即执行, 1=先缓存
        """
        cmd = bytes([
            self.addr,
            0xFD,
            direction,
            (speed_rpm >> 8) & 0xFF, speed_rpm & 0xFF,
            acc,
            (pulses >> 24) & 0xFF, (pulses >> 16) & 0xFF,
            (pulses >> 8) & 0xFF, pulses & 0xFF,
            mode,
            sync,
            0x6B,
        ])
        resp = self._send(cmd)
        return self._parse_response(resp, 0xFD)

    def move_degrees(self, direction: int, speed_rpm: int, degrees: float,
                     acc: int = 0, mode: int = 0) -> str:
        """按角度运动 (便捷方法)"""
        pulses = int(degrees / 360 * self.PULSES_PER_REV)
        return self.move(direction, speed_rpm, pulses, acc, mode)

    def move_revolutions(self, direction: int, speed_rpm: int, revs: float,
                         acc: int = 0, mode: int = 0) -> str:
        """按圈数运动 (便捷方法)"""
        pulses = int(revs * self.PULSES_PER_REV)
        return self.move(direction, speed_rpm, pulses, acc, mode)

    # ---- 速度模式 (F6) ----
    def run(self, direction: int, speed_rpm: int, acc: int = 0, sync: int = 0) -> str:
        """速度模式持续运行

        Args:
            direction: 0=CW, 1=CCW
            speed_rpm: 速度 0~3000 RPM
            acc: 加速度 0=直接启动, 1~255
            sync: 0=立即执行, 1=先缓存
        """
        cmd = bytes([
            self.addr,
            0xF6,
            direction,
            (speed_rpm >> 8) & 0xFF, speed_rpm & 0xFF,
            acc,
            sync,
            0x6B,
        ])
        resp = self._send(cmd)
        return self._parse_response(resp, 0xF6)

    def stop_smooth(self, acc: int = 0) -> str:
        """平滑减速停止 (速度模式发速度=0)"""
        return self.run(0, 0, acc)

    # ---- 立即停止 (FE 98) ----
    def stop_now(self, sync: int = 0) -> str:
        """立即停止 (急停)"""
        cmd = bytes([self.addr, 0xFE, 0x98, sync, 0x6B])
        resp = self._send(cmd)
        return self._parse_response(resp, 0xFE)

    # ---- 解除保护 (0E 52) ----
    def clear_protection(self) -> str:
        """解除堵转/过热/过流保护"""
        cmd = bytes([self.addr, 0x0E, 0x52, 0x6B])
        resp = self._send(cmd)
        return self._parse_response(resp, 0x0E)

    # ---- 回零 (9A) ----
    def home(self, mode: int = 0x02, sync: int = 0) -> str:
        """触发回零

        Args:
            mode: 00=单圈就近, 01=单圈方向, 02=碰撞回零,
                  03=限位回零, 04=绝对零点, 05=掉电记忆
            sync: 0=立即执行, 1=先缓存
        """
        cmd = bytes([self.addr, 0x9A, mode, sync, 0x6B])
        resp = self._send(cmd)
        return self._parse_response(resp, 0x9A)

    def read_home_status(self) -> dict:
        """读取回零状态标志

        返回 dict:
            encoder_ready: 编码器是否就绪
            calibrated: 是否已校准
            homing: 是否正在回零
            failed: 是否回零失败
            overtemp: 是否过热保护
            overcurrent: 是否过流保护
            raw: 原始状态字节
        """
        cmd = bytes([self.addr, 0x3B, 0x6B])
        resp = self._send(cmd)
        if len(resp) < 4:
            return {"error": f"返回数据不足({len(resp)}字节)", "raw": 0}
        flags = resp[2]
        return {
            "encoder_ready": bool(flags & 0x01),
            "calibrated": bool(flags & 0x02),
            "homing": bool(flags & 0x04),
            "failed": bool(flags & 0x08),
            "overtemp": bool(flags & 0x10),
            "overcurrent": bool(flags & 0x20),
            "raw": flags,
        }

    def abort_home(self) -> str:
        """强制中断并退出回零操作"""
        cmd = bytes([self.addr, 0x9C, 0x48, 0x6B])
        resp = self._send(cmd)
        return self._parse_response(resp, 0x9C)

    def home_and_wait(self, mode: int = 0x02, poll_interval: float = 0.5,
                      timeout: float = 60.0) -> str:
        """触发回零并等待完成，失败时自动中断

        Args:
            mode: 回零模式，默认02(碰撞回零)
            poll_interval: 轮询间隔秒数
            timeout: 超时秒数
        """
        # 触发回零
        result = self.home(mode)
        if "命令执行成功" not in result:
            return f"回零触发失败: {result}"

        print("回零中...", end="", flush=True)
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(poll_interval)
            st = self.read_home_status()
            if "error" in st:
                print(f"\n状态读取错误: {st['error']}")
                self.abort_home()
                return "回零异常终止"

            if not st["homing"] and not st["failed"]:
                # homing=0, failed=0 → 回零成功
                print(" 完成")
                return "回零成功"

            if st["failed"]:
                print(" 失败", flush=True)
                self.abort_home()
                return "回零失败，已强制中断"

            if st["overtemp"] or st["overcurrent"]:
                print(f" 保护触发(过热={st['overtemp']}, 过流={st['overcurrent']})", flush=True)
                self.abort_home()
                return "回零异常(保护触发)，已强制中断"

            print(".", end="", flush=True)

        # 超时
        print(" 超时", flush=True)
        self.abort_home()
        return f"回零超时({timeout}s)，已强制中断"

    # ---- 修改回零参数 (功能码 4C) ----
    def set_home_params(self, store: bool = True, mode: int = 0x02,
                        direction: int = 0x01, speed: int = 30,
                        timeout_ms: int = 20000, detect_speed: int = 300,
                        detect_current: int = 1000, detect_time: int = 60,
                        auto_home: bool = True) -> str:
        """修改回零参数 (TTL 功能码 4C，19字节)

        参数:
            store: 是否保存到EEPROM(掉电不丢失)
            mode: 回零模式 00~05 (默认02=碰撞回零)
            direction: 方向 0=CW, 1=CCW
            speed: 回零速度 0~3000 RPM
            timeout_ms: 回零超时时间 毫秒
            detect_speed: 碰撞检测转速 RPM
            detect_current: 碰撞检测电流 mA
            detect_time: 碰撞检测时间 ms
            auto_home: 上电是否自动回零
        """
        cmd = bytes([
            self.addr,
            0x4C,
            0xAE if store else 0x00,
            mode & 0xFF,
            direction & 0xFF,
            (speed >> 8) & 0xFF, speed & 0xFF,
            (timeout_ms >> 24) & 0xFF, (timeout_ms >> 16) & 0xFF,
            (timeout_ms >> 8) & 0xFF, timeout_ms & 0xFF,
            (detect_speed >> 8) & 0xFF, detect_speed & 0xFF,
            (detect_current >> 8) & 0xFF, detect_current & 0xFF,
            (detect_time >> 8) & 0xFF, detect_time & 0xFF,
            0x01 if auto_home else 0x00,
            0x6B,
        ])
        resp = self._send(cmd)
        return self._parse_response(resp, 0x4C)


def main():
    if len(sys.argv) < 2:
        print("用法:")
        print("  python3 motor_control.py move <方向> <速度RPM> <脉冲数> [加速度]")
        print("  python3 motor_control.py degree <方向> <速度RPM> <角度> [加速度]")
        print("  python3 motor_control.py rev <方向> <速度RPM> <圈数> [加速度]")
        print("  python3 motor_control.py run <方向> <速度RPM> [加速度]")
        print("  python3 motor_control.py stop [smooth|now]")
        print("  python3 motor_control.py clear              # 解除堵转/过热/过流保护")
        print("  python3 motor_control.py home [模式]         # 触发回零并等待(默认碰撞回零02)")
        print("  python3 motor_control.py status              # 读取回零状态标志")
        print("  python3 motor_control.py sethome             # 设置回零参数(碰撞回零/CCW/30RPM/20s超时/1000mA/上电自动)")
        print()
        print("方向: 0=CW(顺时针), 1=CCW(逆时针)")
        print("示例:")
        print("  python3 motor_control.py rev 1 1500 10       # CCW 1500RPM 转10圈")
        print("  python3 motor_control.py degree 0 500 90     # CW 500RPM 转90度")
        print("  python3 motor_control.py run 1 800            # CCW 800RPM 持续运行")
        print("  python3 motor_control.py stop                 # 平滑停止")
        print("  python3 motor_control.py stop now             # 立即停止")
        print("  python3 motor_control.py clear                # 解除保护")
        print("  python3 motor_control.py home                 # 碰撞回零(自动等待)")
        print("  python3 motor_control.py home 03              # 限位回零(自动等待)")
        print("  python3 motor_control.py status               # 读取回零状态")
        print("  python3 motor_control.py sethome              # 设置回零参数")
        return

    motor = StepperMotor("/dev/ttyUSB0", addr=0x01)

    try:
        cmd = sys.argv[1]

        if cmd == "move":
            d, spd, pulses = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
            acc = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            print(motor.move(d, spd, pulses, acc))

        elif cmd == "degree":
            d, spd, deg = int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
            acc = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            print(motor.move_degrees(d, spd, deg, acc))

        elif cmd == "rev":
            d, spd, revs = int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
            acc = int(sys.argv[5]) if len(sys.argv) > 5 else 0
            print(motor.move_revolutions(d, spd, revs, acc))

        elif cmd == "run":
            d, spd = int(sys.argv[2]), int(sys.argv[3])
            acc = int(sys.argv[4]) if len(sys.argv) > 4 else 0
            print(motor.run(d, spd, acc))

        elif cmd == "stop":
            if len(sys.argv) > 2 and sys.argv[2] == "now":
                print(motor.stop_now())
            else:
                print(motor.stop_smooth())

        elif cmd == "clear":
            print(motor.clear_protection())

        elif cmd == "home":
            mode = int(sys.argv[2]) if len(sys.argv) > 2 else 0x02
            print(motor.home_and_wait(mode))

        elif cmd == "status":
            st = motor.read_home_status()
            if "error" in st:
                print(f"错误: {st['error']}")
            else:
                print(f"编码器就绪: {st['encoder_ready']}")
                print(f"已校准:     {st['calibrated']}")
                print(f"正在回零:   {st['homing']}")
                print(f"回零失败:   {st['failed']}")
                print(f"过热保护:   {st['overtemp']}")
                print(f"过流保护:   {st['overcurrent']}")
                print(f"原始字节:   0x{st['raw']:02X}")

        elif cmd == "sethome":
            print(motor.set_home_params())

        else:
            print(f"未知命令: {cmd}")
    finally:
        motor.close()


if __name__ == "__main__":
    main()
