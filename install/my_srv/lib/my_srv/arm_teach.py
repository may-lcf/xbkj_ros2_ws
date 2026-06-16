#!/usr/bin/env python3
"""
arm_teach.py - 机械臂手动示教工具

功能：
  - 向 000~005 号舵机发送释放阻力指令，使其可被手动扳动
  - 循环依次读取 000~005 号舵机当前位置（收到反馈后再读取下一个，005 之后回到 000）
  - 命令行参数 --lock：读取当前各关节位置后发送锁定指令，恢复舵机阻力并退出

用法：
  python3 arm_teach.py               # 默认：小阻力释放 + 循环读取位置
  python3 arm_teach.py --hard        # 大阻力释放 + 循环读取位置
  python3 arm_teach.py --lock        # 恢复舵机阻力（读取当前位后锁定）
"""

import sys
import os
import re
import time
import argparse
import threading

# 将 z_uart.py 所在的 /home/lcf/OpenCV 目录加入搜索路径（独立运行时备用）
_OPENCV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', '..', '..', '..', '..', 'OpenCV')
_OPENCV_DIR = os.path.normpath(_OPENCV_DIR)
if _OPENCV_DIR not in sys.path:
    sys.path.insert(0, _OPENCV_DIR)

import z_uart
from z_uart import uart_send_str, setup_uart, close_uart

# ─── 配置 ──────────────────────────────────────────────────────────────────
SERVO_COUNT  = 6      # 舵机编号范围 000 ~ 005
READ_TIMEOUT = 1.5    # 等待单个舵机反馈的超时时间（秒）
LOCK_TIME    = 200    # 恢复阻力时发送的运动时间（ms），避免位置突变

# ROS2 → RViz2 关节名映射（对应 URDF 中 joint1~joint7）
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
# ───────────────────────────────────────────────────────────────────────────


def _sid(idx: int) -> str:
    """返回 3 位补零的舵机编号字符串，例如 2 -> '002'"""
    return f"{idx:03d}"


def _wait_response(timeout: float = READ_TIMEOUT) -> str | None:
    """
    等待串口接收线程（z_uart.serialEvent）完成一帧数据。
    返回接收到的字符串，超时返回 None。
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if z_uart.uart_get_ok:
            data = z_uart.uart_receive_buf
            z_uart.uart_receive_buf = ''
            z_uart.uart_get_ok = 0
            return data
        time.sleep(0.005)
    return None


def _parse_pwm(response: str) -> int | None:
    """
    从响应中解析 PWM 值，支持带命令回显的格式。
    例如 '#002P1350!' -> 1350
         '#002PRAD!#002P1350!' -> 1350（跳过回显帧）
    """
    # 使用正则查找 #XXXP<纯数字>! 格式，忽略回显帧中的 PRAD
    m = re.search(r'#\d{3}P(\d+)!', response)
    return int(m.group(1)) if m else None


def release_all(mode: str = 'K') -> None:
    """
    向 000~005 号舵机依次发送释放阻力指令并等待响应。
    mode='K' → 小阻力（#XXXPULK!）
    mode='M' → 大阻力（#XXXPULM!）
    """
    cmd_suffix = f"PUL{mode}"
    print(f"正在释放舵机阻力（{'小阻力 PULK' if mode == 'K' else '大阻力 PULM'}）...")
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        cmd = f"#{sid}{cmd_suffix}!"
        uart_send_str(cmd)
        resp = _wait_response(timeout=0.8)
        if resp:
            print(f"  舵机 {sid} <- {resp.strip()}")
        else:
            print(f"  舵机 {sid} 无响应（已发送 {cmd}）")
    print("阻力释放完毕，现在可以用手扳动机械臂。\n")


def _wait_prad_response(timeout: float = READ_TIMEOUT) -> str | None:
    """
    等待舵机位置响应（#XXXP<数字>! 格式）。
    若先收到命令回显帧（#XXXPRAD!），则清除并继续等待真实响应。
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if z_uart.uart_get_ok:
            data = z_uart.uart_receive_buf
            z_uart.uart_receive_buf = ''
            z_uart.uart_get_ok = 0
            # 检查是否包含真实位置数据（P后跟数字）
            if re.search(r'#\d{3}P\d+!', data):
                return data
            # 否则只是命令回显，继续等待
        time.sleep(0.005)
    return None


def read_once() -> dict[str, int | None]:
    """
    依次读取 000~005 号舵机当前位置，收到反馈后才请求下一个。
    返回 {id_str: pwm_value or None}
    """
    positions: dict[str, int | None] = {}
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        uart_send_str(f"#{sid}PRAD!")
        resp = _wait_prad_response()
        if resp:
            pwm = _parse_pwm(resp)
            positions[sid] = pwm
        else:
            positions[sid] = None
    return positions


def print_positions(positions: dict[str, int | None]) -> None:
    parts = []
    for sid, pwm in positions.items():
        parts.append(f"{sid}:{pwm if pwm is not None else 'N/A':>5}")
    print("  ".join(parts))


def lock_all(positions: dict[str, int | None]) -> None:
    """
    向 000~005 号舵机发送位置指令以恢复阻力（重新进入 PID 保持模式）。
    使用最后读取到的位置，若无效则默认 1500（中位）。
    """
    print("正在恢复舵机阻力...")
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        pwm = positions.get(sid) or 1500
        cmd = f"#{sid}P{pwm}T{LOCK_TIME}!"
        uart_send_str(cmd)
        time.sleep(0.05)
        print(f"  舵机 {sid} 已锁定（PWM={pwm}，指令={cmd}）")
    print("所有舵机已恢复阻力。")


# ─── PWM ↔ 弧度 转换（映射关系见用户定义）────────────────────────────────────
# PWM 500  → 0°   → RViz2 -135° (-2.356 rad)
# PWM 1500 → 135° → RViz2   0°  ( 0     rad)
# PWM 2500 → 270° → RViz2  135° ( 2.356 rad)
def pwm_to_rad(pwm: int) -> float:
    """舵机 PWM(500~2500) → RViz2 弧度(-2.356~2.356)"""
    return (pwm - 1500) * 2.356 / 1000.0


def pwm_to_gripper(pwm: int) -> tuple[float, float]:
    """舵机 005 PWM → (joint6_rad, joint7_rad)"""
    if pwm >= 1600:
        return (-0.2, -0.2)   # 夹爪闭合
    elif pwm <= 1300:
        return (0.5, 0.5)     # 夹爪张开
    else:
        # 1300~1600 之间线性插值：0.5(张开) → -0.2(闭合)
        t = (pwm - 1300) / 300.0
        val = 0.5 - t * 0.7
        return (val, val)


def _build_joint_command(positions: dict, move_time: int = 200) -> str:
    """
    将舵机 PWM 位置字典转换为串口指令字符串。
    格式：{#000P<pwm>T<ms>!#001P<pwm>T<ms>!...}
    move_time：运动时间（ms），建议与采样周期接近以实现平滑跟随
    """
    parts = []
    for i in range(SERVO_COUNT):
        sid = _sid(i)
        pwm = positions.get(sid)
        if pwm is not None:
            parts.append(f"#{sid}P{pwm}T{move_time}!")
    return '{' + ''.join(parts) + '}'


def _publish_joint_states(node, publisher, positions: dict) -> None:
    """将舵机 PWM 位置转换为 JointState 消息并发布到 /joint_states"""
    from sensor_msgs.msg import JointState as JointStateMsg

    msg = JointStateMsg()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = list(JOINT_NAMES)

    # 舵机 000~004 → joint1~joint5
    for i in range(5):
        pwm = positions.get(_sid(i))
        msg.position.append(pwm_to_rad(pwm) if pwm is not None else 0.0)

    # 舵机 005 → joint6 + joint7（夹爪）
    gripper_pwm = positions.get('005')
    if gripper_pwm is not None:
        j6, j7 = pwm_to_gripper(gripper_pwm)
    else:
        j6, j7 = (0.5, 0.5)   # 默认张开
    msg.position.append(j6)
    msg.position.append(j7)

    publisher.publish(msg)
# ───────────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description='机械臂手动示教工具：释放/恢复舵机阻力，循环读取关节位置',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        '--lock', action='store_true',
        help='读取当前各关节位置后发送锁定指令，恢复舵机阻力并退出',
    )
    mode_group.add_argument(
        '--hard', action='store_true',
        help='以大阻力模式释放舵机（默认为小阻力 PULK）',
    )
    parser.add_argument(
        '--baud', type=int, default=115200,
        help='串口波特率（默认 115200）',
    )
    parser.add_argument(
        '--ros', action='store_true',
        help='启用 ROS2 模式，向 /joint_states 发布关节状态（用于 RViz2 同步显示）',
    )
    args = parser.parse_args()

    # ── ROS2 初始化（仅在 --ros 模式下） ──
    ros_node = None
    ros_pub = None
    ros_cmd_pub = None
    if args.ros:
        import rclpy
        from sensor_msgs.msg import JointState as JointStateMsg
        from std_msgs.msg import String as RosString
        rclpy.init()
        ros_node = rclpy.create_node('arm_teach_node')
        ros_pub = ros_node.create_publisher(JointStateMsg, '/joint_states', 10)
        # 发布串口指令格式的关节命令，供从机 servo_node 直接跟随
        ros_cmd_pub = ros_node.create_publisher(RosString, '/joint_commands', 10)
        # 后台线程处理 ROS2 通信
        _spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        _spin_thread.start()
        ros_node.get_logger().info('ROS2 发布已启动 → /joint_states + /joint_commands')

    print("正在初始化串口...")
    if not setup_uart(args.baud):
        print("错误：串口初始化失败，请检查连接后重试。")
        sys.exit(1)

    try:
        if args.lock:
            # ── 锁定模式：读取位置 → 发送锁定指令 → 退出
            print("\n[锁定模式] 读取当前各关节位置...")
            positions = read_once()
            print_positions(positions)
            print()
            lock_all(positions)

        else:
            # ── 示教模式：释放阻力 → 循环读取位置
            release_mode = 'M' if args.hard else 'K'
            release_all(release_mode)

            print("开始循环读取关节位置（按 Ctrl+C 退出）...\n")
            header = "  ".join(f"  {_sid(i)} " for i in range(SERVO_COUNT))
            print(header)
            print("-" * len(header))

            while True:
                positions = read_once()
                print_positions(positions)
                # ROS2 模式：发布到 /joint_states（RViz2）和 /joint_commands（从机跟随）
                if ros_node:
                    if ros_pub:
                        _publish_joint_states(ros_node, ros_pub, positions)
                    if ros_cmd_pub:
                        from std_msgs.msg import String as RosString
                        cmd_msg = RosString()
                        cmd_msg.data = _build_joint_command(positions)
                        ros_cmd_pub.publish(cmd_msg)
                time.sleep(0.05)   # 避免过于密集刷新

    except KeyboardInterrupt:
        print("\n\n收到退出信号...")
    finally:
        close_uart()
        if ros_node:
            ros_node.destroy_node()
            import rclpy
            rclpy.shutdown()
        print("串口已关闭，退出。")  # noqa: T201


if __name__ == '__main__':
    main()
