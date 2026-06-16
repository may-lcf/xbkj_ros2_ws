#!/usr/bin/env python3
# joy_arm_node: press-to-move arm control via joystick buttons
#
# Button mapping (teleop_control_node.py usbType=0, PS2 USB adapter):
#   axes[4] dpad-X: +1=LR(right), -1=LL(left)
#   axes[5] dpad-Y: -1=LU(up),    +1=LD(down)
#   buttons[0]=RD  buttons[4]=RU
#   buttons[1]=RR  buttons[3]=RL
#   buttons[6]=L1  buttons[7]=R1
#   buttons[8]=L2  buttons[9]=R2
#   buttons[10]=SELECT (reset)
#
# Servo mapping:
#   LL/LR  -> #000  LD/LU  -> #001
#   RD/RU  -> #002  RR/RL  -> #003
#   L1/R1  -> #004  L2/R2  -> #005 (gripper)
#   Press = move to limit (T2000), Release = PDST

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

MOVE_TIME = 2000
INIT_CMD = '{#000P1500T1000!#001P1666T1000!#002P2219T1000!#003P0905T1000!#004P1500T1000!#005P1500T1000!}'


class JoyArmNode(Node):
    def __init__(self):
        super().__init__('joy_arm_node')

        self.declare_parameter('move_time', MOVE_TIME)
        self.move_time = self.get_parameter('move_time').value

        self.axes         = []
        self.buttons      = []
        self.last_axes    = []
        self.last_buttons = []

        self.cmd_pub = self.create_publisher(String, '/joint_commands', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('joy_arm_node started')
        self._send(INIT_CMD)

    def _send(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def _move(self, servo, pos):
        cmd = '#{:03d}P{:04d}T{:04d}!'.format(servo, pos, self.move_time)
        self._send(cmd)
        self.get_logger().info(cmd)

    def _stop(self, servo):
        cmd = '#{:03d}PDST!'.format(servo)
        self._send(cmd)
        self.get_logger().info(cmd)

    def joy_callback(self, msg):
        new_axes    = list(msg.axes)
        new_buttons = list(msg.buttons)

        if not self.axes:
            self.last_axes    = new_axes[:]
            self.last_buttons = [0] * len(new_buttons)
        else:
            self.last_axes    = self.axes[:]
            self.last_buttons = self.buttons[:]

        self.axes    = new_axes
        self.buttons = new_buttons

        ax = self.axes
        la = self.last_axes
        bt = self.buttons
        lb = self.last_buttons

        # SELECT reset
        if len(bt) > 8 and bt[8] == 1 and lb[8] == 0:
            self._send(INIT_CMD)
            self.get_logger().info('reset to init pos')
            return

        # dpad-X -> servo #000
        if len(ax) > 4 and len(la) > 4:
            if ax[4] != 0.0 and la[4] == 0.0:
                self._move(0, 600 if ax[4] < 0 else 2400)   # LL->600, LR->2400
            elif ax[4] == 0.0 and la[4] != 0.0:
                self._stop(0)

        # dpad-Y -> servo #001
        if len(ax) > 5 and len(la) > 5:
            if ax[5] != 0.0 and la[5] == 0.0:
                self._move(1, 600 if ax[5] < 0 else 2400)   # LU->600, LD->2400
            elif ax[5] == 0.0 and la[5] != 0.0:
                self._stop(1)

        # button map: (btn_idx, servo, pos_when_pressed)
        btn_map = [
            (0,  2,  600),   # RD -> #002 P0600
            (2,  2, 2400),   # RU -> #002 P2400
            (1,  3, 2400),   # RR -> #003 P2400
            (3,  3,  600),   # RL -> #003 P0600
            (4,  4, 2400),   # L1 -> #004 P2400
            (5,  4,  600),   # R1 -> #004 P0600
            (6,  5,  600),   # L2 -> #005 P0600 (open)
            (7,  5, 2400),   # R2 -> #005 P2400 (close)
        ]

        for bidx, servo, pos in btn_map:
            if len(bt) <= bidx or len(lb) <= bidx:
                continue
            if bt[bidx] == 1 and lb[bidx] == 0:
                self._move(servo, pos)
            elif bt[bidx] == 0 and lb[bidx] == 1:
                self._stop(servo)


def main(args=None):
    rclpy.init(args=args)
    node = JoyArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
