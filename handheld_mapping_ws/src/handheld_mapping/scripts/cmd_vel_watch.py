#!/usr/bin/python3
"""
Standalone cmd_vel watcher — run in a separate terminal to see velocity commands.

Usage:
    ros2 run handheld_mapping cmd_vel_watch
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelWatch(Node):
    def __init__(self):
        super().__init__('cmd_vel_watch')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.count = 0
        self.get_logger().info('Watching /cmd_vel ...')
        print(f'\n{"#":>6s}  {"linear.x":>9s}  {"angular.z":>9s}\n' + '-' * 33)

    def _cb(self, msg):
        self.count += 1
        vx = msg.linear.x
        vz = msg.angular.z
        flag = ' <<<' if abs(vx) > 0.001 or abs(vz) > 0.001 else ''
        print(f'{self.count:>6d}  {vx:>+9.4f}  {vz:>+9.4f}{flag}')


def main():
    rclpy.init()
    node = CmdVelWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
