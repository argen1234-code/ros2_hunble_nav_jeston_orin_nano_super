#!/usr/bin/python3
"""
Monitors /cmd_vel topic and logs velocity commands — use for verifying
the navigation output before sending to a real robot motor controller.

Displays:
  - linear.x  (m/s)   forward velocity
  - angular.z (rad/s) rotation velocity
  - timestamp
  - running stats (min, max, count)

Logs all commands to a CSV file for offline analysis.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import os
import csv
import time


class CmdVelMonitor(Node):

    def __init__(self):
        super().__init__('cmd_vel_monitor')

        log_dir = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), '..', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'cmd_vel_{timestamp}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ['stamp_sec', 'stamp_nsec', 'linear_x', 'linear_y',
             'linear_z', 'angular_x', 'angular_y', 'angular_z'])

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._callback, 10)

        # Statistics
        self.count = 0
        self.vx_sum = 0.0
        self.vx_max = 0.0
        self.vz_sum = 0.0
        self.vz_max = 0.0
        self.last_msg = None

        self._print_header()
        self.create_timer(5.0, self._print_stats)

        self.get_logger().info(
            f'cmd_vel_monitor active — logging to {self.csv_path}')

    def _print_header(self):
        print()
        print(f'{"Time":>12s}  {"linear.x":>9s}  {"angular.z":>9s}  {"note"}')
        print('-' * 52)

    def _callback(self, msg: Twist):
        self.last_msg = msg
        self.count += 1
        vx = msg.linear.x
        vz = msg.angular.z

        self.vx_sum += abs(vx)
        self.vx_max = max(self.vx_max, abs(vx))
        self.vz_sum += abs(vz)
        self.vz_max = max(self.vz_max, abs(vz))

        now = self.get_clock().now()
        t_str = f'{now.seconds_nanoseconds()[0]}.{now.seconds_nanoseconds()[1] // 1000000:03d}'

        # Highlight non-zero commands
        note = ''
        if abs(vx) > 0.001 or abs(vz) > 0.001:
            note = '>>> ACTIVE'

        print(f'{t_str:>12s}  {vx:>+9.4f}  {vz:>+9.4f}  {note}')

        self.csv_writer.writerow([
            now.seconds_nanoseconds()[0],
            now.seconds_nanoseconds()[1],
            vx, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, vz,
        ])

    def _print_stats(self):
        if self.count == 0:
            return
        avg_vx = self.vx_sum / self.count
        avg_vz = self.vz_sum / self.count
        print()
        print(f'  [stats] msgs={self.count}  '
              f'avg_|vx|={avg_vx:.3f}  max_|vx|={self.vx_max:.3f}  '
              f'avg_|vz|={avg_vz:.3f}  max_|vz|={self.vz_max:.3f}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
