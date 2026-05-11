#!/usr/bin/python3
"""
Forward /cmd_vel to STM32 over serial.

Binary protocol (11 bytes per packet):
  [0xAA] [0x55] [vx: float32 LE] [vz: float32 LE] [checksum: XOR of bytes 2..9]

Usage:
  ros2 run handheld_mapping cmd_vel_sender --ros-args -p port:=/dev/ttyACM0 -p baudrate:=115200
"""

import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


HEADER = struct.pack('<BB', 0xAA, 0x55)
PACKET_FMT = '<ff'   # vx (m/s), vz (rad/s)
PACKET_DATA_LEN = struct.calcsize(PACKET_FMT)  # 8


class CmdVelSender(Node):
    def __init__(self):
        super().__init__('cmd_vel_sender')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('min_send_interval', 0.02)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self._min_interval = self.get_parameter('min_send_interval').value

        self._last_send = self.get_clock().now()

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0)
            self.get_logger().info(f'Serial opened: {port} @ {baudrate}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {port}: {e}')
            raise

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._callback, 10)

    def _callback(self, msg: Twist):
        now = self.get_clock().now()
        if (now - self._last_send).nanoseconds * 1e-9 < self._min_interval:
            return
        self._last_send = now

        vx = msg.linear.x
        vz = msg.angular.z

        data = struct.pack(PACKET_FMT, vx, vz)
        checksum = self._xor(data)

        packet = HEADER + data + struct.pack('<B', checksum)
        try:
            self.ser.write(packet)
            self.get_logger().debug(
                f'Sent: vx={vx:+.4f} vz={vz:+.4f} checksum=0x{checksum:02X}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    @staticmethod
    def _xor(data: bytes) -> int:
        result = 0
        for b in data:
            result ^= b
        return result

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()