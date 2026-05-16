#!/usr/bin/python3
"""
Bidirectional STM32 serial bridge with mode-aware cmd_vel routing.

TX (Jetson → STM32)  12-byte packet @ send_hz:
  ┌──────┬──────┬────────┬──────────────────┬──────────────────┬──────────┐
  │  0   │  1   │   2    │     3 ~ 6        │     7 ~ 10       │    11    │
  │ 0xAA │ 0x55 │ mode   │ vx  float32 LE   │ vz  float32 LE   │ XOR      │
  │ 帧头1│帧头2 │ 运行模式│ 线速度 m/s       │ 角速度 rad/s     │ 校验     │
  └──────┴──────┴────────┴──────────────────┴──────────────────┴──────────┘
  XOR = buf[2] ^ buf[3] ^ ... ^ buf[10]  (9 bytes)

  mode: 1=GPS(建图导航)  2=REMOTE(遥控)  3=LINE(巡线)

RX (STM32 → Jetson)  15-byte frame:
  ┌──────┬──────┬────────────────────────┬──────────────────┬──────────────────┬──────────┐
  │  0   │  1   │       2 ~ 5            │     6 ~ 9        │    10 ~ 13       │   14     │
  │ 0xAA │ 0x55 │ heading_to_target_deg  │   current_lat    │   current_lon    │ XOR      │
  │ 帧头1│帧头2 │ float32 LE, 车头→GPS目标│ float32 LE, 纬度  │ float32 LE, 经度  │ 校验     │
  └──────┴──────┴────────────────────────┴──────────────────┴──────────────────┴──────────┘
  XOR = buf[2] ^ buf[3] ^ ... ^ buf[13]  (12 bytes)

Cmd_vel routing:
  - mode 1 (GPS) / mode 3 (LINE): forward /cmd_vel from Nav2
  - mode 2 (REMOTE):              forward /remote_cmd_vel from MQTT

Usage:
  ros2 run handheld_mapping stm32_bridge --ros-args -p port:=/dev/ttyACM0 -p baudrate:=115200
"""

import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int8
import serial


# ── Protocol constants ─────────────────────────────────────────────
RX_HEADER     = b'\xaa\x55'
RX_FRAME_LEN  = 15     # header(2) + heading(4) + lat(4) + lon(4) + checksum(1)
RX_PAYLOAD_LEN = 12    # bytes 2..13 xor'd into byte 14

TX_HEADER     = struct.pack('<BB', 0xAA, 0x55)
TX_FMT        = '<Bff'  # mode(uint8) + vx(float32) + vz(float32)
TX_DATA_LEN   = struct.calcsize(TX_FMT)  # 9
TX_FRAME_LEN  = 2 + TX_DATA_LEN + 1      # 12

MODE_GPS    = 1
MODE_REMOTE = 2
MODE_LINE   = 3


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_hz', 50.0)
        self.declare_parameter('read_hz', 100.0)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        send_hz = self.get_parameter('send_hz').value
        read_hz = self.get_parameter('read_hz').value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0)
            self.get_logger().info(f'Serial opened: {port} @ {baudrate}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {port}: {e}')
            raise

        self._rx_buf = bytearray()

        # ── Cached latest values ───────────────────────────────────
        self._mode = MODE_GPS
        self._nav_cmd_vel = Twist()       # from /cmd_vel (Nav2)
        self._remote_cmd_vel = Twist()    # from /remote_cmd_vel (MQTT)

        # ── Subscriptions ───────────────────────────────────────────
        self._mode_sub = self.create_subscription(
            Int8, '/robot_mode', self._on_mode, 10)
        self._nav_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_nav_cmd_vel, 10)
        self._remote_sub = self.create_subscription(
            Twist, '/remote_cmd_vel', self._on_remote_cmd_vel, 10)

        # ── Publishers ──────────────────────────────────────────────
        self._heading_pub = self.create_publisher(Float32, '/gps_heading', 10)
        self._gps_pub = self.create_publisher(NavSatFix, '/gps_fix', 10)

        # ── Timers ──────────────────────────────────────────────────
        self._send_timer = self.create_timer(1.0 / send_hz, self._send_to_stm32)
        self._read_timer = self.create_timer(1.0 / read_hz, self._read_from_stm32)

        self.get_logger().info(f'STM32 bridge ready: send={send_hz}Hz read={read_hz}Hz')

    # ── Subscriptions ──────────────────────────────────────────────

    def _on_mode(self, msg: Int8):
        self._mode = msg.data

    def _on_nav_cmd_vel(self, msg: Twist):
        self._nav_cmd_vel = msg

    def _on_remote_cmd_vel(self, msg: Twist):
        self._remote_cmd_vel = msg

    # ── TX: Jetson → STM32 ─────────────────────────────────────────

    def _send_to_stm32(self):
        """Read /cmd_vel or /remote_cmd_vel based on mode, pack and send."""
        if self._mode == MODE_REMOTE:
            vx = self._remote_cmd_vel.linear.x
            vz = self._remote_cmd_vel.angular.z
        else:
            # GPS or LINE: use Nav2 cmd_vel
            vx = self._nav_cmd_vel.linear.x
            vz = self._nav_cmd_vel.angular.z

        data = struct.pack(TX_FMT, self._mode, vx, vz)
        checksum = self._xor(data)
        packet = TX_HEADER + data + struct.pack('<B', checksum)

        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f'TX error: {e}')

    # ── RX: STM32 → Jetson ─────────────────────────────────────────

    def _read_from_stm32(self):
        try:
            while self.ser.in_waiting > 0:
                self._rx_buf.extend(self.ser.read(self.ser.in_waiting))
                self._parse_frames()
        except serial.SerialException as e:
            self.get_logger().error(f'RX error: {e}')

    def _parse_frames(self):
        while len(self._rx_buf) >= RX_FRAME_LEN:
            idx = self._rx_buf.find(RX_HEADER, 0, len(self._rx_buf) - RX_FRAME_LEN + 1)
            if idx < 0:
                self._rx_buf = self._rx_buf[-RX_FRAME_LEN + 1:]
                return
            if idx > 0:
                del self._rx_buf[:idx]

            frame = self._rx_buf[:RX_FRAME_LEN]
            payload = frame[2:14]   # heading(4) + lat(4) + lon(4) = 12 bytes
            expected = frame[14]
            actual = self._xor(payload)

            if expected != actual:
                self.get_logger().debug(f'RX checksum mismatch')
                del self._rx_buf[:2]
                continue

            heading = struct.unpack('<f', frame[2:6])[0]
            lat     = struct.unpack('<f', frame[6:10])[0]
            lon     = struct.unpack('<f', frame[10:14])[0]
            del self._rx_buf[:RX_FRAME_LEN]

            self.get_logger().info(
                f'RX: heading={heading:.1f}° lat={lat:.6f} lon={lon:.6f}')

            self._heading_pub.publish(Float32(data=heading))

            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'gps'
            fix.latitude = float(lat)
            fix.longitude = float(lon)
            fix.altitude = 0.0
            self._gps_pub.publish(fix)

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
    node = Stm32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
