#!/usr/bin/env python3
"""
YDLidar ROS2 LaserScan driver — pure Python, no YDLidar SDK dependency.

Uses the raw start-scan protocol (A5 60) only — no handshake, no health check,
no device info query. Works with wired USB and wireless (ESP8266) serial bridges
because it never waits for a command-response round-trip.

Protocol (Triangle LiDAR X4/TminiPro):
  Frame: AA 55 [CT] [LSN] [FSA_lo FSA_hi] [LSA_lo LSA_hi] [CS_lo CS_hi] [samples...]
  Sample: [dist_lo dist_hi]  (uint16, millimetres)
  Angles: q6 format (1/64 degree), LSB is checkbit (always 1 for valid data)

Parameters (all ROS2-declarable, overridable from launch):
  port          - serial device path
  baudrate      - serial baud rate (128000 wired, 230400 wireless/ESP8266)
  mode          - "wired" or "wireless" (only affects default timeouts)
  frame_id      - LaserScan header frame_id
  scan_topic    - published topic name
  angle_min/max - scan field of view (degrees)
  range_min/max - valid range (metres)
  frequency     - scan publish rate (Hz)
  sample_rate   - LiDAR sample rate code (9 = 9 kHz for X4)
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial


Q6_FULL_CIRCLE = 23040   # 360 * 64
Q6_TO_RAD = math.pi / 180.0 / 64.0  # q6 units → radians

SYNC = b'\xAA\x55'
FRAME_HEADER_LEN = 10     # sync(2) + ct(1) + lsn(1) + fsa(2) + lsa(2) + cs(2)
MIN_FRAME_LEN = FRAME_HEADER_LEN + 2  # header + at least 1 sample


class YDLidarDriver(Node):
    def __init__(self):
        super().__init__('ydlidar_driver')

        # ── ROS parameters ──────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 128000)
        self.declare_parameter('mode', 'wired')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('angle_min', -180.0)
        self.declare_parameter('angle_max', 180.0)
        self.declare_parameter('range_min', 0.01)
        self.declare_parameter('range_max', 64.0)
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('sample_rate', 9)
        self.declare_parameter('reversion', True)
        self.declare_parameter('inverted', True)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        mode = self.get_parameter('mode').value
        self.frame_id = self.get_parameter('frame_id').value
        scan_topic = self.get_parameter('scan_topic').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        frequency = self.get_parameter('frequency').value
        sample_rate = self.get_parameter('sample_rate').value
        self._reversion = self.get_parameter('reversion').value
        self._inverted = self.get_parameter('inverted').value

        if self._reversion or self._inverted:
            self.get_logger().info(
                f'Angle correction: reversion={self._reversion} inverted={self._inverted}')

        # Derived scan geometry
        points_per_scan = int(sample_rate * 1000 / frequency)
        self.angle_increment = 2.0 * math.pi / points_per_scan
        self.num_points = points_per_scan
        angle_min_rad = math.radians(self.get_parameter('angle_min').value)
        angle_max_rad = math.radians(self.get_parameter('angle_max').value)

        self.get_logger().info(
            f'port={port} baud={baudrate} mode={mode}')
        self.get_logger().info(
            f'{points_per_scan} pts/scan, angle_inc={math.degrees(self.angle_increment):.4f} deg')

        # ── Publisher ───────────────────────────────────────────────
        self.pub = self.create_publisher(LaserScan, scan_topic, 10)

        # ── Persistent scan array ───────────────────────────────────
        self.declare_parameter('stale_cycles', 2)
        stale_cycles = self.get_parameter('stale_cycles').value

        self._ranges = [float('inf')] * self.num_points
        self._ranges_age = [0] * self.num_points     # cycles since last update
        self._publish_cycle = 0
        self._max_stale = stale_cycles               # expire after N cycles
        self._lock = threading.Lock()
        self._running = True
        self._ser_ready = threading.Event()

        # ── Diagnostics ────────────────────────────────────────────
        self._diag_frames = 0
        self._diag_samples = 0
        self._diag_bytes = 0
        self._diag_buf_max = 0
        self._diag_rejected = 0

        # ── Timers ─────────────────────────────────────────────────
        self._diag_timer = self.create_timer(30.0, self._print_diag)
        self._timer = self.create_timer(1.0 / frequency, self._publish)

        # ── Serial + reader in background (avoids blocking ROS init) ─
        self.ser = None
        self._serial_thread = threading.Thread(
            target=self._serial_bootstrap, args=(port, baudrate), daemon=True)
        self._serial_thread.start()

    def _serial_bootstrap(self, port, baudrate):
        """Open serial port and start the read loop (runs in background)."""
        try:
            self.get_logger().info(f'Opening {port} @ {baudrate}...')
            self.ser = serial.Serial(port, baudrate, timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            return

        self._start_scan()
        self._ser_ready.set()
        self._read_loop()

    # ── LiDAR control ───────────────────────────────────────────────

    def _start_scan(self):
        """Enter scan mode — stop first, then start (wireless-safe)."""
        for _ in range(2):
            self.ser.write(b'\xA5\x65')
            time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.write(b'\xA5\x60')
        self.get_logger().info('Scan started')

    def _stop_scan(self):
        if self.ser is None:
            return
        try:
            self.ser.write(b'\xA5\x65')
        except Exception:
            pass

    # ── Serial read loop ────────────────────────────────────────────

    def _read_loop(self):
        """Non-blocking read — accumulate raw bytes in a buffer."""
        buf = b''
        while self._running:
            try:
                n = self.ser.in_waiting
                if n:
                    data = self.ser.read(n)
                    if data:
                        self._diag_bytes += len(data)
                        buf += data
                        buf = self._extract_frames(buf)
                        if len(buf) > self._diag_buf_max:
                            self._diag_buf_max = len(buf)
                else:
                    time.sleep(0.001)
            except serial.SerialException:
                time.sleep(0.01)

    def _print_diag(self):
        self.get_logger().info(
            f'diag: bytes={self._diag_bytes} frames={self._diag_frames} '
            f'samples={self._diag_samples} rejected={self._diag_rejected} '
            f'buf_max={self._diag_buf_max}')

    # ── Frame parser ────────────────────────────────────────────────

    def _extract_frames(self, buf):
        """Pull complete scan frames from *buf*, accumulate samples.

        Returns the unconsumed tail of *buf*.
        """
        while True:
            idx = buf.find(SYNC)
            if idx < 0:
                return buf
            buf = buf[idx:]                     # discard garbage before sync

            if len(buf) < MIN_FRAME_LEN:
                break

            # Unpack fixed header fields
            ct = buf[2]
            lsn = buf[3]
            fsa_raw = struct.unpack_from('<H', buf, 4)[0]
            lsa_raw = struct.unpack_from('<H', buf, 6)[0]

            # Basic sanity
            if lsn < 1 or lsn > 1024:
                self._diag_rejected += 1
                buf = buf[1:]
                continue
            if ct != 0x00:                      # normal scan data only
                self._diag_rejected += 1
                buf = buf[1:]
                continue
            if not (fsa_raw & 0x01) or not (lsa_raw & 0x01):
                self._diag_rejected += 1
                buf = buf[1:]
                continue

            frame_len = FRAME_HEADER_LEN + lsn * 2
            if len(buf) < frame_len:
                break

            # ── Valid frame — decode ────────────────────────────────
            self._diag_frames += 1
            fsa_q6 = fsa_raw >> 1
            lsa_q6 = lsa_raw >> 1

            # Handle zero-crossing
            if lsa_q6 < fsa_q6:
                lsa_q6 += Q6_FULL_CIRCLE

            # Interpolate each sample
            delta_q6 = lsa_q6 - fsa_q6
            sample_bytes = buf[FRAME_HEADER_LEN:frame_len]

            with self._lock:
                for i in range(lsn):
                    dist_mm = struct.unpack_from('<H', sample_bytes, i * 2)[0]
                    if dist_mm == 0:
                        continue

                    if lsn == 1:
                        angle_q6 = fsa_q6
                    else:
                        angle_q6 = fsa_q6 + delta_q6 * i // (lsn - 1)

                    angle_q6 %= Q6_FULL_CIRCLE
                    angle_rad = angle_q6 * Q6_TO_RAD

                    # Apply LiDAR model corrections (X4: reversion + inverted)
                    if self._reversion:
                        angle_rad = angle_rad + math.pi
                    if self._inverted:
                        angle_rad = 2.0 * math.pi - angle_rad
                    angle_rad %= 2.0 * math.pi

                    idx = int(angle_rad / self.angle_increment + 0.5) % self.num_points

                    dist_m = dist_mm * 0.001
                    if self.range_min <= dist_m <= self.range_max:
                        self._ranges[idx] = dist_m
                        self._ranges_age[idx] = 0
                        self._diag_samples += 1

            buf = buf[frame_len:]

        return buf

    # ── Publisher ───────────────────────────────────────────────────

    def _publish(self):
        """Publish LaserScan with persistent accumulation.

        Instead of resetting the array each cycle (which loses data
        over slow wireless links), bins are only set to inf when they
        haven't been updated for _max_stale cycles.
        """
        with self._lock:
            self._publish_cycle += 1

            # Build output: expire bins that haven't been updated recently
            ranges = []
            for i in range(self.num_points):
                if self._ranges_age[i] >= self._max_stale:
                    ranges.append(float('inf'))
                    self._ranges[i] = float('inf')
                else:
                    ranges.append(self._ranges[i])
                    self._ranges_age[i] += 1

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = []

        self.pub.publish(msg)

    # ── Shutdown ────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._stop_scan()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YDLidarDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()