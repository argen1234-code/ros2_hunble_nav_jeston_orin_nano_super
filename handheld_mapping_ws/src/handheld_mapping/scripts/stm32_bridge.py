#!/usr/bin/python3
"""Bidirectional, mode-aware STM32 USB bridge.

Jetson sends the existing 12-byte cmd_vel frame. STM32 sends either the
versioned 151-byte sensor frame (preferred) or the legacy 15-byte GPS frame.
"""

import json
import math
import struct
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Int8, String
import serial


LEGACY_HEADER = b'\xaa\x55'
LEGACY_FRAME_LEN = 15
SENSOR_HEADER = b'\xcc\x55'
SENSOR_VERSION = 1
SENSOR_FRAME_LEN = 151

TX_HEADER = LEGACY_HEADER
TX_FMT = '<Bff'

MODE_GPS_ROS = 1
MODE_REMOTE = 2
MODE_INDOOR = 3
MODE_GPS_ONLY = 4

GPS_ROUTE_BEGIN = 1
GPS_ROUTE_POINT = 2
GPS_ROUTE_COMMIT = 3
GPS_ROUTE_CLEAR = 4

FLAG_GPS_VALID = 0x01
FLAG_MAG_VALID = 0x02
FLAG_IMU_VALID = 0x04
FLAG_GNSS_HEADING_VALID = 0x08
FLAG_ROUTE_VALID = 0x10
FLAG_TARGET_VALID = 0x20

# Starts at byte 14. Four GNSS floats, three magnetometer floats, nine
# JY901S diagnostic floats, four wheel floats, then target/route doubles.
SENSOR_VALUES_FMT = '<ddd' + ('f' * 20) + 'dddd'
SENSOR_VALUES_SIZE = struct.calcsize(SENSOR_VALUES_FMT)


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_hz', 50.0)
        self.declare_parameter('read_hz', 100.0)
        self.declare_parameter('nav_linear_sign', -1.0)
        self.declare_parameter('nav_angular_sign', -1.0)
        self.declare_parameter('nav_cmd_timeout', 0.5)
        self.declare_parameter('remote_cmd_timeout', 0.5)

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        self._nav_linear_sign = float(self.get_parameter('nav_linear_sign').value)
        self._nav_angular_sign = float(self.get_parameter('nav_angular_sign').value)
        self._nav_timeout = float(self.get_parameter('nav_cmd_timeout').value)
        self._remote_timeout = float(self.get_parameter('remote_cmd_timeout').value)
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self._rx_buf = bytearray()
        self._mode = MODE_REMOTE
        self._nav_cmd = Twist()
        self._remote_cmd = Twist()
        self._nav_time = 0.0
        self._remote_time = 0.0
        self._gps_ros_enabled = False
        self._gps_route_packets = deque()

        self.create_subscription(Int8, '/robot_mode', self._on_mode, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_nav_cmd, 10)
        self.create_subscription(Twist, '/remote_cmd_vel', self._on_remote_cmd, 10)
        self.create_subscription(
            Bool, '/gps_ros/control_enabled', self._on_gps_ros_enabled, 10)
        self.create_subscription(
            String, '/gps_only/route_command', self._on_gps_route_command, 10)

        self._legacy_heading_pub = self.create_publisher(Float32, '/gps_heading', 10)
        self._gps_pub = self.create_publisher(NavSatFix, '/gps_fix', 10)
        self._target_pub = self.create_publisher(NavSatFix, '/gps/target_fix', 10)
        self._sensor_pub = self.create_publisher(String, '/stm32/sensors', 10)
        self._wheel_pub = self.create_publisher(String, '/wheel_speeds', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        send_hz = float(self.get_parameter('send_hz').value)
        read_hz = float(self.get_parameter('read_hz').value)
        self.create_timer(1.0 / send_hz, self._send)
        self.create_timer(1.0 / read_hz, self._read)
        self.get_logger().info(
            f'STM32 bridge ready: {port}@{baudrate}, tx={send_hz}Hz rx={read_hz}Hz')

    def _on_mode(self, msg):
        if msg.data not in (MODE_GPS_ROS, MODE_REMOTE, MODE_INDOOR, MODE_GPS_ONLY):
            self.get_logger().warning(f'Ignoring unsupported robot mode {msg.data}')
            return
        if msg.data != self._mode:
            # A mode change must send zero until that mode produces a fresh command.
            self._nav_time = 0.0
            self._remote_time = 0.0
        self._mode = msg.data

    def _on_nav_cmd(self, msg):
        self._nav_cmd = msg
        self._nav_time = time.monotonic()

    def _on_remote_cmd(self, msg):
        self._remote_cmd = msg
        self._remote_time = time.monotonic()

    def _on_gps_ros_enabled(self, msg):
        self._gps_ros_enabled = bool(msg.data)

    def _on_gps_route_command(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        command = str(data.get('command', '')).upper()
        if command == 'GPS_ONLY_ROUTE_CLEAR':
            self._gps_route_packets.clear()
            self._gps_route_packets.append(
                self._build_route_packet(GPS_ROUTE_CLEAR, 0, 0, 1, 0.0, 0.0))
            return
        if command != 'GPS_ONLY_ROUTE_SET':
            return

        raw_waypoints = data.get('waypoints', [])
        if not isinstance(raw_waypoints, list):
            return
        waypoints = []
        for point in raw_waypoints[:10]:
            try:
                latitude = float(point['latitude'])
                longitude = float(point['longitude'])
            except (KeyError, TypeError, ValueError):
                continue
            if (math.isfinite(latitude) and math.isfinite(longitude) and
                    -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
                waypoints.append((latitude, longitude))
        if not waypoints:
            return

        total = len(waypoints)
        loop_enable = 1 if data.get('loop_enable', True) else 0
        packets = [self._build_route_packet(
            GPS_ROUTE_BEGIN, 0, total, loop_enable, 0.0, 0.0)]
        packets.extend(
            self._build_route_packet(
                GPS_ROUTE_POINT, index, total, loop_enable, latitude, longitude)
            for index, (latitude, longitude) in enumerate(waypoints))
        packets.append(self._build_route_packet(
            GPS_ROUTE_COMMIT, 0, total, loop_enable, 0.0, 0.0))
        self._gps_route_packets.clear()
        self._gps_route_packets.extend(packets)
        self.get_logger().info(f'Queued {total} GPS-only waypoints for STM32')

    def _build_route_packet(self, command, index, total, loop_enable, latitude, longitude):
        data = struct.pack(
            '<BBBBdd', command, index, total, loop_enable, latitude, longitude)
        return b'\xdd\x55' + data + bytes([self._xor(data)])

    def _send(self):
        now = time.monotonic()
        vx = 0.0
        wz = 0.0
        if self._mode == MODE_REMOTE and now - self._remote_time <= self._remote_timeout:
            # Preserve the already verified WeChat/chassis sign convention.
            vx = self._remote_cmd.linear.x
            wz = self._remote_cmd.angular.z
        elif (self._mode == MODE_INDOOR or
              (self._mode == MODE_GPS_ROS and self._gps_ros_enabled)) and \
                now - self._nav_time <= self._nav_timeout:
            # Nav2 REP-103 -> chassis sign convention.
            vx = self._nav_cmd.linear.x * self._nav_linear_sign
            wz = self._nav_cmd.angular.z * self._nav_angular_sign
        # GPS_ONLY deliberately receives zero velocity; STM32 owns its controller.
        data = struct.pack(TX_FMT, self._mode, vx, wz)
        try:
            self.ser.write(TX_HEADER + data + bytes([self._xor(data)]))
            if self._gps_route_packets:
                self.ser.write(self._gps_route_packets.popleft())
        except serial.SerialException as exc:
            self.get_logger().error(f'STM32 TX error: {exc}')

    def _read(self):
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self._rx_buf.extend(self.ser.read(waiting))
                self._parse_frames()
        except serial.SerialException as exc:
            self.get_logger().error(f'STM32 RX error: {exc}')

    def _parse_frames(self):
        while len(self._rx_buf) >= LEGACY_FRAME_LEN:
            sensor_idx = self._rx_buf.find(SENSOR_HEADER)
            legacy_idx = self._rx_buf.find(LEGACY_HEADER)
            candidates = [i for i in (sensor_idx, legacy_idx) if i >= 0]
            if not candidates:
                del self._rx_buf[:-1]
                return
            idx = min(candidates)
            if idx:
                del self._rx_buf[:idx]
            if self._rx_buf.startswith(SENSOR_HEADER):
                if len(self._rx_buf) < SENSOR_FRAME_LEN:
                    return
                frame = bytes(self._rx_buf[:SENSOR_FRAME_LEN])
                if frame[2] != SENSOR_VERSION or self._xor(frame[2:150]) != frame[150]:
                    del self._rx_buf[:2]
                    continue
                del self._rx_buf[:SENSOR_FRAME_LEN]
                self._publish_sensor_frame(frame)
            else:
                if len(self._rx_buf) < LEGACY_FRAME_LEN:
                    return
                frame = bytes(self._rx_buf[:LEGACY_FRAME_LEN])
                if self._xor(frame[2:14]) != frame[14]:
                    del self._rx_buf[:2]
                    continue
                del self._rx_buf[:LEGACY_FRAME_LEN]
                heading, lat, lon = struct.unpack('<fff', frame[2:14])
                self._legacy_heading_pub.publish(Float32(data=heading))
                self._publish_fix(self._gps_pub, lat, lon, 0.0, True, 'gps')

    def _publish_sensor_frame(self, frame):
        flags = frame[3]
        car_mode, satellites, fix_quality = frame[4], frame[5], frame[6]
        route_total, route_slot = frame[7], frame[8]
        nav_active, heading_status, loop_enable = frame[9], frame[10], frame[11]
        sequence = struct.unpack_from('<H', frame, 12)[0]
        values = struct.unpack_from(SENSOR_VALUES_FMT, frame, 14)
        (lat, lon, altitude, gnss_heading, gnss_speed, velocity_north,
         velocity_east, mag_yaw, mag_pitch, mag_roll, imu_roll, imu_pitch,
         imu_yaw, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
         wheel_0, wheel_1, wheel_2, wheel_3, target_lat, target_lon,
         route_lat, route_lon) = values

        gps_valid = bool(flags & FLAG_GPS_VALID)
        sensor = {
            'version': SENSOR_VERSION, 'sequence': sequence, 'flags': flags,
            'stm32_mode': car_mode, 'gps_valid': gps_valid,
            'satellites': satellites, 'fix_quality': fix_quality,
            'latitude': lat, 'longitude': lon, 'altitude': altitude,
            'gnss_heading_valid': bool(flags & FLAG_GNSS_HEADING_VALID),
            'gnss_heading': gnss_heading, 'gnss_speed': gnss_speed,
            'velocity_north': velocity_north, 'velocity_east': velocity_east,
            'mag_valid': bool(flags & FLAG_MAG_VALID), 'mag_yaw': mag_yaw,
            'mag_pitch': mag_pitch, 'mag_roll': mag_roll,
            # JY901S yaw is diagnostic only and is never an absolute heading source.
            'imu_valid': bool(flags & FLAG_IMU_VALID), 'imu_roll': imu_roll,
            'imu_pitch': imu_pitch, 'imu_yaw_diagnostic': imu_yaw,
            'gyro': [gyro_x, gyro_y, gyro_z], 'acc': [acc_x, acc_y, acc_z],
            'wheel_speeds': [wheel_0, wheel_1, wheel_2, wheel_3],
            'target_valid': bool(flags & FLAG_TARGET_VALID),
            'target_latitude': target_lat, 'target_longitude': target_lon,
            'route_valid': bool(flags & FLAG_ROUTE_VALID),
            'route_total': route_total, 'route_slot': route_slot,
            'route_latitude': route_lat, 'route_longitude': route_lon,
            'navigation_active': bool(nav_active),
            'heading_status': heading_status, 'loop_enable': bool(loop_enable),
            'received_monotonic': time.monotonic(),
        }
        self._sensor_pub.publish(String(data=json.dumps(sensor, separators=(',', ':'))))
        self._wheel_pub.publish(String(data=json.dumps({
            'sequence': sequence, 'rpm': sensor['wheel_speeds']
        }, separators=(',', ':'))))
        if gps_valid:
            self._publish_fix(self._gps_pub, lat, lon, altitude, True, 'gps')
        if sensor['target_valid']:
            self._publish_fix(self._target_pub, target_lat, target_lon, 0.0, True, 'gps')
        if sensor['imu_valid']:
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = 'imu_link'
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.x = math.radians(gyro_x)
            imu.angular_velocity.y = math.radians(gyro_y)
            imu.angular_velocity.z = math.radians(gyro_z)
            imu.linear_acceleration.x = acc_x * 9.80665
            imu.linear_acceleration.y = acc_y * 9.80665
            imu.linear_acceleration.z = acc_z * 9.80665
            self._imu_pub.publish(imu)

    def _publish_fix(self, publisher, lat, lon, altitude, valid, frame_id):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.status.status = (NavSatStatus.STATUS_FIX if valid
                             else NavSatStatus.STATUS_NO_FIX)
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(altitude)
        publisher.publish(msg)

    @staticmethod
    def _xor(data):
        value = 0
        for byte in data:
            value ^= byte
        return value

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
