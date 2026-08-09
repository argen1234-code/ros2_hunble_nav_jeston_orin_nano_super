#!/usr/bin/python3
"""Convert ROS OccupancyGrid and Nav2 Path data into MQTT-friendly JSON."""

import base64
import json
import math
import struct
import zlib

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String


def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    body = chunk_type + data
    return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)


def encode_grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    rows = bytearray()
    for row in range(height):
        rows.append(0)  # PNG filter type 0
        start = row * width
        rows.extend(pixels[start:start + width])
    signature = b'\x89PNG\r\n\x1a\n'
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return signature + _png_chunk(b'IHDR', ihdr) + _png_chunk(b'IDAT', zlib.compress(bytes(rows), 6)) + _png_chunk(b'IEND', b'')


class IndoorMapBridge(Node):
    def __init__(self):
        super().__init__('indoor_map_bridge')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('publish_interval', 2.0)
        self.declare_parameter('max_dimension', 640)

        self._max_dimension = int(self.get_parameter('max_dimension').value)
        self._latest_map = None
        self._last_signature = None
        self._revision = 0

        self._map_pub = self.create_publisher(String, '/indoor/map', 1)
        self._path_pub = self.create_publisher(String, '/indoor/path', 5)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self._on_map, map_qos)
        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self._on_path, 5)
        self.create_timer(float(self.get_parameter('publish_interval').value), self._publish_map)
        self.get_logger().info('Indoor map bridge ready')

    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _on_path(self, msg: Path):
        poses = msg.poses
        if not poses:
            payload = {'type': 'indoor_path', 'frame_id': msg.header.frame_id, 'points': []}
        else:
            step = max(1, math.ceil(len(poses) / 120))
            points = [
                {'x': round(p.pose.position.x, 4), 'y': round(p.pose.position.y, 4)}
                for p in poses[::step]
            ]
            if points[-1] != {
                'x': round(poses[-1].pose.position.x, 4),
                'y': round(poses[-1].pose.position.y, 4),
            }:
                points.append({
                    'x': round(poses[-1].pose.position.x, 4),
                    'y': round(poses[-1].pose.position.y, 4),
                })
            payload = {'type': 'indoor_path', 'frame_id': msg.header.frame_id, 'points': points}
        self._path_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def _publish_map(self):
        msg = self._latest_map
        if msg is None or msg.info.width == 0 or msg.info.height == 0:
            return

        raw_signature = zlib.crc32(bytes((value + 1) & 0xFF for value in msg.data)) & 0xFFFFFFFF
        signature = (raw_signature, msg.info.width, msg.info.height, msg.info.resolution)
        if signature == self._last_signature:
            return
        self._last_signature = signature

        width = int(msg.info.width)
        height = int(msg.info.height)
        stride = max(1, math.ceil(max(width, height) / self._max_dimension))
        out_width = math.ceil(width / stride)
        out_height = math.ceil(height / stride)
        pixels = bytearray(out_width * out_height)

        out_index = 0
        # OccupancyGrid row 0 is the map's bottom row. PNG row 0 is the top.
        for out_y in range(out_height):
            source_y = min(height - 1, height - 1 - out_y * stride)
            row_start = source_y * width
            for out_x in range(out_width):
                value = msg.data[row_start + min(width - 1, out_x * stride)]
                if value < 0:
                    gray = 205
                elif value >= 65:
                    gray = 0
                else:
                    gray = max(1, 254 - int(value * 2.53))
                pixels[out_index] = gray
                out_index += 1

        png = encode_grayscale_png(out_width, out_height, bytes(pixels))
        q = msg.info.origin.orientation
        origin_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._revision += 1
        payload = {
            'type': 'indoor_map',
            'revision': self._revision,
            'frame_id': msg.header.frame_id or 'map',
            'width': out_width,
            'height': out_height,
            'resolution': float(msg.info.resolution) * stride,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'origin_yaw': origin_yaw,
            'png_base64': base64.b64encode(png).decode('ascii'),
        }
        self._map_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        self.get_logger().info(
            f'Map revision {self._revision}: {out_width}x{out_height}, PNG={len(png)} bytes')


def main():
    rclpy.init()
    node = IndoorMapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
