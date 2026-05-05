#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

import os
import time


class MapSaver(Node):
    """Subscribes to /map and saves it to disk on service call."""

    def __init__(self):
        super().__init__('map_saver')

        self.latest_map = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

        self.save_srv = self.create_service(
            Trigger, '~/save_map', self._save_map_callback)

        # Save to the package's share/maps directory
        import ament_index_python
        try:
            share_dir = ament_index_python.get_package_share_directory(
                'handheld_mapping')
        except Exception:
            share_dir = os.path.join(
                os.path.dirname(os.path.realpath(__file__)), '..', '..', 'share',
                'handheld_mapping')

        self.save_dir = os.path.join(share_dir, 'maps')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(
            f'MapSaver ready — call "/map_saver/save_map" to save.\n'
            f'  Maps saved to: {self.save_dir}')

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _save_map_callback(self, request, response):
        if self.latest_map is None:
            response.success = False
            response.message = 'No map received yet.'
            return response

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        pgm_path = os.path.join(self.save_dir, f'map_{timestamp}.pgm')
        yaml_path = os.path.join(self.save_dir, f'map_{timestamp}.yaml')

        try:
            self._write_pgm(self.latest_map, pgm_path)
            self._write_yaml(self.latest_map, pgm_path, yaml_path)
            response.success = True
            response.message = f'Map saved to {yaml_path}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Failed to save map: {e}'
            self.get_logger().error(response.message)

        return response

    def _write_pgm(self, map_msg: OccupancyGrid, path: str):
        """Write occupancy grid as PGM (P5 format)."""
        width = map_msg.info.width
        height = map_msg.info.height
        data = map_msg.data

        with open(path, 'wb') as f:
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            for y in range(height):
                row_start = y * width
                for x in range(width):
                    val = data[row_start + x]
                    if val == -1:
                        f.write(b'\xcd')  # unknown: 205
                    elif val == 100:
                        f.write(b'\x00')  # occupied: 0
                    else:
                        f.write(b'\xff')  # free: 255

    def _write_yaml(self, map_msg: OccupancyGrid, pgm_path: str, yaml_path: str):
        """Write map YAML metadata with absolute image path."""
        pgm_abs = os.path.abspath(pgm_path)
        info = map_msg.info
        with open(yaml_path, 'w') as f:
            f.write(f'image: {pgm_abs}\n')
            f.write(f'mode: trinary\n')
            f.write(f'resolution: {info.resolution}\n')
            f.write(f'origin: [{info.origin.position.x}, '
                    f'{info.origin.position.y}, {info.origin.position.z}]\n')
            f.write(f'negate: 0\n')
            f.write(f'occupied_thresh: 0.65\n')
            f.write(f'free_thresh: 0.196\n')


def main():
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
