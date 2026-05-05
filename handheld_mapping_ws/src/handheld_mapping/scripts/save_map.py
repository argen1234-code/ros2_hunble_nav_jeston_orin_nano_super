#!/usr/bin/python3
"""
Convenience script to trigger map saving.

Usage:
    ros2 run handheld_mapping save_map

or call the service directly:
    ros2 service call /map_saver/save_map std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('save_map_client')
    cli = node.create_client(Trigger, '/map_saver/save_map')

    while not cli.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Waiting for /map_saver/save_map service...')

    req = Trigger.Request()
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Result: {future.result().message}')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
