#!/usr/bin/python3
"""Jetson-side GPS+ROS fusion controller.

Global GPS route management and heading-to-map conversion live here. Nav2
still performs local planning and obstacle avoidance. Absolute heading uses
dual-antenna GNSS first and QMC5883 second; JY901S yaw is intentionally not
used because it is not a trustworthy absolute heading source on this robot.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String


MODE_GPS_ROS = 1


def normalize_degrees(value):
    return (value + 180.0) % 360.0 - 180.0


def distance_and_bearing(lat1, lon1, lat2, lon2):
    radius = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    distance = radius * 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return distance, bearing


class GpsRosController(Node):
    def __init__(self):
        super().__init__('virtual_goal_publisher')
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('update_interval', 0.5)
        self.declare_parameter('arrival_radius', 1.0)
        self.declare_parameter('waypoint_dwell', 5.0)
        self.declare_parameter('sensor_timeout', 0.75)
        self.declare_parameter('min_goal_translation', 0.30)
        self.declare_parameter('min_goal_yaw_deg', 8.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        self._lookahead = float(self.get_parameter('lookahead_distance').value)
        self._arrival_radius = float(self.get_parameter('arrival_radius').value)
        self._dwell_seconds = float(self.get_parameter('waypoint_dwell').value)
        self._sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self._min_translation = float(self.get_parameter('min_goal_translation').value)
        self._min_yaw = math.radians(float(self.get_parameter('min_goal_yaw_deg').value))
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value

        from tf2_ros import Buffer, TransformListener
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_errors = (LookupException, ConnectivityException, ExtrapolationException)

        self._mode = 2
        self._sensor = None
        self._sensor_time = 0.0
        self._route = {}
        self._route_total = 0
        self._waypoint_index = 0
        self._dwell_until = 0.0
        self._last_goal = None
        self._goal_handle = None
        self._request_pending = False
        self._control_enabled = False

        self.create_subscription(Int8, '/robot_mode', self._on_mode, 10)
        self.create_subscription(String, '/stm32/sensors', self._on_sensors, 10)
        self._state_pub = self.create_publisher(String, '/gps_ros/state', 10)
        self._enable_pub = self.create_publisher(Bool, '/gps_ros/control_enabled', 10)
        self._action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        interval = float(self.get_parameter('update_interval').value)
        self.create_timer(interval, self._tick)
        self.get_logger().info(
            'GPS+ROS controller ready: heading=GNSS then QMC5883; JY901S yaw disabled')

    def _on_mode(self, msg):
        if msg.data == self._mode:
            return
        self._mode = msg.data
        self._last_goal = None
        self._waypoint_index = 0
        self._dwell_until = 0.0
        if self._mode != MODE_GPS_ROS:
            self._stop('MODE_INACTIVE')

    def _on_sensors(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self._sensor = data
        self._sensor_time = time.monotonic()
        total = int(data.get('route_total', 0))
        if total != self._route_total:
            self._route = {}
            self._route_total = total
            self._waypoint_index = 0
            self._last_goal = None
        if data.get('route_valid') and 0 <= int(data.get('route_slot', -1)) < total:
            slot = int(data['route_slot'])
            self._route[slot] = (
                float(data.get('route_latitude', 0.0)),
                float(data.get('route_longitude', 0.0)))

    def _heading(self):
        if self._sensor.get('gnss_heading_valid'):
            return float(self._sensor['gnss_heading']) % 360.0, 'GNSS_DUAL'
        if self._sensor.get('mag_valid'):
            return float(self._sensor['mag_yaw']) % 360.0, 'QMC5883'
        return None, 'NONE'

    def _robot_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except self._tf_errors:
            return None
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return transform.transform.translation.x, transform.transform.translation.y, yaw

    def _tick(self):
        if self._mode != MODE_GPS_ROS:
            self._set_enabled(False)
            return
        now = time.monotonic()
        if self._sensor is None or now - self._sensor_time > self._sensor_timeout:
            self._stop('STM32_TIMEOUT')
            return
        if not self._sensor.get('gps_valid'):
            self._stop('GPS_INVALID')
            return
        heading, heading_source = self._heading()
        if heading is None:
            self._stop('HEADING_INVALID')
            return
        if self._route_total <= 0 or len(self._route) < self._route_total:
            self._stop('ROUTE_INCOMPLETE')
            return
        if not self._sensor.get('navigation_active'):
            self._stop('MISSION_INACTIVE')
            return
        pose = self._robot_pose()
        if pose is None:
            self._stop('TF_UNAVAILABLE')
            return
        if not self._action.server_is_ready():
            self._stop('NAV2_UNAVAILABLE')
            return

        if now < self._dwell_until:
            self._stop('WAYPOINT_DWELL', cancel=False)
            return

        current_lat = float(self._sensor['latitude'])
        current_lon = float(self._sensor['longitude'])
        target_lat, target_lon = self._route[self._waypoint_index]
        distance, bearing = distance_and_bearing(
            current_lat, current_lon, target_lat, target_lon)

        if distance <= self._arrival_radius:
            if self._waypoint_index + 1 < self._route_total:
                self._waypoint_index += 1
            elif self._sensor.get('loop_enable') and self._route_total > 1:
                self._waypoint_index = 0
            else:
                self._stop('MISSION_COMPLETE')
                return
            self._dwell_until = now + self._dwell_seconds
            self._last_goal = None
            self._stop('WAYPOINT_REACHED')
            return

        map_x, map_y, map_yaw = pose
        heading_error_cw = normalize_degrees(bearing - heading)
        goal_yaw = map_yaw - math.radians(heading_error_cw)
        carrot_distance = min(self._lookahead, distance)
        goal_x = map_x + carrot_distance * math.cos(goal_yaw)
        goal_y = map_y + carrot_distance * math.sin(goal_yaw)

        self._publish_state('NAVIGATING', heading_source, distance, bearing,
                            heading, heading_error_cw)
        self._set_enabled(True)
        if self._last_goal is not None:
            shift = math.hypot(goal_x - self._last_goal[0], goal_y - self._last_goal[1])
            yaw_shift = abs(math.atan2(math.sin(goal_yaw - self._last_goal[2]),
                                       math.cos(goal_yaw - self._last_goal[2])))
            if shift < self._min_translation and yaw_shift < self._min_yaw:
                return
        if not self._request_pending:
            self._last_goal = (goal_x, goal_y, goal_yaw)
            self._send_goal(goal_x, goal_y, goal_yaw)

    def _send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = Quaternion(
            z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        self._request_pending = True
        future = self._action.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self._request_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'GPS+ROS goal request failed: {exc}')
            self._set_enabled(False)
            return
        if handle.accepted:
            self._goal_handle = handle
        else:
            self._set_enabled(False)

    def _set_enabled(self, enabled):
        self._control_enabled = bool(enabled)
        self._enable_pub.publish(Bool(data=self._control_enabled))

    def _stop(self, state, cancel=True):
        self._set_enabled(False)
        if cancel and self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self._request_pending = False
        self._publish_state(state)

    def _publish_state(self, state, heading_source='NONE', distance=None,
                       bearing=None, heading=None, heading_error=None):
        payload = {
            'state': state, 'control_enabled': self._control_enabled,
            'waypoint_index': self._waypoint_index,
            'waypoint_total': self._route_total,
            'heading_source': heading_source,
            'distance_to_target': distance, 'target_bearing': bearing,
            'fused_heading': heading, 'heading_error': heading_error,
            'timestamp': time.time(),
        }
        self._state_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))


def main():
    rclpy.init()
    node = GpsRosController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
