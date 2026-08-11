#!/usr/bin/python3
"""Direct GPS heading controller with reactive LiDAR obstacle avoidance.

GPS+ROS mode deliberately does not use Nav2 planning. GPS and an absolute
heading source point the robot toward the active waypoint, while LaserScan
locally slows or turns the robot around obstacles. Indoor mode continues to
use Nav2 through its separate /cmd_vel channel. JY901S yaw remains diagnostic
only and is never used as an absolute heading source.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String


MODE_GPS_ROS = 1


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


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
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('arrival_radius', 1.0)
        self.declare_parameter('waypoint_dwell', 5.0)
        self.declare_parameter('sensor_timeout', 0.75)
        self.declare_parameter('scan_timeout', 0.60)
        self.declare_parameter('cruise_speed', 0.22)
        self.declare_parameter('minimum_drive_speed', 0.07)
        self.declare_parameter('heading_kp', 1.15)
        self.declare_parameter('max_angular_speed', 0.85)
        self.declare_parameter('turn_in_place_angle_deg', 55.0)
        self.declare_parameter('front_stop_distance', 0.42)
        self.declare_parameter('front_slow_distance', 0.95)
        self.declare_parameter('side_stop_distance', 0.30)
        self.declare_parameter('obstacle_turn_speed', 0.70)
        self.declare_parameter('front_half_angle_deg', 28.0)
        self.declare_parameter('side_sector_angle_deg', 95.0)

        self._arrival_radius = float(self.get_parameter('arrival_radius').value)
        self._dwell_seconds = float(self.get_parameter('waypoint_dwell').value)
        self._sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self._scan_timeout = float(self.get_parameter('scan_timeout').value)
        self._cruise_speed = float(self.get_parameter('cruise_speed').value)
        self._minimum_drive_speed = float(
            self.get_parameter('minimum_drive_speed').value)
        self._heading_kp = float(self.get_parameter('heading_kp').value)
        self._max_angular = float(self.get_parameter('max_angular_speed').value)
        self._turn_in_place_angle = float(
            self.get_parameter('turn_in_place_angle_deg').value)
        self._front_stop = float(self.get_parameter('front_stop_distance').value)
        self._front_slow = max(
            self._front_stop + 0.05,
            float(self.get_parameter('front_slow_distance').value))
        self._side_stop = float(self.get_parameter('side_stop_distance').value)
        self._obstacle_turn = float(self.get_parameter('obstacle_turn_speed').value)
        self._front_half_angle = math.radians(float(
            self.get_parameter('front_half_angle_deg').value))
        self._side_sector_angle = math.radians(float(
            self.get_parameter('side_sector_angle_deg').value))

        self._mode = 2
        self._sensor = None
        self._sensor_time = 0.0
        self._scan = None
        self._scan_time = 0.0
        self._route = {}
        self._route_total = 0
        self._cloud_route = []
        self._cloud_mission_active = False
        self._cloud_mission_configured = False
        self._patrol_mode = 'ONCE'
        self._patrol_direction = 1
        self._mission_dwell_seconds = self._dwell_seconds
        self._waypoint_index = 0
        self._dwell_until = 0.0
        self._control_enabled = False
        self._avoid_direction = 0
        self._last_state_signature = None
        self._last_state_publish = 0.0

        self.create_subscription(Int8, '/robot_mode', self._on_mode, 10)
        self.create_subscription(String, '/stm32/sensors', self._on_sensors, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(
            String, '/gps_ros/mission_command', self._on_mission_command, 10)
        self._cmd_pub = self.create_publisher(Twist, '/gps_ros/cmd_vel', 10)
        self._state_pub = self.create_publisher(String, '/gps_ros/state', 10)
        self._enable_pub = self.create_publisher(Bool, '/gps_ros/control_enabled', 10)

        control_hz = max(2.0, float(self.get_parameter('control_hz').value))
        self.create_timer(1.0 / control_hz, self._tick)
        self.get_logger().info(
            'GPS+ROS direct controller ready: no Nav2 planning, '
            f'control={control_hz:.1f}Hz speed={self._cruise_speed:.2f}m/s '
            f'obstacle_stop={self._front_stop:.2f}m')

    def _on_mode(self, msg):
        new_mode = int(msg.data)
        if new_mode == self._mode:
            return
        self._mode = new_mode
        self._waypoint_index = 0
        self._dwell_until = 0.0
        self._avoid_direction = 0
        if self._mode != MODE_GPS_ROS:
            self._cloud_mission_active = False
            self._cloud_mission_configured = False
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
        if data.get('route_valid') and 0 <= int(data.get('route_slot', -1)) < total:
            slot = int(data['route_slot'])
            self._route[slot] = (
                float(data.get('route_latitude', 0.0)),
                float(data.get('route_longitude', 0.0)))

    def _on_scan(self, msg):
        self._scan = msg
        self._scan_time = time.monotonic()

    def _on_mission_command(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        command = str(data.get('command', '')).upper()
        if command == 'GPS_ROS_MISSION_CANCEL':
            self._cloud_mission_active = False
            self._cloud_mission_configured = True
            self._cloud_route = []
            self._waypoint_index = 0
            self._stop('MISSION_CANCELLED')
            return
        if command != 'GPS_ROS_MISSION_START':
            return

        route = []
        raw_waypoints = data.get('waypoints', [])
        if not isinstance(raw_waypoints, list):
            raw_waypoints = []
        for point in raw_waypoints[:50]:
            try:
                latitude = float(point['latitude'])
                longitude = float(point['longitude'])
            except (KeyError, TypeError, ValueError):
                continue
            if (math.isfinite(latitude) and math.isfinite(longitude) and
                    -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
                route.append((latitude, longitude))
        if not route:
            self._stop('MISSION_REJECTED_EMPTY')
            return

        patrol_mode = str(data.get('patrol_mode', 'ONCE')).upper()
        if patrol_mode not in ('ONCE', 'LOOP', 'PING_PONG'):
            patrol_mode = 'ONCE'
        self._cloud_route = route
        self._cloud_mission_active = True
        self._cloud_mission_configured = True
        self._patrol_mode = patrol_mode
        self._patrol_direction = 1
        try:
            dwell_seconds = float(data.get('dwell_seconds', self._dwell_seconds))
        except (TypeError, ValueError):
            dwell_seconds = self._dwell_seconds
        self._mission_dwell_seconds = clamp(dwell_seconds, 0.0, 60.0)
        self._waypoint_index = 0
        self._dwell_until = 0.0
        self._avoid_direction = 0
        self._publish_state('MISSION_ACCEPTED')

    def _active_route(self):
        if self._cloud_mission_configured:
            return self._cloud_route, self._cloud_mission_active, self._patrol_mode
        if self._route_total > 0 and len(self._route) >= self._route_total:
            route = [self._route[index] for index in range(self._route_total)]
            patrol = 'LOOP' if self._sensor.get('loop_enable') else 'ONCE'
            return route, bool(self._sensor.get('navigation_active')), patrol
        return [], False, 'ONCE'

    def _advance_waypoint(self, route_total, patrol_mode):
        if patrol_mode == 'PING_PONG' and route_total > 1:
            next_index = self._waypoint_index + self._patrol_direction
            if next_index < 0 or next_index >= route_total:
                self._patrol_direction *= -1
                next_index = self._waypoint_index + self._patrol_direction
            self._waypoint_index = next_index
            return True
        if self._waypoint_index + 1 < route_total:
            self._waypoint_index += 1
            return True
        if patrol_mode == 'LOOP' and route_total > 1:
            self._waypoint_index = 0
            return True
        return False

    def _heading(self):
        if self._sensor.get('gnss_heading_valid'):
            value = float(self._sensor['gnss_heading'])
            if math.isfinite(value):
                return value % 360.0, 'GNSS_DUAL'
        if self._sensor.get('mag_valid'):
            value = float(self._sensor['mag_yaw'])
            if math.isfinite(value):
                return value % 360.0, 'QMC5883'
        return None, 'NONE'

    def _sector_clearance(self, start_angle, end_angle):
        scan = self._scan
        if scan is None or not scan.ranges or scan.angle_increment == 0.0:
            return 0.0
        values = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if start_angle <= angle <= end_angle and math.isfinite(distance):
                if scan.range_min <= distance <= scan.range_max:
                    values.append(float(distance))
            angle += scan.angle_increment
        if not values:
            return (scan.range_max if math.isfinite(scan.range_max) and
                    scan.range_max > 0.0 else 10.0)
        values.sort()
        # Ignore at most two isolated bad returns without hiding a real object.
        return values[min(2, len(values) - 1)]

    def _scan_clearances(self):
        front = self._sector_clearance(
            -self._front_half_angle, self._front_half_angle)
        left = self._sector_clearance(
            self._front_half_angle * 0.5, self._side_sector_angle)
        right = self._sector_clearance(
            -self._side_sector_angle, -self._front_half_angle * 0.5)
        return front, left, right

    def _choose_avoid_direction(self, left, right, heading_error_cw):
        if self._avoid_direction != 0 and abs(left - right) < 0.35:
            return self._avoid_direction
        if left > right + 0.08:
            return 1   # ROS angular.z positive turns left.
        if right > left + 0.08:
            return -1
        return -1 if heading_error_cw > 0.0 else 1

    def _compute_command(self, heading_error_cw, front, left, right):
        heading_error_rad = math.radians(heading_error_cw)
        heading_angular = clamp(
            -self._heading_kp * heading_error_rad,
            -self._max_angular, self._max_angular)

        if abs(heading_error_cw) >= self._turn_in_place_angle:
            linear = 0.0
        else:
            alignment = max(0.25, math.cos(heading_error_rad))
            linear = self._cruise_speed * alignment

        state = 'NAVIGATING'
        obstacle_active = False
        if front <= self._front_stop:
            self._avoid_direction = self._choose_avoid_direction(
                left, right, heading_error_cw)
            linear = 0.0
            angular = self._avoid_direction * self._obstacle_turn
            state = 'OBSTACLE_TURN'
            obstacle_active = True
        elif front < self._front_slow:
            self._avoid_direction = self._choose_avoid_direction(
                left, right, heading_error_cw)
            ratio = clamp(
                (front - self._front_stop) /
                (self._front_slow - self._front_stop), 0.0, 1.0)
            linear = min(linear, max(self._minimum_drive_speed,
                                     self._cruise_speed * ratio))
            avoid_angular = self._avoid_direction * (
                0.38 + (1.0 - ratio) * (self._obstacle_turn - 0.38))
            angular = clamp(
                avoid_angular + 0.20 * heading_angular,
                -self._max_angular, self._max_angular)
            state = 'OBSTACLE_AVOIDING'
            obstacle_active = True
        else:
            if front > self._front_slow + 0.15:
                self._avoid_direction = 0
            angular = heading_angular
            if left < self._side_stop:
                angular = min(angular, -0.40)
                state = 'LEFT_SIDE_CLEARANCE'
                obstacle_active = True
            elif right < self._side_stop:
                angular = max(angular, 0.40)
                state = 'RIGHT_SIDE_CLEARANCE'
                obstacle_active = True

        return linear, angular, state, obstacle_active

    def _tick(self):
        if self._mode != MODE_GPS_ROS:
            self._set_enabled(False)
            return
        now = time.monotonic()
        if self._sensor is None or now - self._sensor_time > self._sensor_timeout:
            self._stop('STM32_TIMEOUT')
            return
        if self._scan is None or now - self._scan_time > self._scan_timeout:
            self._stop('LIDAR_TIMEOUT')
            return
        if not self._sensor.get('gps_valid'):
            self._stop('GPS_INVALID')
            return
        heading, heading_source = self._heading()
        if heading is None:
            self._stop('HEADING_INVALID')
            return
        route, mission_active, patrol_mode = self._active_route()
        route_total = len(route)
        if route_total <= 0:
            self._stop('ROUTE_INCOMPLETE')
            return
        if not mission_active:
            self._stop('MISSION_INACTIVE')
            return
        if now < self._dwell_until:
            self._stop('WAYPOINT_DWELL')
            return

        current_lat = float(self._sensor['latitude'])
        current_lon = float(self._sensor['longitude'])
        if self._waypoint_index >= route_total:
            self._waypoint_index = 0
        target_lat, target_lon = route[self._waypoint_index]
        distance, bearing = distance_and_bearing(
            current_lat, current_lon, target_lat, target_lon)

        if distance <= self._arrival_radius:
            if not self._advance_waypoint(route_total, patrol_mode):
                self._cloud_mission_active = False
                self._stop('MISSION_COMPLETE')
                return
            dwell = (self._mission_dwell_seconds if self._cloud_mission_active
                     else self._dwell_seconds)
            self._dwell_until = now + dwell
            self._avoid_direction = 0
            self._stop('WAYPOINT_REACHED')
            return

        heading_error_cw = normalize_degrees(bearing - heading)
        front, left, right = self._scan_clearances()
        linear, angular, state, obstacle_active = self._compute_command(
            heading_error_cw, front, left, right)

        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self._cmd_pub.publish(command)
        self._set_enabled(True)
        self._publish_state(
            state, heading_source, distance, bearing, heading, heading_error_cw,
            front, left, right, linear, angular, obstacle_active)

    def _set_enabled(self, enabled):
        enabled = bool(enabled)
        if enabled != self._control_enabled:
            self._control_enabled = enabled
            self._enable_pub.publish(Bool(data=enabled))

    def _stop(self, state):
        self._cmd_pub.publish(Twist())
        self._set_enabled(False)
        self._publish_state(state)

    def _publish_state(self, state, heading_source='NONE', distance=None,
                       bearing=None, heading=None, heading_error=None,
                       front=None, left=None, right=None, linear=0.0,
                       angular=0.0, obstacle_active=False):
        payload = {
            'state': state,
            'controller': 'GPS_LIDAR_REACTIVE',
            'path_planning': False,
            'control_enabled': self._control_enabled,
            'waypoint_index': self._waypoint_index,
            'waypoint_total': (len(self._cloud_route) if self._cloud_mission_configured
                               else self._route_total),
            'mission_source': ('WECHAT_GPS' if self._cloud_mission_configured else 'STM32'),
            'patrol_mode': self._patrol_mode if self._cloud_mission_configured else None,
            'heading_source': heading_source,
            'distance_to_target': distance,
            'target_bearing': bearing,
            'fused_heading': heading,
            'heading_error': heading_error,
            'lidar_front': front,
            'lidar_left': left,
            'lidar_right': right,
            'obstacle_active': obstacle_active,
            'command_linear': linear,
            'command_angular': angular,
            'timestamp': time.time(),
        }
        signature_payload = dict(payload)
        signature_payload.pop('timestamp', None)
        signature = json.dumps(signature_payload, sort_keys=True, separators=(',', ':'))
        now = time.monotonic()
        if signature == self._last_state_signature and now - self._last_state_publish < 0.5:
            return
        self._last_state_signature = signature
        self._last_state_publish = now
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
