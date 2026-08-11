#!/usr/bin/python3
"""Nav2 multi-waypoint indoor patrol manager."""

import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Int8, String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


MODE_INDOOR = 3


class IndoorMissionManager(Node):
    def __init__(self):
        super().__init__('indoor_mission_manager')
        # Indoor obstacles often need more than one clear/backup/rotate cycle
        # before the planner finds a new corridor.
        self.declare_parameter('max_goal_retries', 5)
        self.declare_parameter('retry_delay', 1.5)
        self.declare_parameter('direct_control', True)
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('goal_radius', 0.30)
        self.declare_parameter('goal_yaw_tolerance', 0.18)
        self.declare_parameter('position_kp', 1.10)
        self.declare_parameter('position_ki', 0.02)
        self.declare_parameter('position_kd', 0.06)
        self.declare_parameter('position_integral_limit', 0.40)
        self.declare_parameter('max_linear_speed', 0.24)
        self.declare_parameter('angle_kp', 2.20)
        self.declare_parameter('angle_ki', 0.04)
        self.declare_parameter('angle_kd', 0.18)
        self.declare_parameter('angle_integral_limit', 0.50)
        self.declare_parameter('max_angular_speed', 0.78)
        self.declare_parameter('final_turn_speed_scale', 1.5)
        self.declare_parameter('turn_in_place_angle', 0.70)
        self.declare_parameter('obstacle_turn_speed', 0.68)
        self.declare_parameter('obstacle_turn_speed_scale', 2.0)
        self.declare_parameter('front_stop_distance', 0.34)
        self.declare_parameter('front_slow_distance', 0.68)
        self.declare_parameter('scan_timeout', 0.80)
        self._max_goal_retries = max(
            0, int(self.get_parameter('max_goal_retries').value))
        self._retry_delay = max(
            0.0, float(self.get_parameter('retry_delay').value))
        self._direct_control = bool(self.get_parameter('direct_control').value)
        self._control_hz = max(
            2.0, float(self.get_parameter('control_hz').value))
        self._goal_radius = float(self.get_parameter('goal_radius').value)
        self._goal_yaw_tolerance = max(
            0.03, float(self.get_parameter('goal_yaw_tolerance').value))
        self._position_kp = float(self.get_parameter('position_kp').value)
        self._position_ki = float(self.get_parameter('position_ki').value)
        self._position_kd = float(self.get_parameter('position_kd').value)
        self._position_integral_limit = max(
            0.0, float(self.get_parameter('position_integral_limit').value))
        self._max_linear = float(self.get_parameter('max_linear_speed').value)
        self._angle_kp = float(self.get_parameter('angle_kp').value)
        self._angle_ki = float(self.get_parameter('angle_ki').value)
        self._angle_kd = float(self.get_parameter('angle_kd').value)
        self._angle_integral_limit = max(
            0.0, float(self.get_parameter('angle_integral_limit').value))
        self._max_angular = float(self.get_parameter('max_angular_speed').value)
        self._final_turn_speed_scale = max(
            1.0, float(self.get_parameter('final_turn_speed_scale').value))
        self._turn_in_place_angle = max(
            0.05, float(self.get_parameter('turn_in_place_angle').value))
        self._obstacle_turn_speed = max(
            0.0, float(self.get_parameter('obstacle_turn_speed').value))
        self._obstacle_turn_speed_scale = max(
            1.0, float(self.get_parameter('obstacle_turn_speed_scale').value))
        self._front_stop = float(self.get_parameter('front_stop_distance').value)
        self._front_slow = max(
            self._front_stop + 0.05,
            float(self.get_parameter('front_slow_distance').value))
        self._scan_timeout = max(
            0.1, float(self.get_parameter('scan_timeout').value))

        self._action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._clear_local_costmap = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global_costmap = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # Feed the direct controller into the existing velocity smoother; this
        # keeps one deterministic command path to the STM32 bridge while
        # bypassing Nav2 planning and control actions.
        self._cmd_pub = self.create_publisher(Twist, '/direct_cmd_vel', 10)
        self._scan = None
        self._scan_time = 0.0
        self._imu_wz = 0.0
        self._imu_time = 0.0
        self._state_pub = self.create_publisher(String, '/indoor/mission_state', 10)
        self.create_subscription(String, '/indoor/mission_command', self._on_command, 10)
        self.create_subscription(Int8, '/robot_mode', self._on_mode, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data_raw', self._on_imu, 10)
        self.create_timer(1.0 / self._control_hz, self._tick)

        self._mode = 2
        self._active = False
        self._goal_active = False
        self._goal_handle = None
        self._waypoints = []
        self._index = 0
        self._direction = 1
        self._patrol_mode = 'PING_PONG'
        self._dwell_seconds = 2.0
        self._dwell_until = 0.0
        self._mission_id = ''
        self._last_distance = None
        self._last_feedback_publish = 0.0
        self._last_state_signature = None
        self._last_state_publish = 0.0
        self._last_record_time = 0.0
        self._last_record_xy = None
        self._last_control_time = 0.0
        self._position_integral_x = 0.0
        self._position_integral_y = 0.0
        self._previous_error_x = 0.0
        self._previous_error_y = 0.0
        self._angle_integral = 0.0
        self._previous_angle_error = 0.0
        self._avoid_direction = 0
        self._turn_direction = 0
        self._position_reached = False
        self._generation = 0
        self._goal_retry_count = 0
        self.get_logger().info(
            f'Indoor mission manager ready: retries={self._max_goal_retries} '
            f'retry_delay={self._retry_delay:.1f}s '
            f'direct_pid={self._control_hz:.1f}Hz')

    def _on_mode(self, msg: Int8):
        self._mode = int(msg.data)
        if self._active and self._mode != MODE_INDOOR:
            self._cancel('MODE_CHANGED')

    def _on_scan(self, msg):
        self._scan = msg
        self._scan_time = time.monotonic()

    def _on_imu(self, msg):
        self._imu_wz = float(msg.angular_velocity.z)
        self._imu_time = time.monotonic()

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _reset_direct_pid(self):
        self._last_control_time = 0.0
        self._position_integral_x = 0.0
        self._position_integral_y = 0.0
        self._previous_error_x = 0.0
        self._previous_error_y = 0.0
        self._angle_integral = 0.0
        self._previous_angle_error = 0.0
        self._avoid_direction = 0

    def _clearance(self, lo, hi):
        vals = []
        a = self._scan.angle_min
        for r in self._scan.ranges:
            if lo <= a <= hi and math.isfinite(r) and r > max(0.08, self._scan.range_min):
                vals.append(r)
            a += self._scan.angle_increment
        return min(vals) if vals else 3.0

    def _direct_tick(self):
        if not self._waypoints:
            return
        now = time.monotonic()
        if self._scan is None or now - self._scan_time > self._scan_timeout:
            self._publish_state(
                'WAITING_FOR_SCAN', message='Laser scan missing or stale')
            self._stop_cmd()
            self._reset_direct_pid()
            return
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            self._publish_state('WAITING_FOR_POSE', message='Waiting for map pose')
            self._stop_cmd()
            self._reset_direct_pid()
            return
        p = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        goal = self._waypoints[self._index]
        error_x = goal['x'] - p.x
        error_y = goal['y'] - p.y
        dist = math.hypot(error_x, error_y)
        self._last_distance = dist

        dt = (now - self._last_control_time
              if self._last_control_time > 0.0 else 1.0 / self._control_hz)
        dt = max(0.02, min(0.25, dt))
        self._last_control_time = now

        # Two independent world-frame position PID loops produce a desired
        # velocity vector. A differential-drive chassis follows its direction
        # with the heading loop and its magnitude with linear velocity.
        self._position_integral_x = self._clamp(
            self._position_integral_x + error_x * dt,
            self._position_integral_limit)
        self._position_integral_y = self._clamp(
            self._position_integral_y + error_y * dt,
            self._position_integral_limit)
        derivative_x = (error_x - self._previous_error_x) / dt
        derivative_y = (error_y - self._previous_error_y) / dt
        self._previous_error_x = error_x
        self._previous_error_y = error_y
        control_x = (self._position_kp * error_x +
                     self._position_ki * self._position_integral_x +
                     self._position_kd * derivative_x)
        control_y = (self._position_kp * error_y +
                     self._position_ki * self._position_integral_y +
                     self._position_kd * derivative_y)

        # Position arrival is latched for the current waypoint. RF2O position
        # naturally drifts a few centimetres during an in-place turn; without
        # this hysteresis, distance crossing the goal-radius boundary switches
        # desired_yaw between final heading and point bearing by nearly 180°.
        if dist <= self._goal_radius:
            self._position_reached = True
        near_goal = self._position_reached
        desired_yaw = (goal['yaw'] if near_goal
                       else math.atan2(control_y, control_x))
        angle_error = self._wrap(desired_yaw - yaw)
        self._angle_integral = self._clamp(
            self._angle_integral + angle_error * dt,
            self._angle_integral_limit)

        imu_fresh = now - self._imu_time < 0.5
        if imu_fresh:
            # Use the measured yaw direction only to decide whether motion is
            # reducing the absolute error. This avoids assuming that the
            # STM32/JY901S sign convention is identical to ROS yaw.
            angle_derivative = 0.0
        else:
            angle_derivative = self._wrap(
                angle_error - self._previous_angle_error) / dt
        self._previous_angle_error = angle_error
        angular = (self._angle_kp * angle_error +
                   self._angle_ki * self._angle_integral +
                   self._angle_kd * angle_derivative)
        if imu_fresh and abs(angle_error) > 1e-3:
            # IMU damping is sign-robust: when yaw is moving toward the goal,
            # reduce the command; when moving away, add damping in the error
            # direction. The map TF remains the absolute angle reference.
            damping = self._angle_kd * abs(self._imu_wz)
            if angle_error * self._imu_wz > 0.0:
                angular -= math.copysign(damping, angle_error)
            else:
                angular += math.copysign(damping, angle_error)
        angular = self._clamp(angular, self._max_angular)

        if near_goal and abs(angle_error) <= self._goal_yaw_tolerance:
            self._stop_cmd()
            self._reset_direct_pid()
            self._publish_state('WAYPOINT_REACHED', message=f"到达{goal['name']}")
            if not self._advance_index():
                self._active = False
                self._publish_state('COMPLETED', message='Mission completed')
            else:
                self._position_reached = False
                self._reset_direct_pid()
                self._dwell_until = now + self._dwell_seconds
            return

        left = self._clearance(0.15, 1.45)
        right = self._clearance(-1.45, -0.15)
        front = self._clearance(-0.35, 0.35)
        cmd = Twist()

        turning_in_place = near_goal or abs(angle_error) >= self._turn_in_place_angle
        blocked = front <= self._front_stop
        if turning_in_place:
            # LiDAR only gates translational motion. Pure rotation must keep
            # running even if an obstacle is visible in the front sector.
            if abs(angle_error) >= self._turn_in_place_angle:
                # A large target turn (including final yaw alignment at a
                # waypoint) has an inherently ambiguous shortest-turn sign
                # when TF yaw jitters. Latch the first valid side until the
                # robot leaves the large-angle region, preventing chatter.
                if self._turn_direction == 0:
                    self._turn_direction = 1 if angle_error >= 0.0 else -1
                angular = self._turn_direction * abs(angular)
            else:
                self._turn_direction = 0
            if ((near_goal and abs(angle_error) > self._goal_yaw_tolerance) or
                    abs(angle_error) >= math.pi / 2.0):
                # Use 1.5x angular speed for final yaw alignment and U-turns
                # of 90 degrees or more, while retaining direction latching.
                angular = self._clamp(
                    angular * self._final_turn_speed_scale,
                    self._max_angular * self._final_turn_speed_scale)
            cmd.angular.z = angular
            self._avoid_direction = 0
        elif blocked:
            # Stop translation and rotate toward the clearer side. Keep the
            # selected side until the front corridor opens to avoid chatter.
            if self._avoid_direction == 0:
                if abs(left - right) < 0.12:
                    self._avoid_direction = 1 if angle_error >= 0.0 else -1
                else:
                    self._avoid_direction = 1 if left > right else -1
            cmd.angular.z = (
                self._avoid_direction * self._obstacle_turn_speed *
                self._obstacle_turn_speed_scale)
        else:
            self._avoid_direction = 0
            position_speed = math.hypot(control_x, control_y)
            # Project the world-frame position effort onto the chassis forward
            # axis. Heading error therefore reduces speed without another
            # planner or trajectory controller.
            heading_scale = max(0.0, math.cos(angle_error))
            cmd.linear.x = min(
                self._max_linear, position_speed) * heading_scale
            if front < self._front_slow:
                cmd.linear.x *= max(
                    0.0, (front - self._front_stop) /
                    (self._front_slow - self._front_stop))
            cmd.angular.z = angular
        self._cmd_pub.publish(cmd)
        self._publish_state(
            'AVOIDING' if blocked and not turning_in_place else 'NAVIGATING',
            message='XY position PID + IMU-damped heading PID',
            extra={
                'control': {
                    'error_x': round(error_x, 3),
                    'error_y': round(error_y, 3),
                    'angle_error': round(angle_error, 3),
                    'linear': round(cmd.linear.x, 3),
                    'angular': round(cmd.angular.z, 3),
                    'imu_feedback': imu_fresh,
                    'front_clearance': round(front, 3),
                },
            })

    def _stop_cmd(self):
        self._cmd_pub.publish(Twist())

    def _on_command(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_state('ERROR', message=f'Invalid mission JSON: {exc}')
            return

        command = str(data.get('command', '')).upper()
        if command == 'INDOOR_MISSION_START':
            self._start(data)
        elif command == 'INDOOR_MISSION_CANCEL':
            self._cancel('USER_CANCELLED')
        elif command == 'INDOOR_RECORD_POINT':
            self._record_current_pose(data)

    def _start(self, data):
        points = data.get('waypoints') or []
        waypoints = []
        for index, point in enumerate(points):
            try:
                waypoints.append({
                    'id': point.get('id', index + 1),
                    'name': point.get('name', f'目标点 {index + 1}'),
                    'x': float(point['x']),
                    'y': float(point['y']),
                    'yaw': float(point.get('yaw', 0.0)),
                })
            except (KeyError, TypeError, ValueError):
                self._publish_state('ERROR', message=f'Invalid waypoint {index + 1}')
                return
        if not waypoints:
            self._publish_state('ERROR', message='No waypoints')
            return

        self._generation += 1
        self._cancel_goal_only()
        self._waypoints = waypoints
        self._index = 0
        self._direction = 1
        self._patrol_mode = str(data.get('patrol_mode', 'PING_PONG')).upper()
        if self._patrol_mode not in ('ONCE', 'LOOP', 'PING_PONG'):
            self._patrol_mode = 'PING_PONG'
        self._dwell_seconds = max(0.0, min(60.0, float(data.get('dwell_seconds', 2.0))))
        self._mission_id = str(data.get('mission_id', f'mission_{int(time.time())}'))
        self._active = True
        self._goal_active = False
        self._dwell_until = 0.0
        self._position_reached = False
        self._reset_direct_pid()
        self._last_distance = None
        self._last_feedback_publish = 0.0
        self._goal_retry_count = 0
        self._publish_state('STARTED', message='Mission accepted')

    def _tick(self):
        if not self._active or self._goal_active:
            return
        if self._mode != MODE_INDOOR:
            self._publish_state('WAITING_FOR_INDOOR_MODE', message='Waiting for indoor mode')
            return
        if self._direct_control:
            if self._dwell_until > time.monotonic():
                self._stop_cmd(); return
            self._direct_tick()
            return
        if self._dwell_until > time.monotonic():
            return
        if not self._action.server_is_ready():
            self._publish_state('WAITING_FOR_NAV2', message='NavigateToPose action unavailable')
            return
        self._send_current_goal()

    def _send_current_goal(self):
        point = self._waypoints[self._index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point['x']
        goal.pose.pose.position.y = point['y']
        goal.pose.pose.orientation = Quaternion(
            z=math.sin(point['yaw'] / 2.0),
            w=math.cos(point['yaw'] / 2.0))
        self._goal_active = True
        generation = self._generation
        future = self._action.send_goal_async(
            goal,
            feedback_callback=lambda msg: self._on_feedback(msg, generation))
        future.add_done_callback(lambda result: self._on_goal_response(result, generation))
        self._publish_state('NAVIGATING', message=f"前往{point['name']}")

    def _on_goal_response(self, future, generation):
        if generation != self._generation:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self._goal_active = False
            self._active = False
            self._publish_state('ERROR', message=f'Goal request failed: {exc}')
            return
        if not handle.accepted:
            self._goal_active = False
            self._active = False
            self._publish_state('GOAL_REJECTED', message='Nav2 rejected goal')
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda result: self._on_result(result, generation))

    def _on_feedback(self, feedback_msg, generation):
        if generation != self._generation:
            return
        feedback = feedback_msg.feedback
        self._last_distance = float(getattr(feedback, 'distance_remaining', 0.0))
        now = time.monotonic()
        if now - self._last_feedback_publish < 0.5:
            return
        self._last_feedback_publish = now
        self._publish_state('NAVIGATING', message='Navigation in progress')

    def _on_result(self, future, generation):
        if generation != self._generation:
            return
        self._goal_active = False
        self._goal_handle = None
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self._publish_state('ERROR', message=f'Navigation result failed: {exc}')
            return
        if not self._active:
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            if (status == GoalStatus.STATUS_ABORTED and
                    self._mode == MODE_INDOOR and
                    self._goal_retry_count < self._max_goal_retries):
                self._goal_retry_count += 1
                self._last_distance = None
                self._clear_costmaps()
                self._dwell_until = time.monotonic() + self._retry_delay
                self._publish_state(
                    'RETRYING',
                    message=(f'Nav2 aborted; retrying waypoint '
                             f'{self._goal_retry_count}/{self._max_goal_retries}'))
                self.get_logger().warning(
                    f'Waypoint {self._index + 1} aborted; clearing costmaps and '
                    f'retrying ({self._goal_retry_count}/{self._max_goal_retries})')
                return
            self._active = False
            self._publish_state(
                'FAILED',
                message=(f'Nav2 status={status}; retries exhausted '
                         f'({self._goal_retry_count}/{self._max_goal_retries})'))
            return

        reached = self._waypoints[self._index]
        self._goal_retry_count = 0
        self._publish_state('WAYPOINT_REACHED', message=f"到达{reached['name']}")
        if not self._advance_index():
            self._active = False
            self._publish_state('COMPLETED', message='Mission completed')
            return
        self._dwell_until = time.monotonic() + self._dwell_seconds

    def _clear_costmaps(self):
        for name, client in (
                ('local', self._clear_local_costmap),
                ('global', self._clear_global_costmap)):
            if not client.service_is_ready():
                self.get_logger().warning(
                    f'{name} costmap clear service is not ready')
                continue
            try:
                client.call_async(ClearEntireCostmap.Request())
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to clear {name} costmap: {exc}')

    def _advance_index(self):
        count = len(self._waypoints)
        if count <= 1:
            return self._patrol_mode != 'ONCE'
        if self._patrol_mode == 'ONCE':
            if self._index >= count - 1:
                return False
            self._index += 1
        elif self._patrol_mode == 'LOOP':
            self._index = (self._index + 1) % count
        else:
            if self._direction > 0 and self._index >= count - 1:
                self._direction = -1
            elif self._direction < 0 and self._index <= 0:
                self._direction = 1
            self._index += self._direction
        return True

    def _cancel_goal_only(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._goal_active = False

    def _cancel(self, reason):
        self._generation += 1
        self._cancel_goal_only()
        self._stop_cmd()
        was_active = self._active
        self._active = False
        self._dwell_until = 0.0
        self._position_reached = False
        self._reset_direct_pid()
        self._goal_retry_count = 0
        if was_active or reason == 'USER_CANCELLED':
            self._publish_state('CANCELLED', message=reason)

    def _record_current_pose(self, data):
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception as exc:
            self._publish_state('ERROR', message=f'Cannot record pose: {exc}')
            return
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        now = time.monotonic()
        if (self._last_record_xy is not None and
                now - self._last_record_time < 8.0 and
                math.hypot(x - self._last_record_xy[0],
                           y - self._last_record_xy[1]) < 0.20):
            self.get_logger().warning(
                'Ignoring repeated record-point command at the same pose')
            return
        self._last_record_time = now
        self._last_record_xy = (x, y)
        self._publish_state(
            'RECORDED_POINT',
            message='Current pose recorded',
            extra={
                'request_id': data.get('request_id', ''),
                'point': {
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                },
            })

    def _publish_state(self, state, message='', extra=None):
        payload = {
            'type': 'indoor_mission_state',
            'state': state,
            'message': message,
            'mission_id': self._mission_id,
            'active': self._active,
            'current_waypoint': self._index + 1 if self._waypoints else 0,
            'total_waypoints': len(self._waypoints),
            'distance_remaining': (
                round(self._last_distance, 2) if self._last_distance is not None else None),
            'retry_count': self._goal_retry_count,
            'max_retries': self._max_goal_retries,
            'timestamp': time.time(),
        }
        if extra:
            payload.update(extra)
        signature_payload = dict(payload)
        signature_payload.pop('timestamp', None)
        signature = json.dumps(signature_payload, sort_keys=True, separators=(',', ':'))
        now = time.monotonic()
        if signature == self._last_state_signature and now - self._last_state_publish < 1.0:
            return
        self._last_state_signature = signature
        self._last_state_publish = now
        self._state_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))


def main():
    rclpy.init()
    node = IndoorMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
