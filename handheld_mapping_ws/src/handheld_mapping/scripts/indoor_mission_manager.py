#!/usr/bin/python3
"""Nav2 multi-waypoint indoor patrol manager."""

import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int8, String
from tf2_ros import Buffer, TransformListener


MODE_INDOOR = 3


class IndoorMissionManager(Node):
    def __init__(self):
        super().__init__('indoor_mission_manager')
        self._action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._state_pub = self.create_publisher(String, '/indoor/mission_state', 10)
        self.create_subscription(String, '/indoor/mission_command', self._on_command, 10)
        self.create_subscription(Int8, '/robot_mode', self._on_mode, 10)
        self.create_timer(0.2, self._tick)

        self._mode = 1
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
        self._generation = 0
        self.get_logger().info('Indoor mission manager ready')

    def _on_mode(self, msg: Int8):
        self._mode = int(msg.data)
        if self._active and self._mode != MODE_INDOOR:
            self._cancel('MODE_CHANGED')

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
        self._last_distance = None
        self._last_feedback_publish = 0.0
        self._publish_state('STARTED', message='Mission accepted')

    def _tick(self):
        if not self._active or self._goal_active:
            return
        if self._mode != MODE_INDOOR:
            self._publish_state('WAITING_FOR_INDOOR_MODE', message='Waiting for indoor mode')
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
            self._active = False
            self._publish_state('FAILED', message=f'Nav2 status={status}')
            return

        reached = self._waypoints[self._index]
        self._publish_state('WAYPOINT_REACHED', message=f"到达{reached['name']}")
        if not self._advance_index():
            self._active = False
            self._publish_state('COMPLETED', message='Mission completed')
            return
        self._dwell_until = time.monotonic() + self._dwell_seconds

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
        was_active = self._active
        self._active = False
        self._dwell_until = 0.0
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
        self._publish_state(
            'RECORDED_POINT',
            message='Current pose recorded',
            extra={
                'request_id': data.get('request_id', ''),
                'point': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
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
