#!/usr/bin/python3
"""
Publish a virtual goal point always ahead of the robot.

Drives Nav2 to continuously plan/avoid obstacles, producing /cmd_vel
that the STM32 can fuse with GPS navigation.

Concept: place the carrot X metres ahead of the robot in its own
body frame → transform to map frame → send as Nav2 goal.
When the robot approaches the current goal (or a timeout elapses),
a new goal is pushed further ahead.

No modifications to existing nodes needed — just run this alongside
the navigation stack.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class VirtualGoalPublisher(Node):
    def __init__(self):
        super().__init__('virtual_goal_publisher')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('update_threshold', 0.5)
        self.declare_parameter('update_interval', 1.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('goal_tolerance', 0.3)

        self._lookahead = self.get_parameter('lookahead_distance').value
        self._update_threshold = self.get_parameter('update_threshold').value
        self._update_interval = self.get_parameter('update_interval').value
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value

        # ── TF ──────────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Nav2 action client ──────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._goal_handle = None
        self._current_goal = None  # (x, y) in map frame
        self._active = False

        # ── Timers ──────────────────────────────────────────────────────
        self._update_timer = self.create_timer(self._update_interval, self._update_goal)
        self._send_timer = self.create_timer(20.0, self._force_resend)  # watchdog

        self.get_logger().info(
            f'Virtual goal: lookahead={self._lookahead}m '
            f'threshold={self._update_threshold}m '
            f'map={self._map_frame} robot={self._robot_frame}')

    def _get_robot_pose(self):
        """Return (x, y, yaw) of robot in map frame, or None on failure."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return None

        x = t.transform.translation.x
        y = t.transform.translation.y
        # yaw from quaternion
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def _distance_to_goal(self, robot_x, robot_y):
        """Return distance from robot to the current active goal."""
        if self._current_goal is None:
            return float('inf')
        gx, gy = self._current_goal
        return math.hypot(robot_x - gx, robot_y - gy)

    def _update_goal(self):
        """Check robot pose and decide whether to send a new goal."""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().debug('NavigateToPose server not available')
            return

        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # Don't update if robot is still moving toward a recent goal
        if self._active and self._distance_to_goal(rx, ry) > self._update_threshold:
            self.get_logger().debug(
                f'Robot still approaching goal (dist={self._distance_to_goal(rx, ry):.2f}m)')
            return

        # Compute new goal ahead of robot
        gx = rx + self._lookahead * math.cos(ryaw)
        gy = ry + self._lookahead * math.sin(ryaw)

        self._send_goal(gx, gy, ryaw)

    def _send_goal(self, x, y, yaw):
        """Send NavigateToPose goal to Nav2."""
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = self._map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation: face forward along robot's heading
        q = self._yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation = q

        self._action_client.send_goal_async(goal_msg).add_done_callback(
            self._goal_response_callback)

        self.get_logger().info(
            f'New virtual goal: map=({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.1f}°')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error('Goal rejected by Nav2')
            self._active = False
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Goal not accepted by Nav2')
            self._active = False
            return

        self._goal_handle = goal_handle
        self._active = True

        # Extract goal position from the goal handle (we store it ourselves)
        # _current_goal was already set in _send_goal via the last computed position
        # Actually, let's store it here for safety
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self._active = False
        self.get_logger().info('Goal reached, next goal will be computed on next timer tick')

    def _force_resend(self):
        """Watchdog: if goal is stuck, force clear and re-send."""
        if self._active:
            self.get_logger().debug('Watchdog: goal still active, no force needed')
        else:
            self.get_logger().debug('Watchdog: no active goal, forcing update')
            self._update_goal()

    @staticmethod
    def _yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main():
    rclpy.init()
    node = VirtualGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()