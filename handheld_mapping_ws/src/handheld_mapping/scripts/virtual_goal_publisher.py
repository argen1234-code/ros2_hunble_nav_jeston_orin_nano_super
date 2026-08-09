#!/usr/bin/python3
"""
Continuously publish a virtual goal ahead of the robot.

At each tick:
  1. Read robot pose via TF (map → base_footprint)
  2. Place a goal lookahead_distance metres ahead in the robot's heading
  3. Send it to Nav2 /navigate_to_pose

Nav2 re-plans on every update, /cmd_vel flows to STM32 for obstacle
avoidance fused with GPS cruise.

Usage:
  ros2 run handheld_mapping virtual_goal_publisher
  ros2 run handheld_mapping virtual_goal_publisher --ros-args -p lookahead_distance:=3.0 -p update_interval:=0.5
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int8


class VirtualGoalPublisher(Node):
    def __init__(self):
        super().__init__('virtual_goal_publisher')

        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('update_interval', 1.0)
        self.declare_parameter('min_goal_translation', 0.35)
        self.declare_parameter('min_goal_yaw_deg', 10.0)
        self.declare_parameter('require_valid_gps', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        self._lookahead = self.get_parameter('lookahead_distance').value
        self._update_interval = self.get_parameter('update_interval').value
        self._min_goal_translation = self.get_parameter('min_goal_translation').value
        self._min_goal_yaw = math.radians(
            self.get_parameter('min_goal_yaw_deg').value)
        self._require_valid_gps = self.get_parameter('require_valid_gps').value
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value

        # TF
        from tf2_ros import Buffer, TransformListener
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._exceptions = (LookupException, ConnectivityException, ExtrapolationException)

        # GPS heading from STM32 (degrees, 0=straight ahead, CW+)
        self._gps_heading = None
        self._heading_sub = self.create_subscription(
            Float32, '/gps_heading', self._on_gps_heading, 10)
        self._gps_valid = False
        self._gps_sub = self.create_subscription(
            NavSatFix, '/gps_fix', self._on_gps_fix, 10)

        # Robot mode: only publish goals in GPS mode (1)
        self._robot_mode = 2  # safe default: REMOTE until a mode command arrives
        self._mode_sub = self.create_subscription(
            Int8, '/robot_mode', self._on_robot_mode, 10)

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._last_goal = None
        self._goal_handle = None
        self._goal_request_pending = False

        # Periodic carrot update
        self._timer = self.create_timer(self._update_interval, self._tick)

        self.get_logger().info(
            f'Virtual goal: lookahead={self._lookahead}m '
            f'interval={self._update_interval}s '
            f'min_shift={self._min_goal_translation}m '
            f'map={self._map_frame} robot={self._robot_frame}')

    def _on_gps_heading(self, msg: Float32):
        self._gps_heading = msg.data

    def _on_gps_fix(self, msg: NavSatFix):
        self._gps_valid = (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and not (abs(msg.latitude) < 1e-7 and abs(msg.longitude) < 1e-7)
        )

    def _on_robot_mode(self, msg: Int8):
        new_mode = msg.data
        if new_mode != self._robot_mode:
            self._robot_mode = new_mode
            self._last_goal = None
            if new_mode != 1:
                self._cancel_goal()

    def _get_robot_pose(self):
        """Return (x, y, yaw) of robot in map frame, or None on failure."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except self._exceptions:
            return None

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def _tick(self):
        """Send virtual goal in GPS mode; idle in REMOTE/INDOOR modes."""
        if self._robot_mode != 1:  # GPS = 1
            self.get_logger().debug(f'Mode {self._robot_mode}: goal publishing paused')
            return

        if self._require_valid_gps and not self._gps_valid:
            self.get_logger().debug('GPS fix invalid: virtual goal publishing paused')
            return

        if self._action_client.server_is_ready() is False:
            self.get_logger().debug('Action server not ready')
            return

        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().debug('TF not available yet')
            return
        rx, ry, ryaw = pose

        # Goal direction: GPS heading from STM32 if available, else robot heading
        if self._gps_heading is not None and abs(self._gps_heading) > 0.01:
            goal_yaw = ryaw + math.radians(self._gps_heading)
            self.get_logger().debug(
                f'GPS heading={self._gps_heading:.1f}° goal_yaw={math.degrees(goal_yaw):.0f}°')
        else:
            goal_yaw = ryaw

        gx = rx + self._lookahead * math.cos(goal_yaw)
        gy = ry + self._lookahead * math.sin(goal_yaw)

        # Avoid continuously preempting Nav2. Re-issue the carrot only after
        # meaningful translation or heading change.
        if self._last_goal is not None:
            d = math.hypot(gx - self._last_goal[0], gy - self._last_goal[1])
            yaw_delta = abs(math.atan2(
                math.sin(goal_yaw - self._last_goal[2]),
                math.cos(goal_yaw - self._last_goal[2])))
            if d < self._min_goal_translation and yaw_delta < self._min_goal_yaw:
                return

        if self._goal_request_pending:
            return

        self._last_goal = (gx, gy, goal_yaw)
        self._send(gx, gy, goal_yaw)

    def _send(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        goal.pose.pose.orientation = q

        self._goal_request_pending = True
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f'Carrot → map=({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}°')

    def _on_goal_response(self, future):
        self._goal_request_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Virtual goal request failed: {exc}')
            return
        if handle.accepted:
            self._goal_handle = handle

    def _cancel_goal(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None
        self._goal_request_pending = False


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
