#!/usr/bin/python3
"""
Bidirectional MQTT bridge for Alibaba Cloud IoT Hub.

Publishes robot pose (TF map→base_footprint) to MQTT at a configurable
interval, and handles cloud commands:

  Mode commands:  GPS (1) / REMOTE (2) / LINE (3) → /robot_mode
  Move commands:  forward/backward/left/right/stop + speed → /remote_cmd_vel
  Other:          EMERGENCY → stop, TAKE_PHOTO → log

Usage:
  ros2 run handheld_mapping mqtt_bridge
  ros2 run handheld_mapping mqtt_bridge --ros-args -p client_id:="..." -p username:="..."
"""

import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8

import paho.mqtt.client as mqtt


# Mode constants (sync with STM32)
MODE_GPS    = 1
MODE_REMOTE = 2
MODE_LINE   = 3


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        # ── MQTT connection params ──────────────────────────────────
        self.declare_parameter('broker', 'iot-06z00gf86e5n0dr.mqtt.iothub.aliyuncs.com')
        self.declare_parameter('port', 1883)
        self.declare_parameter('client_id',
            'k1ck5t83zdZ.test|securemode=2,signmethod=hmacsha256,timestamp=1747068235323|')
        self.declare_parameter('username', 'test&k1ck5t83zdZ')
        self.declare_parameter('password',
            'dc2752751c8cd987419c9cb1d81ec37909a70d1b31d202eeb2ec7799bbd05017')

        # ── Topics ──────────────────────────────────────────────────
        self.declare_parameter('sub_topic', '/k1ck5t83zdZ/test/user/get')
        self.declare_parameter('pub_topic', '/k1ck5t83zdZ/test/user/robot')
        self.declare_parameter('gps_pub_topic', '/k1ck5t83zdZ/test/user/esp8266duan')

        # ── TF ──────────────────────────────────────────────────────
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('publish_interval', 1.0)

        # ── Speed limits ────────────────────────────────────────────
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 2.0)

        _broker   = self.get_parameter('broker').value
        _port     = self.get_parameter('port').value
        _cid      = self.get_parameter('client_id').value
        _user     = self.get_parameter('username').value
        _pw       = self.get_parameter('password').value

        self._sub_topic     = self.get_parameter('sub_topic').value
        self._pub_topic     = self.get_parameter('pub_topic').value
        self._gps_pub_topic = self.get_parameter('gps_pub_topic').value
        self._map_frame     = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._interval    = self.get_parameter('publish_interval').value
        self._max_linear  = self.get_parameter('max_linear').value
        self._max_angular = self.get_parameter('max_angular').value

        # ── State ───────────────────────────────────────────────────
        self._mode = MODE_GPS       # current robot mode
        self._speed_pct = 50        # speed percentage (0-100), default 50

        # ── TF buffer ───────────────────────────────────────────────
        from tf2_ros import Buffer, TransformListener
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_ex = (LookupException, ConnectivityException, ExtrapolationException)

        # ── ROS2 publishers ─────────────────────────────────────────
        self._mode_pub = self.create_publisher(Int8, '/robot_mode', 10)
        self._remote_cmd_pub = self.create_publisher(Twist, '/remote_cmd_vel', 10)

        # ── ROS2 subscriptions ──────────────────────────────────────
        self._gps_sub = self.create_subscription(
            NavSatFix, '/gps_fix', self._on_gps_fix, 10)

        # Publish initial mode
        self._mode_pub.publish(Int8(data=self._mode))

        # ── MQTT client ─────────────────────────────────────────────
        self._mqtt = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=_cid)
        self._mqtt.username_pw_set(_user, _pw)
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_message = self._on_message
        self._mqtt_connected = False

        self._mqtt_thread = threading.Thread(target=self._mqtt_loop, daemon=True)
        self._mqtt_thread.start()

        # Periodic pose publisher
        self._pose_timer = self.create_timer(self._interval, self._publish_pose)

        self.get_logger().info(
            f'MQTT bridge: broker={_broker}:{_port} pub→{self._pub_topic} sub←{self._sub_topic}')

    # ── MQTT callbacks ─────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self._mqtt_connected = True
            self.get_logger().info(f'MQTT connected, subscribing: {self._sub_topic}')
            client.subscribe(self._sub_topic, qos=1)
        else:
            self.get_logger().error(f'MQTT connect failed: rc={reason_code}')

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        self.get_logger().info(f'MQTT ← {msg.topic}: {payload}')

        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Non-JSON payload ignored: {payload}')
            return

        if 'command' in data:
            self._handle_thing_command(data)
        elif 'linear' in data or 'angular' in data:
            self._handle_direct_cmd_vel(data)
        else:
            self.get_logger().warn(f'Unknown command shape: {data}')

    # ── Thing Model command handler ────────────────────────────────────

    def _handle_thing_command(self, data):
        cmd = data.get('command', '').upper()
        spd = data.get('speed', self._speed_pct)

        if cmd in ('GPS', 'REMOTE', 'LINE'):
            self._handle_mode_switch(cmd)

        elif cmd == 'EMERGENCY':
            self.get_logger().info(f'[云] 急停 → cmd_vel=0')
            self._remote_cmd_pub.publish(Twist())

        elif cmd == 'TAKE_PHOTO':
            self.get_logger().info(f'[云] 拍照指令')

        elif cmd == 'SPEED':
            self._speed_pct = int(spd)
            self.get_logger().info(f'[云] 速度设为 {self._speed_pct}%')

        elif cmd in ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'STOP'):
            self._handle_move(cmd, int(spd))

        else:
            self.get_logger().info(f'[云] 未知指令: {cmd}, params={data}')

    # ── Mode switch ────────────────────────────────────────────────────

    def _handle_mode_switch(self, cmd: str):
        mode_map = {'GPS': MODE_GPS, 'REMOTE': MODE_REMOTE, 'LINE': MODE_LINE}
        new_mode = mode_map[cmd]
        if new_mode != self._mode:
            self._mode = new_mode
            self._mode_pub.publish(Int8(data=self._mode))
            self.get_logger().info(f'[云] 模式切换 → {cmd} (mode={self._mode})')
        else:
            # Same mode — still respond with pose for GPS
            if cmd == 'GPS':
                self._publish_pose()

    # ── Remote movement → /remote_cmd_vel ──────────────────────────────

    def _handle_move(self, direction: str, speed_pct: int):
        """Convert direction + speed% into a Twist, publish to /remote_cmd_vel."""
        frac = max(0, min(100, speed_pct)) / 100.0

        tw = Twist()
        if direction == 'FORWARD':
            tw.linear.x  = -frac * self._max_linear
        elif direction == 'BACKWARD':
            tw.linear.x  = frac * self._max_linear
        elif direction == 'LEFT':
            tw.angular.z = -frac * self._max_angular
        elif direction == 'RIGHT':
            tw.angular.z = frac * self._max_angular
        # STOP → all zeros

        self._remote_cmd_pub.publish(tw)
        self.get_logger().info(
            f'[云] 遥控 {direction} speed={speed_pct}% → '
            f'vx={tw.linear.x:.2f} vz={tw.angular.z:.2f}')

    # ── Direct cmd_vel (JSON with linear/angular keys) ─────────────────

    def _handle_direct_cmd_vel(self, data):
        tw = Twist()
        if 'linear' in data:
            lin = data['linear']
            tw.linear.x  = float(lin.get('x', 0.0))
            tw.linear.y  = float(lin.get('y', 0.0))
            tw.linear.z  = float(lin.get('z', 0.0))
        if 'angular' in data:
            ang = data['angular']
            tw.angular.x = float(ang.get('x', 0.0))
            tw.angular.y = float(ang.get('y', 0.0))
            tw.angular.z = float(ang.get('z', 0.0))
        self._remote_cmd_pub.publish(tw)
        self.get_logger().info(f'直接 cmd_vel: vx={tw.linear.x:.2f} vz={tw.angular.z:.2f}')

    # ── GPS fix from STM32 → MQTT ─────────────────────────────────────

    def _on_gps_fix(self, msg: NavSatFix):
        if not self._mqtt_connected:
            return
        payload = json.dumps({
            'longitude_car': msg.longitude,
            'latitude_car': msg.latitude,
        })
        self._mqtt.publish(self._gps_pub_topic, payload, qos=1)
        self.get_logger().info(
            f'MQTT → GPS: lon={msg.longitude:.6f} lat={msg.latitude:.6f}')

    # ── Outgoing pose publisher ────────────────────────────────────────

    def _publish_pose(self):
        if not self._mqtt_connected:
            self.get_logger().debug('MQTT not connected, skipping pose publish')
            return

        pose = self._get_robot_pose()
        if pose is None:
            self.get_logger().debug('TF not available, skipping pose publish')
            return

        x, y, yaw = pose
        payload = json.dumps({
            'x': round(x, 4),
            'y': round(y, 4),
            'yaw': round(math.degrees(yaw), 2),
            'mode': self._mode,
            'timestamp': time.time(),
        })
        self._mqtt.publish(self._pub_topic, payload, qos=1)
        self.get_logger().info(f'MQTT → pose: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) mode={self._mode}')

    def _get_robot_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except self._tf_ex:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def _mqtt_loop(self):
        try:
            self._mqtt.connect(
                self.get_parameter('broker').value,
                self.get_parameter('port').value,
                60)
            self._mqtt.loop_forever()
        except Exception as e:
            self.get_logger().error(f'MQTT thread error: {e}')


def main():
    rclpy.init()
    node = MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
