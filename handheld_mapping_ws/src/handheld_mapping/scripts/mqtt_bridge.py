#!/usr/bin/python3
"""
Bidirectional MQTT bridge for Alibaba Cloud IoT Hub.

Publishes robot pose (TF map→base_footprint) to MQTT at a configurable
interval, and handles cloud commands:

  Mode commands:  GPS (1) / REMOTE (2) / INDOOR (3) → /robot_mode
  Move commands:  forward/backward/left/right/stop + speed → /remote_cmd_vel
  Other:          EMERGENCY → stop, TAKE_PHOTO → log

Usage:
  ros2 run handheld_mapping mqtt_bridge
  ros2 run handheld_mapping mqtt_bridge --ros-args -p client_id:="..." -p username:="..."
"""

import json
import math
import os
import time
import threading

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8, String

import paho.mqtt.client as mqtt


# Mode constants (sync with STM32)
MODE_GPS    = 1
MODE_REMOTE = 2
MODE_INDOOR = 3


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        package_share = get_package_share_directory('handheld_mapping')
        bundled_ca = os.path.join(package_share, 'certs', 'emqxsl-ca.crt')

        # ── MQTT connection params ──────────────────────────────────
        # EMQX Serverless defaults for zero-config startup. Every value can
        # still be overridden through an EMQX_* environment variable.
        self.declare_parameter(
            'broker', os.environ.get('EMQX_BROKER', 'i6130f30.ala.cn-hangzhou.emqxsl.cn'))
        self.declare_parameter('port', int(os.environ.get('EMQX_PORT', '8883')))
        self.declare_parameter(
            'client_id', os.environ.get('EMQX_CLIENT_ID', 'robot_001_jetson'))
        self.declare_parameter(
            'username', os.environ.get('EMQX_USERNAME', 'robot_001'))
        self.declare_parameter('password', os.environ.get('EMQX_PASSWORD', '123456'))
        self.declare_parameter(
            'ca_certs', os.environ.get('EMQX_CA_CERTS', bundled_ca))

        # ── Topics ──────────────────────────────────────────────────
        self.declare_parameter('sub_topic', '/k1ck5t83zdZ/test/user/get')
        self.declare_parameter('pub_topic', '/k1ck5t83zdZ/test/user/robot')
        self.declare_parameter('gps_pub_topic', '/k1ck5t83zdZ/test/user/esp8266duan')
        self.declare_parameter('map_pub_topic', '/k1ck5t83zdZ/test/user/map')
        self.declare_parameter('path_pub_topic', '/k1ck5t83zdZ/test/user/path')
        self.declare_parameter('mission_pub_topic', '/k1ck5t83zdZ/test/user/mission')

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
        _ca_certs = self.get_parameter('ca_certs').value

        self._sub_topic     = self.get_parameter('sub_topic').value
        self._pub_topic     = self.get_parameter('pub_topic').value
        self._gps_pub_topic = self.get_parameter('gps_pub_topic').value
        self._map_pub_topic = self.get_parameter('map_pub_topic').value
        self._path_pub_topic = self.get_parameter('path_pub_topic').value
        self._mission_pub_topic = self.get_parameter('mission_pub_topic').value
        self._map_frame     = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._interval    = self.get_parameter('publish_interval').value
        self._max_linear  = self.get_parameter('max_linear').value
        self._max_angular = self.get_parameter('max_angular').value

        # ── State ───────────────────────────────────────────────────
        self._mode = MODE_GPS       # current robot mode
        self._speed_pct = 50        # speed percentage (0-100), default 50
        self._last_gps_publish = 0.0

        # ── TF buffer ───────────────────────────────────────────────
        from tf2_ros import Buffer, TransformListener
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_ex = (LookupException, ConnectivityException, ExtrapolationException)

        # ── ROS2 publishers ─────────────────────────────────────────
        self._mode_pub = self.create_publisher(Int8, '/robot_mode', 10)
        self._remote_cmd_pub = self.create_publisher(Twist, '/remote_cmd_vel', 10)
        self._indoor_command_pub = self.create_publisher(String, '/indoor/mission_command', 10)

        # ── ROS2 subscriptions ──────────────────────────────────────
        self._gps_sub = self.create_subscription(
            NavSatFix, '/gps_fix', self._on_gps_fix, 10)
        self.create_subscription(String, '/indoor/map', self._on_indoor_map, 1)
        self.create_subscription(String, '/indoor/path', self._on_indoor_path, 5)
        self.create_subscription(String, '/indoor/mission_state', self._on_indoor_mission_state, 10)

        # Publish initial mode
        self._mode_pub.publish(Int8(data=self._mode))

        # ── MQTT client ─────────────────────────────────────────────
        self._mqtt = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=_cid)
        self._mqtt.username_pw_set(_user, _pw)
        try:
            self._mqtt.tls_set(ca_certs=_ca_certs)
        except Exception as exc:
            self.get_logger().fatal(f'Cannot configure MQTT TLS CA { _ca_certs }: {exc}')
            raise
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

        if cmd in ('GPS', 'REMOTE', 'INDOOR', 'LINE'):
            self._handle_mode_switch(cmd)

        elif cmd == 'EMERGENCY':
            self._indoor_command_pub.publish(String(data=json.dumps({
                'command': 'INDOOR_MISSION_CANCEL',
                'reason': 'EMERGENCY',
                'request_id': data.get('request_id', ''),
            }, separators=(',', ':'))))
            # REMOTE + zero velocity prevents STM32 from continuing to consume
            # Nav2 cmd_vel while the asynchronous Nav2 cancellation completes.
            self._mode = MODE_REMOTE
            self._mode_pub.publish(Int8(data=self._mode))
            self.get_logger().info(f'[云] 急停 → cmd_vel=0')
            self._remote_cmd_pub.publish(Twist())

        elif cmd == 'TAKE_PHOTO':
            self.get_logger().info(f'[云] 拍照指令')

        elif cmd == 'SPEED':
            self._speed_pct = int(spd)
            self.get_logger().info(f'[云] 速度设为 {self._speed_pct}%')

        elif cmd in ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'STOP'):
            self._handle_move(cmd, int(spd))

        elif cmd in ('INDOOR_MISSION_START', 'INDOOR_MISSION_CANCEL', 'INDOOR_RECORD_POINT'):
            if cmd == 'INDOOR_MISSION_START' and self._mode != MODE_INDOOR:
                self._mode = MODE_INDOOR
                self._mode_pub.publish(Int8(data=self._mode))
            self._indoor_command_pub.publish(
                String(data=json.dumps(data, separators=(',', ':'))))
            self.get_logger().info(f'[云] 室内导航命令 → {cmd}')

        else:
            self.get_logger().info(f'[云] 未知指令: {cmd}, params={data}')

    # ── Mode switch ────────────────────────────────────────────────────

    def _handle_mode_switch(self, cmd: str):
        mode_map = {
            'GPS': MODE_GPS,
            'REMOTE': MODE_REMOTE,
            'INDOOR': MODE_INDOOR,
            'LINE': MODE_INDOOR,  # Backward-compatible alias.
        }
        new_mode = mode_map[cmd]
        if new_mode != self._mode:
            self._remote_cmd_pub.publish(Twist())
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
        now = time.monotonic()
        if now - self._last_gps_publish < 1.0:
            return
        self._last_gps_publish = now
        payload = json.dumps({
            'longitude_car': msg.longitude,
            'latitude_car': msg.latitude,
        })
        self._mqtt.publish(self._gps_pub_topic, payload, qos=1)
        self.get_logger().info(
            f'MQTT → GPS: lon={msg.longitude:.6f} lat={msg.latitude:.6f}')

    def _publish_ros_json(self, topic, msg, retain=False):
        if not self._mqtt_connected:
            return
        self._mqtt.publish(topic, msg.data, qos=1, retain=retain)

    def _on_indoor_map(self, msg: String):
        self._publish_ros_json(self._map_pub_topic, msg, retain=True)

    def _on_indoor_path(self, msg: String):
        self._publish_ros_json(self._path_pub_topic, msg)

    def _on_indoor_mission_state(self, msg: String):
        self._publish_ros_json(self._mission_pub_topic, msg, retain=True)

    # ── Outgoing pose publisher ────────────────────────────────────────

    def _publish_pose(self):
        if not self._mqtt_connected:
            self.get_logger().debug('MQTT not connected, skipping pose publish')
            return

        pose = self._get_robot_pose()
        if pose is None:
            payload = json.dumps({
                'robot_id': 'robot_001',
                'online': True,
                'mode': self._mode,
                'timestamp': time.time(),
            })
            self._mqtt.publish(self._pub_topic, payload, qos=1)
            self.get_logger().debug(
                f'MQTT heartbeat: TF unavailable, mode={self._mode}')
            return

        x, y, yaw = pose
        payload = json.dumps({
            'robot_id': 'robot_001',
            'online': True,
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
