#!/usr/bin/env python3
import json
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def parse_joystick_payload(payload: str):
    """
    Try strict JSON first; if that fails, parse EspruinoHub-style
    object like {m:40,x:0,y:0,s:1} (unquoted keys).
    """
    payload = payload.strip()

    # First try normal JSON
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        pass

    # Fallback: very simple parser for {m:40,x:0,y:0,s:1}
    if payload.startswith("{") and payload.endswith("}"):
        payload = payload[1:-1]

    result = {}
    if not payload:
        return result

    for part in payload.split(","):
        if ":" not in part:
            continue
        k, v = part.split(":", 1)
        key = k.strip().strip('"').strip("'")
        val = v.strip()

        # Try bool
        if val.lower() in ("true", "false"):
            result[key] = (val.lower() == "true")
            continue

        # Try number
        try:
            num = float(val)
            result[key] = int(num) if num.is_integer() else num
            continue
        except ValueError:
            pass

        # Fall back to string
        result[key] = val.strip('"').strip("'")

    return result


class IotMqttBridge(Node):
    """
    Bridge:
      MQTT topic  : /ble/advertise/watch/espruino
      MQTT payload: {m:40,x:0,y:0,s:1} or {"m":40,"x":0,"y":0,"s":1}
        x = forward/back joystick value  (~ -98..+98)
        y = steer joystick value         (~ -98..+98)
        s = speed mode (0=slow, 1=fast)

      ROS topic   : /cmd_vel (geometry_msgs/Twist)

    Features:
      - Fast/slow mode scaling
      - Deadzone around center
      - Exponential smoothing with instant hard stop on release
    """

    def __init__(self):
        super().__init__("espruino_mqtt_bridge")

        # ---------- Parameters ----------
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password", "")
        self.declare_parameter("mqtt_use_tls", False)
        self.declare_parameter("mqtt_ca_cert", "")

        # EspruinoHub joystick topic
        self.declare_parameter(
            "mqtt_joystick_topic", "/ble/advertise/watch/espruino"
        )

        # Max speeds
        # Fast mode: 1.4 m/s, Slow mode: 0.35 m/s, Angular: ±0.628 rad/s
        self.declare_parameter("max_linear_fast", 1.4)
        self.declare_parameter("max_linear_slow", 0.35)
        self.declare_parameter("max_angular", 0.628)

        # Joystick raw range (from Bangle mapping, ~ ±98)
        self.declare_parameter("joystick_max_raw", 98.0)

        # Deadzone in raw joystick units (to avoid drift)
        # e.g. 5 means |x_raw|<5 treated as 0
        self.declare_parameter("deadzone_raw", 7.0)

        # Exponential smoothing factor (0..1)
        # 0.2 = fairly responsive but smooth
        self.declare_parameter("smoothing_alpha", 0.25)

        p = self.get_parameters([
            "mqtt_host", "mqtt_port", "mqtt_username", "mqtt_password",
            "mqtt_use_tls", "mqtt_ca_cert", "mqtt_joystick_topic",
            "max_linear_fast", "max_linear_slow", "max_angular",
            "joystick_max_raw", "deadzone_raw", "smoothing_alpha",
        ])

        self.mqtt_host = p[0].value
        self.mqtt_port = int(p[1].value)
        self.mqtt_username = p[2].value
        self.mqtt_password = p[3].value
        self.mqtt_use_tls = bool(p[4].value)
        self.mqtt_ca_cert = p[5].value
        self.joystick_topic = p[6].value
        self.max_linear_fast = float(p[7].value)
        self.max_linear_slow = float(p[8].value)
        self.max_angular = float(p[9].value)
        self.joystick_max_raw = float(p[10].value)
        self.deadzone_raw = float(p[11].value)
        self.alpha = float(p[12].value)

        # ---------- ROS publisher ----------
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(
            f"Publishing joystick data from MQTT topic "
            f"'{self.joystick_topic}' to ROS topic '/cmd_vel'"
        )

        # ---------- State for smoothing ----------
        self.last_linear = 0.0
        self.last_angular = 0.0

        # ---------- MQTT client ----------
        self.mqtt_client = mqtt.Client()

        if self.mqtt_username:
            self.mqtt_client.username_pw_set(
                self.mqtt_username, self.mqtt_password
            )

        if self.mqtt_use_tls:
            if self.mqtt_ca_cert:
                self.mqtt_client.tls_set(ca_certs=self.mqtt_ca_cert)
            else:
                self.mqtt_client.tls_set()

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        # Run MQTT loop in background thread
        self.mqtt_thread = threading.Thread(
            target=self._mqtt_thread_fn, daemon=True
        )
        self.mqtt_thread.start()

    # ---------- MQTT thread & callbacks ----------

    def _mqtt_thread_fn(self):
        while rclpy.ok():
            try:
                self.get_logger().info(
                    f"Connecting to MQTT broker at {self.mqtt_host}:{self.mqtt_port}"
                )
                self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
                self.mqtt_client.loop_forever()
            except Exception as e:
                self.get_logger().error(f"MQTT connection error: {e}")
                self.get_logger().info("Reconnecting to MQTT in 5 seconds...")
                for _ in range(50):
                    if not rclpy.ok():
                        return
                    time.sleep(0.1)

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker.")
            client.subscribe(self.joystick_topic)
            self.get_logger().info(
                f"Subscribed to joystick topic: {self.joystick_topic}"
            )
        else:
            self.get_logger().error(f"Failed to connect to MQTT, rc={rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="ignore")
        self.get_logger().debug(f"MQTT message on {msg.topic}: {payload}")

        # Accept either proper JSON or Espruino-style {m:40,x:0,y:0,s:1}
        data = parse_joystick_payload(payload)
        if not data:
            self.get_logger().warn("Joystick payload was empty or unparseable")
            return

        # Raw joystick values (Espruino code sends these)
        x_raw = float(data.get("x", 0.0))  # forward/back
        y_raw = float(data.get("y", 0.0))  # steer
        mode_flag = int(data.get("s", 0))  # 0=slow, 1=fast

        # Clamp raw values to expected range
        max_raw = self.joystick_max_raw
        if x_raw > max_raw:
            x_raw = max_raw
        if x_raw < -max_raw:
            x_raw = -max_raw
        if y_raw > max_raw:
            y_raw = max_raw
        if y_raw < -max_raw:
            y_raw = -max_raw

        # ----- Deadzone and hard-stop behaviour -----
        # If joystick near centre, treat as "finger off" → instant stop (no smoothing)
        if abs(x_raw) <= self.deadzone_raw and abs(y_raw) <= self.deadzone_raw:
            linear_out = 0.0
            angular_out = 0.0
            self.last_linear = 0.0
            self.last_angular = 0.0
        else:
            # ----- Mode-dependent scaling -----
            # Choose linear max speed based on mode
            if mode_flag == 1:
                max_linear = self.max_linear_fast   # 1.4 m/s
            else:
                max_linear = self.max_linear_slow   # 0.35 m/s

            # Angular max is same for both modes
            max_angular = self.max_angular         # 0.628 rad/s

            # Normalise raw joystick to [-1, 1]
            norm_x = x_raw / max_raw
            norm_y = y_raw / max_raw

            # Target velocities before smoothing
            linear_target = norm_x * max_linear
            angular_target = norm_y * max_angular

            # ----- Exponential smoothing -----
            a = self.alpha
            linear_out = self.last_linear + a * (linear_target - self.last_linear)
            angular_out = self.last_angular + a * (angular_target - self.last_angular)

            self.last_linear = linear_out
            self.last_angular = angular_out

        twist = Twist()
        twist.linear.x = linear_out
        twist.angular.z = angular_out

        self.cmd_vel_pub.publish(twist)

        self.get_logger().debug(
            f"cmd_vel: mode={'FAST' if mode_flag==1 else 'SLOW'}, "
            f"raw=({x_raw:.1f},{y_raw:.1f}), "
            f"linear.x={twist.linear.x:.3f}, "
            f"angular.z={twist.angular.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IotMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
