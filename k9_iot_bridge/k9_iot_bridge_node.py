#!/usr/bin/env python3
import threading
import time
import json
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # swap for custom msgs later if needed
from geometry_msgs.msg import Twist


class IotMqttBridge(Node):
    def __init__(self):
        super().__init__("espruino_mqtt_bridge")

        # --- Parameters for Mosquitto/MQTT ---
        self.declare_parameter("mqtt_host", "localhost")      # Mosquitto host
        self.declare_parameter("mqtt_port", 1883)             # Mosquitto port
        self.declare_parameter("mqtt_username", "")           # optional
        self.declare_parameter("mqtt_password", "")           # optional
        self.declare_parameter("mqtt_use_tls", False)         # set True if using TLS
        self.declare_parameter("mqtt_ca_cert", "")            # path to CA cert if TLS

        # EspruinoHub topic for the Bangle joystick:
        # e.g. /ble/advertise/watch/espruino -> {"m":40,"x":0,"y":0,"s":1}
        self.declare_parameter("mqtt_joystick_topic",
                               "/ble/advertise/watch/espruino")

        # Scale factors to map joystick units to velocities
        self.declare_parameter("linear_scale", 0.01)   # m/s per unit of x
        self.declare_parameter("angular_scale", 0.01)  # rad/s per unit of y

        p = self.get_parameters([
            "mqtt_host", "mqtt_port", "mqtt_username", "mqtt_password",
            "mqtt_use_tls", "mqtt_ca_cert", "mqtt_joystick_topic",
            "linear_scale", "angular_scale",
        ])

        self.mqtt_host = p[0].value
        self.mqtt_port = int(p[1].value)
        self.mqtt_username = p[2].value
        self.mqtt_password = p[3].value
        self.mqtt_use_tls = bool(p[4].value)
        self.mqtt_ca_cert = p[5].value
        self.joystick_topic = p[6].value
        self.linear_scale = float(p[7].value)
        self.angular_scale = float(p[8].value)

        # ---------- ROS publisher ----------
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(
            f"Publishing joystick data from MQTT topic "
            f"'{self.joystick_topic}' to ROS topic '/cmd_vel'"
        )

        # --- MQTT client setup (for Mosquitto) ---
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
                    f"Connecting to Mosquitto at {self.mqtt_host}:{self.mqtt_port}"
                )
                self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
                self.mqtt_client.loop_forever()
            except Exception as e:
                self.get_logger().error(f"MQTT connection error: {e}")
                self.get_logger().info("Reconnecting to Mosquitto in 5 seconds...")
                for _ in range(50):
                    if not rclpy.ok():
                        return
                    time.sleep(0.1)

    def _sleep(self, seconds: float):
        # Simple ROS-friendly sleep
        rate = self.create_rate(1.0)
        for _ in range(int(seconds)):
            if not rclpy.ok():
                break
            rate.sleep()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to Mosquitto broker.")
            for dev_id in self.device_ids:
                topic = f"{self.base_topic}/{dev_id}/data"
                client.subscribe(topic)
                self.get_logger().info(f"Subscribed to MQTT topic {topic}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT, rc={rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="ignore")
        self.get_logger().debug(f"MQTT message on {msg.topic}: {payload}")

        # Expecting JSON from EspruinoHub: {"m":40,"x":0,"y":0,"s":1}
        try:
            data = json.loads(payload)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse joystick JSON: {e}")
            return

        # x = forward/back, y = left/right steer
        x_raw = float(data.get("x", 0.0))
        y_raw = float(data.get("y", 0.0))
        # s = mode (0/1) is available as data.get("s") if you want it

        twist = Twist()
        twist.linear.x = x_raw * self.linear_scale
        twist.angular.z = y_raw * self.angular_scale

        self.cmd_vel_pub.publish(twist)

        self.get_logger().debug(
            f"cmd_vel: linear.x={twist.linear.x:.3f}, "
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
