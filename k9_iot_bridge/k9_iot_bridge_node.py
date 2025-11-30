#!/usr/bin/env python3
import threading

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # swap for custom msgs later if needed


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

        self.declare_parameter("mqtt_base_topic", "espruino/devices")
        self.declare_parameter("device_ids", ["puck1"])

        p = self.get_parameters([
            "mqtt_host", "mqtt_port", "mqtt_username", "mqtt_password",
            "mqtt_use_tls", "mqtt_ca_cert", "mqtt_base_topic", "device_ids"
        ])

        self.mqtt_host = p[0].value
        self.mqtt_port = int(p[1].value)
        self.mqtt_username = p[2].value
        self.mqtt_password = p[3].value
        self.mqtt_use_tls = bool(p[4].value)
        self.mqtt_ca_cert = p[5].value
        self.base_topic = p[6].value
        self.device_ids = list(p[7].value)

        # --- ROS publishers (MQTT -> ROS) ---
        self.data_publishers = {}
        for dev_id in self.device_ids:
            ros_topic = f"/puck/{dev_id}/data"
            self.data_publishers[dev_id] = self.create_publisher(String, ros_topic, 10)
            self.get_logger().info(
                f"Publishing MQTT data for {dev_id} to ROS topic {ros_topic}"
            )

        # --- ROS subscribers (ROS -> MQTT) ---
        for dev_id in self.device_ids:
            ros_cmd_topic = f"/puck/{dev_id}/cmd"
            self.create_subscription(
                String,
                ros_cmd_topic,
                lambda msg, dev_id=dev_id: self._handle_ros_cmd(dev_id, msg),
                10,
            )
            self.get_logger().info(
                f"Subscribing to ROS commands for {dev_id} on {ros_cmd_topic}"
            )

        # --- MQTT client setup (for Mosquitto) ---
        self.mqtt_client = mqtt.Client()

        if self.mqtt_username:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)

        if self.mqtt_use_tls:
            # if mqtt_ca_cert is empty, it will use system CAs
            if self.mqtt_ca_cert:
                self.mqtt_client.tls_set(ca_certs=self.mqtt_ca_cert)
            else:
                self.mqtt_client.tls_set()  # default system CA

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        # Start MQTT loop in a separate thread
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
                self._sleep(5.0)

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
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="ignore")
        self.get_logger().debug(f"MQTT message on {topic}: {payload}")

        # Expected format: espruino/devices/<id>/data
        parts = topic.split("/")
        if len(parts) >= 3 and parts[0] == "espruino" and parts[1] == "devices":
            dev_id = parts[2]
            pub = self.data_publishers.get(dev_id)
            if pub is not None:
                ros_msg = String()
                ros_msg.data = payload
                pub.publish(ros_msg)

    # ---------- ROS -> MQTT ----------

    def _handle_ros_cmd(self, dev_id: str, msg: String):
        topic = f"{self.base_topic}/{dev_id}/cmd"
        payload = msg.data
        try:
            self.mqtt_client.publish(topic, payload)
            self.get_logger().info(f"Published command to {topic}: {payload}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish MQTT command: {e}")


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
