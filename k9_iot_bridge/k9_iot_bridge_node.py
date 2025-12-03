#!/usr/bin/env python3
import asyncio
import json
import threading
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from bleak import BleakClient


def parse_joystick_payload(payload: str) -> Dict[str, Any]:
    """
    Try strict JSON first; if that fails, parse EspruinoHub-style
    object like {m:40,x:0,y:0,s:1} (unquoted keys).

    Kept from original implementation for maximum compatibility.
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

    result: Dict[str, Any] = {}
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


class BleBridge(Node):
    """
    Bridge:
      BLE device : Bangle.js 2 (or Puck.js) exposing a notify characteristic
                   sending JSON lines like:
                     {"m":40,"x":0,"y":0,"s":1}
      ROS topic  : /cmd_vel (geometry_msgs/Twist)

    Behaviour (NO smoothing, NO deadzone):
      - Raw joystick x/y are clamped to ±joystick_max_raw
      - Mapped linearly to:
          fast mode:  max_linear_fast (e.g. 1.4 m/s)
          slow mode:  max_linear_slow (e.g. 0.35 m/s)
          angular:    ±max_angular    (e.g. 0.628 rad/s)

    BLE:
      - Uses bleak in a background asyncio loop
      - Reconnects automatically if the connection drops
    """

    def __init__(self):
        super().__init__("bangle_ble_joystick_bridge")

        # ---------- Parameters ----------
        # Bangle BLE address (use your Bangle's MAC here or override via params)
        self.declare_parameter("ble_address", "E5:5D:2D:CE:6E:E7")

        # Service / characteristic UUIDs for joystick data.
        # Defaults to Nordic UART Service TX characteristic.
        self.declare_parameter(
            "ble_service_uuid",
            "6e400001-b5a3-f393-e0a9-e50e24dcca9e",
        )
        self.declare_parameter(
            "ble_char_uuid",
            "6e400003-b5a3-f393-e0a9-e50e24dcca9e",  # NUS TX (notify)
        )

        # Max speeds
        self.declare_parameter("max_linear_fast", 1.4)
        self.declare_parameter("max_linear_slow", 0.35)
        self.declare_parameter("max_angular", 0.628)

        # Joystick raw range (from Bangle mapping, ~ ±98)
        self.declare_parameter("joystick_max_raw", 98.0)

        # Reconnect delay (seconds) after a BLE error
        self.declare_parameter("reconnect_delay", 2.0)

        p = self.get_parameters([
            "ble_address",
            "ble_service_uuid",
            "ble_char_uuid",
            "max_linear_fast",
            "max_linear_slow",
            "max_angular",
            "joystick_max_raw",
            "reconnect_delay",
        ])

        self.ble_address = p[0].value
        self.ble_service_uuid = p[1].value
        self.ble_char_uuid = p[2].value
        self.max_linear_fast = float(p[3].value)
        self.max_linear_slow = float(p[4].value)
        self.max_angular = float(p[5].value)
        self.joystick_max_raw = float(p[6].value)
        self.reconnect_delay = float(p[7].value)

        # ---------- ROS publisher with "latest wins" QoS ----------
        cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, "/cmd_vel", cmd_vel_qos
        )
        self.get_logger().info(
            "Publishing joystick data from Bangle BLE to ROS topic '/cmd_vel' "
            "with QoS KEEP_LAST(depth=1), RELIABLE"
        )

        # ---------- BLE client in background thread ----------
        self._ble_thread = threading.Thread(
            target=self._ble_thread_fn,
            daemon=True,
        )
        self._ble_thread.start()

    # ---------- BLE thread & asyncio loop ----------

    def _ble_thread_fn(self):
        """
        Run the asyncio event loop for bleak in a background thread.
        """
        asyncio.run(self._ble_main())

    async def _ble_main(self):
        """
        Main BLE task: connect, subscribe, handle notifications, reconnect on error.
        """
        while rclpy.ok():
            try:
                self.get_logger().info(
                    f"Connecting to Bangle at {self.ble_address}..."
                )
                async with BleakClient(self.ble_address) as client:
                    self.get_logger().info("Connected to Bangle")

                    # Optionally, you could check services here, but usually not needed
                    # await client.get_services()

                    self.get_logger().info(
                        f"Subscribing to notify characteristic {self.ble_char_uuid}"
                    )
                    await client.start_notify(
                        self.ble_char_uuid,
                        self._notification_handler,
                    )

                    # Keep the connection alive until ROS shuts down or BLE disconnects
                    while rclpy.ok() and client.is_connected:
                        await asyncio.sleep(0.1)

                    self.get_logger().warning(
                        "Bangle disconnected, will attempt to reconnect..."
                    )

            except Exception as e:
                self.get_logger().error(f"BLE error: {e}")

            # Backoff before reconnect attempt
            if not rclpy.ok():
                break

            await asyncio.sleep(self.reconnect_delay)

        self.get_logger().info("BLE loop exiting (ROS is shutting down).")

    # ---------- BLE notification callback ----------

    def _notification_handler(self, handle: int, data: bytearray):
        """
        Called by bleak when a BLE notification is received.
        `data` is a bytearray with whatever the Bangle sent (JSON line).
        """
        try:
            payload = data.decode("utf-8", errors="ignore").strip()
        except Exception as e:
            self.get_logger().error(f"Failed to decode BLE payload: {e}")
            return

        if not payload:
            return

        # self.get_logger().debug(f"BLE payload: {payload}")
        self._handle_joystick_payload(payload)

    # ---------- Joystick payload handling (mostly copied from MQTT version) ----------

    def _handle_joystick_payload(self, payload: str):
        # Accept either proper JSON or Espruino-style {m:40,x:0,y:0,s:1}
        data = parse_joystick_payload(payload)
        if not data:
            self.get_logger().warn("Joystick payload was empty or unparseable")
            return

        # Raw joystick values (Bangle code sends these)
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

        # Mode-dependent scaling (NO deadzone, NO smoothing)
        if mode_flag == 1:
            max_linear = self.max_linear_fast   # e.g. 1.4 m/s
        else:
            max_linear = self.max_linear_slow   # e.g. 0.35 m/s

        max_angular = self.max_angular         # same for both modes

        # Normalise raw joystick to [-1, 1]
        norm_x = x_raw / max_raw
        norm_y = y_raw / max_raw

        # Direct mapping to velocities
        linear_out = norm_x * max_linear
        angular_out = norm_y * max_angular

        twist = Twist()
        twist.linear.x = linear_out
        twist.angular.z = angular_out

        self.cmd_vel_pub.publish(twist)

        # Debug logging (optional)
        # self.get_logger().debug(
        #     f"cmd_vel: mode={'FAST' if mode_flag==1 else 'SLOW'}, "
        #     f"raw=({x_raw:.1f},{y_raw:.1f}), "
        #     f"linear.x={twist.linear.x:.3f}, "
        #     f"angular.z={twist.angular.z:.3f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = BleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
