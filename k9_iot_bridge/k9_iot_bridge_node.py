#!/usr/bin/env python3
import asyncio
import json
import threading
import time
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


class BangleBleJoystickBridge(Node):
    """
    Bridge:
      BLE device : Bangle.js 2 exposing a notify characteristic
                   sending JSON lines like:
                     {"m":40,"x":0,"y":0,"s":1}
      ROS topic  : /cmd_vel (geometry_msgs/Twist)

    BLE:
      - Uses bleak in a background asyncio loop
      - Reconnects automatically if the connection drops
      - Watchdog: if no messages for IDLE_TIMEOUT seconds, force reconnect
    """

    def __init__(self):
        super().__init__("bangle_ble_joystick_bridge")

        # ---------- Parameters ----------
        self.declare_parameter("ble_address", "E5:5D:2D:CE:6E:E7")
        self.declare_parameter(
            "ble_service_uuid", "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
        )
        self.declare_parameter(
            "ble_char_uuid", "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # NUS TX (notify)
        )

        # Max speeds
        self.declare_parameter("max_linear_fast", 1.4)
        self.declare_parameter("max_linear_slow", 0.35)
        self.declare_parameter("max_angular", 0.628)

        # Joystick raw range (from Bangle mapping, ~ ±98)
        self.declare_parameter("joystick_max_raw", 98.0)

        # Reconnect delay (seconds) after a BLE error / disconnect
        self.declare_parameter("reconnect_delay", 1.0)

        params = self.get_parameters([
            "ble_address",
            "ble_service_uuid",
            "ble_char_uuid",
            "max_linear_fast",
            "max_linear_slow",
            "max_angular",
            "joystick_max_raw",
            "reconnect_delay",
        ])

        self.ble_address = params[0].value
        self.ble_service_uuid = params[1].value
        self.ble_char_uuid = params[2].value
        self.max_linear_fast = float(params[3].value)
        self.max_linear_slow = float(params[4].value)
        self.max_angular = float(params[5].value)
        self.joystick_max_raw = float(params[6].value)
        self.reconnect_delay = float(params[7].value)

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

        # Buffer for assembling complete JSON lines from BLE
        self._rx_buffer = ""
        # Time of last valid BLE line seen (for watchdog)
        self._last_msg_time = 0.0

        # ---------- BLE client in background thread ----------
        self._ble_thread = threading.Thread(
            target=self._ble_thread_fn,
            daemon=True,
        )
        self._ble_thread.start()

    # ---------- BLE thread & asyncio loop ----------

    def _ble_thread_fn(self):
        asyncio.run(self._ble_main())

    async def _safe_disconnect(self, client: BleakClient):
        """
        Try to disconnect, swallowing the common BlueZ 'UnknownObject' error
        that happens when the device DBus object is already gone.
        """
        try:
            if client.is_connected:
                self.get_logger().info("[BLE] Disconnecting client cleanly...")
                await client.disconnect()
            else:
                # Even if not connected, disconnect() may still touch DBus
                try:
                    await client.disconnect()
                except Exception:
                    pass
        except Exception as e:
            if "UnknownObject" in str(e):
                self.get_logger().warn(
                    "[BLE] Device already gone (DBus UnknownObject). Continuing..."
                )
            else:
                self.get_logger().error(
                    f"[BLE] Error during disconnect: {e}"
                )

    async def _ble_main(self):
        """
        Main BLE task: connect, subscribe, handle notifications, reconnect on error.
        Includes a watchdog: if no messages for IDLE_TIMEOUT seconds, force reconnect.
        """
        IDLE_TIMEOUT = 2.0  # seconds with no joystick messages before we assume it's dead

        while rclpy.ok():
            client = BleakClient(self.ble_address)

            try:
                self.get_logger().info(
                    f"[BLE] Trying to connect to {self.ble_address}..."
                )
                await client.connect()
                if not client.is_connected:
                    raise RuntimeError(
                        "connect() returned but client.is_connected is False"
                    )

                self.get_logger().info("[BLE] Connected to Bangle")

                # Reset watchdog when we connect
                self._last_msg_time = time.time()

                self.get_logger().info(
                    f"[BLE] start_notify on {self.ble_char_uuid}"
                )
                await client.start_notify(
                    self.ble_char_uuid,
                    self._notification_handler,
                )

                # Wait here until ROS is shutting down or connection is considered dead
                while rclpy.ok():
                    if not client.is_connected:
                        self.get_logger().warning(
                            "[BLE] client.is_connected == False"
                        )
                        break

                    now = time.time()
                    if (
                        self._last_msg_time > 0
                        and (now - self._last_msg_time) > IDLE_TIMEOUT
                    ):
                        self.get_logger().warning(
                            f"[BLE] No joystick messages for {now - self._last_msg_time:.1f}s "
                            f"(>{IDLE_TIMEOUT}s), forcing reconnect"
                        )
                        await self._safe_disconnect(client)
                        break

                    await asyncio.sleep(0.5)

                if not rclpy.ok():
                    self.get_logger().info(
                        "[BLE] ROS shutting down, leaving loop"
                    )
                else:
                    self.get_logger().info(
                        "[BLE] Connection ended, will attempt reconnect"
                    )

            except Exception as e:
                self.get_logger().error(
                    f"[BLE] Exception in _ble_main: {e}"
                )

            finally:
                await self._safe_disconnect(client)

            if not rclpy.ok():
                break

            self.get_logger().info(
                f"[BLE] Sleeping {self.reconnect_delay} seconds before retry"
            )
            await asyncio.sleep(self.reconnect_delay)

        self.get_logger().info("[BLE] BLE loop exiting")

    # ---------- BLE notification callback ----------
    def _notification_handler(self, handle: int, data: bytearray):
        """
        Called by bleak when a BLE notification is received.
        We may get partial lines or multiple lines per notification.

        To minimise lag, we implement 'latest-wins' semantics:
        if multiple complete lines arrive at once, we only process the last one.
        """
        try:
            chunk = data.decode("utf-8", errors="ignore")
        except Exception as e:
            self.get_logger().error(f"Failed to decode BLE payload: {e}")
            return

        if not chunk:
            return

        self._rx_buffer += chunk

        # If there's no newline at all yet, we can't form a complete line.
        if "\n" not in self._rx_buffer:
            return

        # Split into lines; last element may be incomplete, so keep it in buffer.
        parts = self._rx_buffer.split("\n")
        # Everything except the last element are complete lines
        complete_lines = parts[:-1]
        # Remainder (possibly empty or partial line) stays in buffer
        self._rx_buffer = parts[-1]

        if not complete_lines:
            return

        # Take only the most recent complete line to avoid replaying stale commands
        latest_line = complete_lines[-1].strip()
        if not latest_line:
            return

        # We saw *some* line over BLE – update watchdog
        self._last_msg_time = time.time()

        # Optional debug:
        # self.get_logger().info(f"BLE latest line: {repr(latest_line)}")

        self._handle_joystick_payload(latest_line)

    # ---------- Joystick payload handling ----------

    def _handle_joystick_payload(self, payload: str):
        # Quick filter: only bother with things that look vaguely like joystick JSON
        if "x" not in payload or "y" not in payload:
            return

        # Strip common REPL junk (e.g. leading '>')
        if payload.startswith(">"):
            payload = payload.lstrip(">").strip()

        data = parse_joystick_payload(payload)
        if not data:
            self.get_logger().warn(
                f"Joystick payload was empty or unparseable: {repr(payload)}"
            )
            return

        # Raw joystick values (Bangle code sends these)
        x_raw = float(data.get("x", 0.0))  # forward/back
        y_raw = float(data.get("y", 0.0))  # steer
        mode_flag = int(data.get("s", 0))  # 0=slow, 1=fast

        # Clamp raw values to expected range
        max_raw = self.joystick_max_raw
        x_raw = max(-max_raw, min(max_raw, x_raw))
        y_raw = max(-max_raw, min(max_raw, y_raw))

        # Mode-dependent scaling (NO deadzone, NO smoothing)
        if mode_flag == 1:
            max_linear = self.max_linear_fast   # e.g. 1.4 m/s
        else:
            max_linear = self.max_linear_slow   # e.g. 0.35 m/s)

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

        # Optional debug:
        # self.get_logger().debug(
        #     f"cmd_vel: mode={'FAST' if mode_flag==1 else 'SLOW'}, "
        #     f"raw=({x_raw:.1f},{y_raw:.1f}), "
        #     f"linear.x={twist.linear.x:.3f}, "
        #     f"angular.z={twist.angular.z:.3f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = BangleBleJoystickBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
