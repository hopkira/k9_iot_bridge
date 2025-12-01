
# 🕹️ Bangle2 → ROS2 Joystick Bridge

This ROS2 node lets you control a robot using a **Bangle.js 2 smartwatch** as a wireless BLE joystick. The joystick has two modes: fast and slow.  In the slow mode, movement is confined to forward, backwards and spin.  In the fast mode a combination of linear and angular movement is possible - and at higher speeds.
The full setup instructions are provided in [my blog](https://k9-build.blogspot.com/2025/11/using-banglejs-2-smartwatch-as-ble-ros2.html).
The node listens for BLE advertising packets (broadcast by the watch) via **EspruinoHub**, parses joystick values, and publishes a standard ROS `cmd_vel` message.

Ideal for remote teleop, walking robots, robot dogs, rovers, etc.  
(Or just making questionable life choices involving robots and watches 😄)

---

## 🚀 What it Does

1. The Bangle.js watch sends BLE advertisement packets containing joystick state:
   ```
   {m:42,x:12,y:-18,s:1}
   ```
   - `x` → forward/back joystick value  
   - `y` → steering  
   - `s` → speed mode (`0 = slow`, `1 = fast`)

2. **EspruinoHub** receives the BLE data and forwards it to MQTT topic:
   ```
   /ble/advertise/watch/espruino
   ```

3. This node subscribes to that topic, applies:
   - deadzone filtering  
   - scaling to real-world velocity  
   - optional smoothing (exponential velocity damping)

4. Finally publishes velocity commands to:

```
/cmd_vel  (geometry_msgs/Twist)
```

Which most ROS robots (including Nav2, diff-drive, custom nodes, etc.) understand.

---

## 🏁 Running the Node

Assuming you’ve already built the workspace:

```bash
source install/setup.bash
ros2 run k9_iot_bridge k9_iot_bridge_node
```

If everything is working, you’ll see something like:

```
[INFO] Listening to MQTT topic /ble/advertise/watch/espruino
[INFO] Publishing velocity to /cmd_vel
```

Move the joystick on the watch and you should see messages updating.

---

## ⚙️ Key Parameters

All parameters can be overridden at startup or changed live.

| Parameter | Description | Default |
|----------|-------------|---------|
| `mqtt_host` | MQTT broker hostname | `"localhost"` |
| `mqtt_port` | MQTT port | `1883` |
| `mqtt_joystick_topic` | Where joystick BLE data appears | `"/ble/advertise/watch/espruino"` |
| `joystick_max_raw` | Expected max raw joystick value | `98.0` |
| `deadzone_raw` | Range near center where motion = zero | `7.0` |
| `smoothing_alpha` | Exponential smoothing factor (0–1) | `0.25` |
| `max_linear_fast` | Max forward/back speed (m/s) | `1.4` |
| `max_linear_slow` | Slow-mode max speed (m/s) | `0.35` |
| `max_angular` | Max turning rate (rad/s) | `0.628` |

---

## 🎛️ Changing Parameters at Runtime

Most values can be tuned without restarting.

Example: increase responsiveness:

```bash
ros2 param set /espruino_mqtt_bridge smoothing_alpha 0.35
```

Reduce twitchiness by increasing the deadzone:

```bash
ros2 param set /espruino_mqtt_bridge deadzone_raw 10
```

Switch to slower max forward speed for indoor testing:

```bash
ros2 param set /espruino_mqtt_bridge max_linear_fast 0.9
```

If you want to see what’s currently set:

```bash
ros2 param list
ros2 param get /espruino_mqtt_bridge max_linear_fast
```

---

## 🤓 How the Smoothing Works

Movement is filtered using an exponential smoothing calculation:

```
next = current + alpha * (target - current)
```

- Higher `alpha` → snappier & more responsive  
- Lower `alpha` → smoother but delayed response  

**Important:**

If the joystick snaps back to centre (meaning finger lifted), the node **bypasses smoothing** and sends:

```
linear = 0
angular = 0
```

…so the robot stops immediately for safety.

---

## 🧪 Testing Without the Robot

You can echo the output:

```
ros2 topic echo /cmd_vel
```

Expected example:

```
linear:  {x: 0.42, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -0.21}
```

---

## 🎉 Summary

✔ Wireless joystick using a Bangle2 with fast and slow modes 
✔ No pairing, just BLE advertisements  
✔ Smooth velocity output with optional deadzone and speed modes  
✔ Fully tweakable at runtime  
✔ Publishes standard ROS `/cmd_vel`  

