# Running Zephyr with Gimbal & Live Object Distance Monitor

## Environment Setup
Run this in any terminal before launching Gazebo:
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

---

## 1. Launch Gazebo Sim (Runway with Zephyr & Gimbal)
```bash
gz sim -v4 -r zephyr_gimbal_runway.sdf
```
*Note: Shaking/jumping has been completely resolved by removing gimbal ground collision constraints, rigid-mounting the gimbal, and adding lightweight aerodynamic landing gear skids.*

---

## 2. Launch ArduPlane SITL
In a new terminal:
```bash
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --add-param-file=$HOME/ardupilot_gazebo/config/gazebo-zephyr-gimbal.parm --console --map
```

### Smooth Runway Takeoff in FBWA Mode:
1. In the MAVProxy console:
   ```text
   mode fbwa
   arm throttle
   rc 3 1800
   ```
2. The aircraft accelerates cleanly down the runway (0 to 15 m/s in ~3 seconds).
3. As it reaches 12–14 m/s (takes only 30–40 meters), give a gentle pitch up on the elevator stick (or in MAVProxy console `rc 2 1650`).
4. The aircraft will lift off cleanly into the sky!

---

## 3. Launch Live Object Distance Monitor (Terminal & Web HUD)
In a new terminal:
```bash
python3 /home/shubh/ardupilot_gazebo/tools/live_distance_monitor.py
```
- **Terminal UI**: Shows a color-coded live-updating table of **EVERY** landmark, tower, building, and moving vehicle (`car_red_250m`, `r1_rover`) sorted by 3D distance with slant range, ground distance, relative altitude (&Delta;Z), bearing, and clock direction.
- **Web Browser HUD**: Open `http://localhost:8088` in any browser on your machine or local network for an interactive live dashboard.

---

## 4. Enable Gimbal Camera Video Streaming
While Gazebo is running:
```bash
gz topic -t /gimbal/camera/enable_streaming -m gz.msgs.Boolean -p "data: 1"
```

### View Live Video Stream
In a separate terminal:
```bash
ffplay -flags low_delay -fflags nobuffer udp://127.0.0.1:5600
```
Or in **QGroundControl**:
- Go to `Settings -> Video -> Source: UDP h.264 Video Stream -> Port: 5600`.

---

## 5. Control the Gimbal
In MAVProxy or via RC channels:
- **Pitch**: RC 7 (1100 = 90&deg; down, 1500 = forward, 1900 = 90&deg; up)
- **Yaw**: RC 8 (1100 = -160&deg; left, 1500 = center forward, 1900 = +160&deg; right)
- **Roll**: RC 6 (1100 = -30&deg;, 1500 = level, 1900 = +30&deg;)