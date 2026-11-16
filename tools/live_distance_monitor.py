#!/usr/bin/env python3
"""
Live Object Distance Monitor for Zephyr with Gimbal in Gazebo Sim
-----------------------------------------------------------------
Features:
1. LIVE 3D DISTANCE LABELS PLACED DIRECTLY OVER EVERY OBJECT IN GAZEBO:
   - Projects 3D floating text tags + glowing beacons hovering right above each target in Gazebo!
   - Dynamic targets (Red Patrol Car, R1 Rover) track their 3D distance label in real time as they drive!
   - No need to switch to terminal or browser while flying—distances are visible right over the objects!
2. 100% STATIC, PERMANENT LIST (NO JUMPING ROWS):
   - Every object maintains its exact permanent row/line organized by runway corridor!
   - No confusing reshuffling as distances change.
   - Live closest target is highlighted with a distinct indicator without moving its row!
3. Real-time Web Radar & Distance HUD on http://localhost:8088.
"""

import sys
import os
import time
import math
import subprocess
import threading
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
import socketserver

# Permanent Static Object Catalog & World Positions
OBJECT_CONFIG = {
    # --- RUNWAY CORRIDOR ---
    "origin_target_pad_0_0": {
        "label": "Origin Bullseye Pad (0, 0)",
        "short": "Origin Pad",
        "group": "RUNWAY CORRIDOR",
        "x": 0.0, "y": 0.0, "z": 0.05,
        "marker_z": 7.0,
        "type": "target",
        "color": {"r": 0.0, "g": 1.0, "b": 1.0}
    },
    "runway_end_north": {
        "label": "Runway North End (1000m)",
        "short": "Runway End",
        "group": "RUNWAY CORRIDOR",
        "x": 0.0, "y": 1000.0, "z": 0.0,
        "marker_z": 8.0,
        "type": "runway",
        "color": {"r": 0.0, "g": 0.8, "b": 1.0}
    },

    # --- ACTIVE MOVING TARGETS ---
    "car_red_250m": {
        "label": "Red Patrol Car (Oval Circuit)",
        "short": "Patrol Car",
        "group": "MOVING TARGETS",
        "x": -80.0, "y": 800.0, "z": 2.5,
        "marker_z": 9.0,
        "type": "moving_vehicle",
        "color": {"r": 1.0, "g": 0.2, "b": 0.2}
    },
    "r1_rover": {
        "label": "R1 Target Rover (Oval Circuit)",
        "short": "R1 Rover",
        "group": "MOVING TARGETS",
        "x": 30.0, "y": 200.0, "z": 0.38,
        "marker_z": 8.0,
        "type": "moving_vehicle",
        "color": {"r": 1.0, "g": 0.5, "b": 0.0}
    },

    # --- WEST FLANK CORRIDOR (Left Side, Y=50m to Y=960m) ---
    "ground_station_50m": {
        "label": "Ground Control Station (50m)",
        "short": "Ground Station",
        "group": "WEST FLANK",
        "x": -110.0, "y": 50.0, "z": 7.5,
        "marker_z": 22.0,
        "type": "facility",
        "color": {"r": 1.0, "g": 0.8, "b": 0.2}
    },
    "helipad_target_150m": {
        "label": "Helipad Target Pad (180m)",
        "short": "Helipad Pad",
        "group": "WEST FLANK",
        "x": -110.0, "y": 180.0, "z": 0.35,
        "marker_z": 8.0,
        "type": "target",
        "color": {"r": 0.0, "g": 1.0, "b": 0.5}
    },
    "radar_tower_400m": {
        "label": "Radar Dome Tower (480m)",
        "short": "Radar Tower",
        "group": "WEST FLANK",
        "x": -110.0, "y": 480.0, "z": 42.5,
        "marker_z": 54.0,
        "type": "tower",
        "color": {"r": 0.8, "g": 0.2, "b": 1.0}
    },
    "heavy_truck_600m": {
        "label": "Heavy Hauler Truck (640m)",
        "short": "Heavy Hauler",
        "group": "WEST FLANK",
        "x": -110.0, "y": 640.0, "z": 7.5,
        "marker_z": 16.0,
        "type": "vehicle",
        "color": {"r": 1.0, "g": 0.5, "b": 0.0}
    },
    "target_pad_500m": {
        "label": "Red Target Bullseye Pad (800m)",
        "short": "Red Bullseye Pad",
        "group": "WEST FLANK",
        "x": -110.0, "y": 800.0, "z": 0.35,
        "marker_z": 8.0,
        "type": "target",
        "color": {"r": 1.0, "g": 0.2, "b": 0.3}
    },
    "hangar_building_150m": {
        "label": "Aircraft Hangar (960m)",
        "short": "Aircraft Hangar",
        "group": "WEST FLANK",
        "x": -110.0, "y": 960.0, "z": 10.0,
        "marker_z": 26.0,
        "type": "facility",
        "color": {"r": 0.3, "g": 0.7, "b": 1.0}
    },
    "building_500m": {
        "label": "West Complex Facility (500m)",
        "short": "Main Complex",
        "group": "WEST FLANK",
        "x": -160.0, "y": 500.0, "z": 25.0,
        "marker_z": 55.0,
        "type": "facility",
        "color": {"r": 0.8, "g": 0.5, "b": 0.2}
    },

    # --- EAST FLANK CORRIDOR (Right Side, Y=180m to Y=960m) ---
    "center_master_target_pad": {
        "label": "50m Master Bullseye Pad (180m)",
        "short": "Master Target Pad",
        "group": "EAST FLANK",
        "x": 50.0, "y": 180.0, "z": 0.35,
        "marker_z": 8.0,
        "type": "target",
        "color": {"r": 0.0, "g": 0.9, "b": 1.0}
    },
    "fuel_tanks_350m": {
        "label": "Fuel Storage Tanks (320m)",
        "short": "Fuel Farm",
        "group": "EAST FLANK",
        "x": 55.0, "y": 320.0, "z": 9.0,
        "marker_z": 24.0,
        "type": "facility",
        "color": {"r": 1.0, "g": 0.9, "b": 0.0}
    },
    "truck_blue_500m": {
        "label": "Blue Cargo Truck (480m)",
        "short": "Blue Cargo Truck",
        "group": "EAST FLANK",
        "x": 50.0, "y": 480.0, "z": 5.5,
        "marker_z": 16.0,
        "type": "vehicle",
        "color": {"r": 0.2, "g": 0.5, "b": 1.0}
    },
    "tri_color_target_pad_750m": {
        "label": "Tri-Color Target Pad (640m)",
        "short": "Tri-Color Pad",
        "group": "EAST FLANK",
        "x": 50.0, "y": 640.0, "z": 0.35,
        "marker_z": 8.0,
        "type": "target",
        "color": {"r": 1.0, "g": 0.1, "b": 0.8}
    },
    "wind_turbine_800m": {
        "label": "Wind Turbine Tower (800m)",
        "short": "Wind Turbine",
        "group": "EAST FLANK",
        "x": 50.0, "y": 800.0, "z": 72.0,
        "marker_z": 85.0,
        "type": "tower",
        "color": {"r": 0.9, "g": 0.9, "b": 0.9}
    },
    "target_tower_1000m": {
        "label": "60m Beacon Target Tower (960m)",
        "short": "Beacon Tower",
        "group": "EAST FLANK",
        "x": 50.0, "y": 960.0, "z": 65.0,
        "marker_z": 78.0,
        "type": "tower",
        "color": {"r": 0.0, "g": 1.0, "b": 0.4}
    },
    "van_white_1000m": {
        "label": "White Delivery Van (250m)",
        "short": "Delivery Van",
        "group": "EAST FLANK",
        "x": 100.0, "y": 250.0, "z": 6.0,
        "marker_z": 14.0,
        "type": "vehicle",
        "color": {"r": 0.8, "g": 0.9, "b": 0.2}
    },
    "solar_farm_900m": {
        "label": "Solar Panel Farm (750m)",
        "short": "Solar Farm",
        "group": "EAST FLANK",
        "x": 110.0, "y": 750.0, "z": 3.0,
        "marker_z": 12.0,
        "type": "facility",
        "color": {"r": 0.2, "g": 0.6, "b": 1.0}
    }
}

# The PERMANENT STATIC ORDER of rows (Never re-ordered!)
STATIC_ORDER = list(OBJECT_CONFIG.keys())

class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.drone_pos = [0.0, -25.0, 0.25]
        self.drone_quat = [0.0, 0.0, 1.0, 0.0]
        self.drone_heading_deg = 0.0
        self.drone_pitch_deg = 0.0
        self.drone_roll_deg = 0.0
        self.groundspeed = 0.0
        self.last_time = time.time()
        self.last_pos = [0.0, -25.0, 0.25]
        
        self.gimbal_angles = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.dynamic_positions = {
            "car_red_250m": {"x": -80.0, "y": 800.0, "z": 2.5, "speed": 30.0},
            "r1_rover": {"x": 30.0, "y": 200.0, "z": 0.38, "speed": 28.0}
        }
        self.last_update = 0.0
        self.connected = False

    def update_drone(self, pos, quat):
        with self.lock:
            now = time.time()
            dt = max(now - self.last_time, 0.001)
            dist_moved = math.sqrt((pos[0]-self.last_pos[0])**2 + (pos[1]-self.last_pos[1])**2)
            self.groundspeed = 0.8 * self.groundspeed + 0.2 * (dist_moved / dt)
            self.last_pos = list(pos)
            self.last_time = now
            self.drone_pos = list(pos)
            self.drone_quat = list(quat)

            # Convert quaternion [x, y, z, w] to Euler roll, pitch, yaw
            x, y, z, w = quat
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            self.drone_roll_deg = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                self.drone_pitch_deg = math.copysign(90.0, sinp)
            else:
                self.drone_pitch_deg = math.degrees(math.asin(sinp))

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            self.drone_heading_deg = (yaw + 360.0) % 360.0
            
            self.last_update = now
            self.connected = True

    def update_dynamic(self, name, pos):
        with self.lock:
            if name in self.dynamic_positions:
                prev = self.dynamic_positions[name]
                dt = max(time.time() - self.last_update, 0.001)
                d = math.sqrt((pos[0]-prev["x"])**2 + (pos[1]-prev["y"])**2)
                speed = d / dt if dt < 1.0 else prev.get("speed", 0.0)
                self.dynamic_positions[name] = {"x": pos[0], "y": pos[1], "z": pos[2], "speed": speed}

    def update_gimbal(self, r, p, y):
        with self.lock:
            self.gimbal_angles = {"roll": r, "pitch": p, "yaw": y}

state = TelemetryState()

def gz_pose_subscriber():
    """Runs gz topic subscriber to get live poses directly from Gazebo Sim"""
    topic = "/world/zephyr_runway/dynamic_pose/info"
    cmd = ["gz", "topic", "-e", "-t", topic]
    
    while True:
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
            curr_name = ""
            in_pos = False
            in_ori = False
            pos = [0.0, 0.0, 0.0]
            ori = [0.0, 0.0, 0.0, 1.0]

            for line in proc.stdout:
                line_str = line.strip()
                if line_str.startswith("name:"):
                    parts = line_str.split('"')
                    if len(parts) >= 2:
                        curr_name = parts[1]
                elif line_str.startswith("position"):
                    in_pos = True
                    in_ori = False
                    pos = [0.0, 0.0, 0.0]
                elif line_str.startswith("orientation"):
                    in_ori = True
                    in_pos = False
                    ori = [0.0, 0.0, 0.0, 1.0]
                elif line_str == "}":
                    if in_pos and curr_name:
                        in_pos = False
                    elif in_ori and curr_name:
                        in_ori = False
                        if curr_name == "zephyr_with_gimbal":
                            state.update_drone(pos, ori)
                        elif curr_name in state.dynamic_positions:
                            state.update_dynamic(curr_name, pos)
                elif in_pos:
                    if line_str.startswith("x:"):
                        pos[0] = float(line_str.split(":")[1])
                    elif line_str.startswith("y:"):
                        pos[1] = float(line_str.split(":")[1])
                    elif line_str.startswith("z:"):
                        pos[2] = float(line_str.split(":")[1])
                elif in_ori:
                    if line_str.startswith("x:"):
                        ori[0] = float(line_str.split(":")[1])
                    elif line_str.startswith("y:"):
                        ori[1] = float(line_str.split(":")[1])
                    elif line_str.startswith("z:"):
                        ori[2] = float(line_str.split(":")[1])
                    elif line_str.startswith("w:"):
                        ori[3] = float(line_str.split(":")[1])

            proc.wait()
        except Exception:
            time.sleep(1.0)

def compute_distances():
    """Computes distances in a 100% STATIC, PERMANENT ORDER."""
    with state.lock:
        d_x, d_y, d_z = state.drone_pos
        heading = state.drone_heading_deg
        speed = state.groundspeed
        pitch = state.drone_pitch_deg
        roll = state.drone_roll_deg
        connected = state.connected
        g_yaw = state.gimbal_angles["yaw"]
        g_pitch = state.gimbal_angles["pitch"]
        dyn = dict(state.dynamic_positions)

    # Calculate distances for every object in the static order
    raw_results = []
    for key in STATIC_ORDER:
        cfg = OBJECT_CONFIG[key]
        
        # Coordinates: use dynamic positions for moving targets
        if key in dyn:
            ox = dyn[key]["x"]
            oy = dyn[key]["y"]
            oz = dyn[key]["z"]
            cur_speed = dyn[key].get("speed", 0.0)
            is_dynamic = True
        else:
            ox = cfg["x"]
            oy = cfg["y"]
            oz = cfg["z"]
            cur_speed = 0.0
            is_dynamic = False

        dx = ox - d_x
        dy = oy - d_y
        dz = oz - d_z
        d_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
        d_horiz = math.sqrt(dx*dx + dy*dy)
        
        # True bearing (0 = North/+Y, 90 = East/+X)
        true_bearing = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
        rel_bearing = (true_bearing - heading + 540.0) % 360.0 - 180.0
        
        clock_hr = int(round((true_bearing - heading) / 30.0)) % 12
        if clock_hr == 0:
            clock_hr = 12
            
        raw_results.append({
            "id": key,
            "label": cfg["label"],
            "short": cfg["short"],
            "group": cfg["group"],
            "type": cfg["type"],
            "color": cfg["color"],
            "marker_z": oz + (cfg["marker_z"] - cfg["z"]),
            "d_3d": d_3d,
            "d_horiz": d_horiz,
            "dz": dz,
            "rel_bearing": rel_bearing,
            "clock": f"{clock_hr} o'clock",
            "is_dynamic": is_dynamic,
            "speed": cur_speed,
            "is_closest": False,
            "x": ox, "y": oy, "z": oz
        })

    # Find closest object to highlight it without reordering
    if raw_results:
        closest_obj = min(raw_results, key=lambda item: item["d_3d"])
        closest_obj["is_closest"] = True

    telemetry = {
        "pos": [round(d_x, 2), round(d_y, 2), round(d_z, 2)],
        "alt_agl": max(0.0, round(d_z - 0.20, 2)),
        "speed_ms": round(speed, 1),
        "speed_kmh": round(speed * 3.6, 1),
        "heading": round(heading, 1),
        "pitch": round(pitch, 1),
        "roll": round(roll, 1),
        "connected": connected,
        "gimbal": {"pitch": g_pitch, "yaw": g_yaw}
    }

    # Returns the STATIC list in exact permanent order
    return telemetry, raw_results

def gazebo_marker_publisher():
    """
    Publishes 3D distance text and glowing beacons directly over every object in Gazebo Sim.
    This places the live distance hovering right above each object in the 3D scene!
    """
    while True:
        try:
            telemetry, objects = compute_distances()
            if not telemetry["connected"]:
                time.sleep(1.0)
                continue
            
            marker_entries = []
            for idx, obj in enumerate(objects):
                m_id = idx + 1
                dist = obj["d_3d"]
                name = obj["short"]
                x = obj["x"]
                y = obj["y"]
                z = obj["marker_z"]
                col = obj["color"]
                
                # Tag text floating above the object
                if obj["is_closest"]:
                    tag_text = f"★ {name}: {dist:.0f}m [CLOSEST]"
                else:
                    tag_text = f"{name}: {dist:.0f}m"
                
                # 1. 3D Text Tag
                marker_entries.append(f"""  {{
    ns: "live_dist_text"
    id: {m_id}
    action: ADD_MODIFY
    type: TEXT
    text: "{tag_text}"
    pose: {{ position: {{ x: {x:.2f}, y: {y:.2f}, z: {z:.2f} }} }}
    scale: {{ x: 6.5, y: 6.5, z: 6.5 }}
    material: {{ ambient: {{ r: {col['r']}, g: {col['g']}, b: {col['b']}, a: 1.0 }}, diffuse: {{ r: {col['r']}, g: {col['g']}, b: {col['b']}, a: 1.0 }} }}
    visibility: ALL
  }}""")
                
                # 2. Glowing beacon sphere beneath the text
                marker_entries.append(f"""  {{
    ns: "live_beacon"
    id: {m_id + 100}
    action: ADD_MODIFY
    type: SPHERE
    pose: {{ position: {{ x: {x:.2f}, y: {y:.2f}, z: {z - 2.5:.2f} }} }}
    scale: {{ x: 2.2, y: 2.2, z: 2.2 }}
    material: {{ ambient: {{ r: {col['r']}, g: {col['g']}, b: {col['b']}, a: 0.9 }}, diffuse: {{ r: {col['r']}, g: {col['g']}, b: {col['b']}, a: 0.9 }} }}
    visibility: ALL
  }}""")
                
            payload = "marker: [\n" + ",\n".join(marker_entries) + "\n]"
            subprocess.run(
                ["gz", "service", "-s", "/marker_array", "--reqtype", "gz.msgs.Marker_V", "--reptype", "gz.msgs.Empty", "--timeout", "800", "--req", payload],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.5
            )
            time.sleep(0.5)
        except Exception:
            time.sleep(1.0)

# Web Server & Live HUD HTML
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Zephyr Gimbal - Live Object Distance HUD</title>
<style>
  :root {
    --bg: #0b0f19;
    --card: #151d30;
    --border: #233152;
    --cyan: #00d2ff;
    --green: #00f076;
    --yellow: #ffd000;
    --red: #ff3366;
    --text: #e2e8f0;
    --muted: #64748b;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace; }
  body { background: var(--bg); color: var(--text); padding: 20px; }
  header { display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid var(--border); padding-bottom: 15px; margin-bottom: 20px; }
  h1 { font-size: 22px; color: var(--cyan); letter-spacing: 1px; }
  .badge { padding: 4px 10px; border-radius: 4px; font-weight: bold; font-size: 12px; }
  .badge-live { background: rgba(0, 240, 118, 0.2); color: var(--green); border: 1px solid var(--green); }
  .badge-3d { background: rgba(0, 210, 255, 0.2); color: var(--cyan); border: 1px solid var(--cyan); margin-left: 10px; }
  .grid-telemetry { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 15px; margin-bottom: 25px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 15px; }
  .card .label { font-size: 11px; text-transform: uppercase; color: var(--muted); margin-bottom: 5px; }
  .card .val { font-size: 24px; font-weight: bold; color: #fff; }
  .card .val small { font-size: 13px; color: var(--muted); font-weight: normal; }
  table { width: 100%; border-collapse: collapse; background: var(--card); border-radius: 8px; overflow: hidden; border: 1px solid var(--border); }
  th { background: #1c2742; padding: 12px 14px; text-align: left; font-size: 12px; text-transform: uppercase; color: var(--cyan); border-bottom: 1px solid var(--border); }
  td { padding: 9px 14px; font-size: 13px; border-bottom: 1px solid rgba(255,255,255,0.05); }
  .group-row { background: #121929; font-weight: bold; color: var(--cyan); font-size: 12px; letter-spacing: 1px; padding: 6px 14px; }
  .row-closest { background: rgba(0, 240, 118, 0.1) !important; border-left: 4px solid var(--green); }
  .dist-highlight { font-size: 14px; font-weight: bold; color: var(--green); }
  .moving-tag { background: rgba(255, 51, 102, 0.2); color: var(--red); padding: 2px 6px; border-radius: 3px; font-size: 11px; font-weight: bold; }
  .target-tag { background: rgba(0, 210, 255, 0.2); color: var(--cyan); padding: 2px 6px; border-radius: 3px; font-size: 11px; }
  .closest-tag { background: rgba(0, 240, 118, 0.25); color: var(--green); padding: 2px 6px; border-radius: 3px; font-size: 11px; font-weight: bold; margin-left: 6px; }
</style>
</head>
<body>
  <header>
    <div>
      <h1>ZEPHYR GIMBAL &bull; REAL-TIME OBJECT DISTANCE HUD</h1>
      <p style="color:var(--muted); font-size:12px; margin-top:4px;">Static Fixed-Order Catalog &bull; 3D Floating Gazebo Overlays Active</p>
    </div>
    <div style="display:flex; align-items:center;">
      <div id="connBadge" class="badge badge-live">GAZEBO LINK: ACTIVE</div>
      <div class="badge badge-3d">GAZEBO 3D OVERLAYS: ON</div>
    </div>
  </header>

  <div class="grid-telemetry">
    <div class="card"><div class="label">Position (X, Y, Z)</div><div class="val" id="tPos">--</div></div>
    <div class="card"><div class="label">Altitude (AGL)</div><div class="val" id="tAlt">-- <small>m</small></div></div>
    <div class="card"><div class="label">Groundspeed</div><div class="val" id="tSpeed">-- <small>km/h</small></div></div>
    <div class="card"><div class="label">Heading</div><div class="val" id="tHeading">--<small>&deg;</small></div></div>
    <div class="card"><div class="label">Pitch / Roll</div><div class="val" id="tAtt">--</div></div>
    <div class="card"><div class="label">Closest Target</div><div class="val" id="tClosest" style="color:var(--yellow); font-size:18px;">--</div></div>
  </div>

  <table>
    <thead>
      <tr>
        <th>#</th>
        <th>Fixed Target / Object Name</th>
        <th>Zone</th>
        <th>Type</th>
        <th>3D Direct Distance</th>
        <th>Horizontal Ground Range</th>
        <th>Relative Altitude (&Delta;Z)</th>
        <th>Bearing</th>
        <th>Clock</th>
      </tr>
    </thead>
    <tbody id="tableBody">
      <tr><td colspan="9" style="text-align:center; padding:30px; color:var(--muted);">Connecting to Gazebo simulation stream...</td></tr>
    </tbody>
  </table>

  <script>
    async function updateHUD() {
      try {
        const res = await fetch('/api/distances');
        const data = await res.json();
        const t = data.telemetry;
        
        document.getElementById('tPos').innerText = `${t.pos[0]}, ${t.pos[1]}, ${t.pos[2]}`;
        document.getElementById('tAlt').innerHTML = `${t.alt_agl} <small>m</small>`;
        document.getElementById('tSpeed').innerHTML = `${t.speed_kmh} <small>km/h</small>`;
        document.getElementById('tHeading').innerHTML = `${t.heading}<small>&deg;</small>`;
        document.getElementById('tAtt').innerHTML = `${t.pitch}&deg; / ${t.roll}&deg;`;
        
        const closestObj = data.objects.find(o => o.is_closest);
        if (closestObj) {
          document.getElementById('tClosest').innerText = `${closestObj.short} (${closestObj.d_3d.toFixed(1)}m)`;
        }

        let rows = '';
        let lastGroup = '';
        data.objects.forEach((obj, idx) => {
          if (obj.group !== lastGroup) {
            lastGroup = obj.group;
            rows += `<tr><td colspan="9" class="group-row">&bull; ${obj.group}</td></tr>`;
          }

          const typeBadge = obj.is_dynamic ? 
            `<span class="moving-tag">MOVING (${obj.speed.toFixed(1)} m/s)</span>` :
            `<span class="target-tag">${obj.type.toUpperCase()}</span>`;
            
          const closestBadge = obj.is_closest ? `<span class="closest-tag">&#9733; CLOSEST</span>` : '';
          const rowClass = obj.is_closest ? 'class="row-closest"' : '';

          const bearingBadge = Math.abs(obj.rel_bearing) < 20 ? 
            `<span style="color:var(--green); font-weight:bold;">&uarr; ${obj.rel_bearing > 0 ? '+' : ''}${obj.rel_bearing.toFixed(1)}&deg; [AHEAD]</span>` :
            `${obj.rel_bearing > 0 ? '+' : ''}${obj.rel_bearing.toFixed(1)}&deg;`;

          rows += `<tr ${rowClass}>
            <td style="color:var(--muted);">${idx + 1}</td>
            <td style="font-weight:600; color:#fff;">${obj.label} ${closestBadge}</td>
            <td style="color:var(--muted); font-size:12px;">${obj.group}</td>
            <td>${typeBadge}</td>
            <td class="dist-highlight">${obj.d_3d.toFixed(1)} m</td>
            <td>${obj.d_horiz.toFixed(1)} m</td>
            <td style="color:${obj.dz >= 0 ? 'var(--cyan)' : 'var(--muted)'};">${obj.dz >= 0 ? '+' : ''}${obj.dz.toFixed(1)} m</td>
            <td>${bearingBadge}</td>
            <td><strong>${obj.clock}</strong></td>
          </tr>`;
        });
        document.getElementById('tableBody').innerHTML = rows;
      } catch (e) {
        document.getElementById('connBadge').innerText = 'WAITING FOR GAZEBO...';
        document.getElementById('connBadge').style.borderColor = 'var(--yellow)';
        document.getElementById('connBadge').style.color = 'var(--yellow)';
      }
    }
    setInterval(updateHUD, 200);
    updateHUD();
  </script>
</body>
</html>
"""

class HUDRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return  # Silence access logs for clean terminal output

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode("utf-8"))
        elif self.path == "/api/distances":
            telemetry, objects = compute_distances()
            payload = json.dumps({"telemetry": telemetry, "objects": objects})
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(payload.encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

def run_web_server(port=8088):
    class ReuseServer(socketserver.TCPServer):
        allow_reuse_address = True
    server = ReuseServer(("0.0.0.0", port), HUDRequestHandler)
    server.serve_forever()

def terminal_display_loop():
    """Renders a clean, 100% STATIC real-time terminal UI with fixed row positions."""
    while True:
        telemetry, objects = compute_distances()
        
        # ANSI clear screen & home
        out = []
        out.append("\033[2J\033[H")
        out.append("\033[1;36m===================================================================================================\033[0m")
        out.append("\033[1;37m   ZEPHYR GIMBAL &bull; LIVE OBJECT DISTANCE MONITOR (STATIC FIXED LIST + 3D GAZEBO LABELS)\033[0m")
        out.append(f"\033[1;34m   Web HUD: \033[1;32mhttp://localhost:8088\033[1;34m | Gazebo 3D Floating Overlays: \033[1;32mACTIVE ON SIM\033[0m")
        out.append("\033[1;36m===================================================================================================\033[0m")
        
        p = telemetry["pos"]
        conn_str = "\033[1;32m[CONNECTED]\033[0m" if telemetry["connected"] else "\033[1;33m[WAITING GAZEBO]\033[0m"
        out.append(f" Status: {conn_str} | Pos: X={p[0]:6.1f}m  Y={p[1]:6.1f}m  Z={p[2]:6.1f}m | Alt AGL: \033[1;33m{telemetry['alt_agl']}m\033[0m")
        out.append(f" Speed: \033[1;32m{telemetry['speed_kmh']} km/h\033[0m ({telemetry['speed_ms']} m/s) | Heading: \033[1;37m{telemetry['heading']:5.1f}°\033[0m | Pitch: {telemetry['pitch']}° | Roll: {telemetry['roll']}°")
        out.append("\033[1;36m---------------------------------------------------------------------------------------------------\033[0m")
        out.append(f" {'#':<3} {'PERMANENT TARGET / OBJECT NAME':<33} {'3D DIST':<11} {'HORIZ':<10} {'ALT (ΔZ)':<10} {'BEARING':<15} {'CLOCK'}")
        out.append("\033[1;36m---------------------------------------------------------------------------------------------------\033[0m")

        last_grp = ""
        for idx, obj in enumerate(objects):
            if obj["group"] != last_grp:
                last_grp = obj["group"]
                out.append(f"\033[1;34m-- {last_grp} ---------------------------------------------------------------------------------\033[0m")

            rel_b = obj["rel_bearing"]
            b_str = f"{rel_b:+5.1f}°"
            if abs(rel_b) < 15.0:
                b_str = f"\033[1;32m{b_str} [AHEAD]\033[0m"
            
            d_str = f"\033[1;32m{obj['d_3d']:7.1f} m\033[0m"
            name_str = obj['label']
            if len(name_str) > 32:
                name_str = name_str[:31] + "…"
                
            if obj["is_closest"]:
                name_str = f"\033[1;33m★ {name_str}\033[0m"
            elif obj["is_dynamic"]:
                name_str = f"\033[1;31m{name_str}\033[0m"
            
            dz_str = f"{obj['dz']:+6.1f} m"
            out.append(f" {idx+1:<3} {name_str:<44} {d_str:<20} {obj['d_horiz']:6.1f} m   {dz_str:<10} {b_str:<24} {obj['clock']}")

        out.append("\033[1;36m===================================================================================================\033[0m")
        out.append("\033[0;37m * 3D distance tags are hovering directly over each object in Gazebo. Rows above NEVER change order.\033[0m")
        
        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()
        time.sleep(0.25)

def main():
    port = 8088
    # 1. Start Gazebo subscriber thread
    sub_thread = threading.Thread(target=gz_pose_subscriber, daemon=True)
    sub_thread.start()

    # 2. Start Gazebo 3D Marker publisher thread (Draws distance tags directly over objects in Gazebo)
    marker_thread = threading.Thread(target=gazebo_marker_publisher, daemon=True)
    marker_thread.start()

    # 3. Start Web HUD thread
    web_thread = threading.Thread(target=run_web_server, args=(port,), daemon=True)
    web_thread.start()

    # 4. Run terminal display loop
    try:
        terminal_display_loop()
    except KeyboardInterrupt:
        print("\nDistance monitor stopped.")

if __name__ == "__main__":
    main()
