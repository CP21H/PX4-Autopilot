#!/usr/bin/env python3
"""
Generate a Gazebo (gz sim / Harmonic) world for a PX4 SITL autonomous drone-racing track
inside a warehouse. Pure stdlib, no dependencies.

Usage:
    python3 make_track_world.py                 # writes drone_track.sdf + gates.json
    python3 make_track_world.py --no-hall       # track on an open ground plane
    python3 make_track_world.py --gate-open 2.5 # change gate opening size

Coordinates: Gazebo ENU (x east, y north, z up), meters.
Origin = bottom-left corner of the track rectangle in your top-down image.
Everything below was measured from that image (~19.9 px per meter), so treat the
numbers as a first pass and edit the tables.
"""
import argparse
import json
import math
from pathlib import Path

# --------------------------------------------------------------------------
# TRACK DEFINITION  (edit me)
# yaw_deg: direction of the gate's NORMAL (the way you fly through it).
#   0   -> gate opening faces +x (you pass east/west)
#   90  -> gate opening faces +y (you pass north/south)
# z: height of the CENTER of the opening.
# Listed in a guessed flight order (start -> ... -> finish). Verify against
# your numbered 3D view and reorder if needed.
# --------------------------------------------------------------------------
GATES = [
    # name,  x,     y,     z,   yaw_deg, color
    ("g01", 18.2,  10.15, 1.5,  60, "yellow"),
    ("g02", 21.25, 17.75, 1.5,  90, "yellow"),
    ("g03", 12.15, 23.8,  1.5,   0, "yellow"),
    ("g04", 20.0,  28.7,  1.5,  90, "yellow"),
    ("g05", 21.9,  37.85, 1.5,  90, "yellow"),
    ("g06", 12.15, 44.9,  1.5,   0, "yellow"),
    ("g07", 4.85,  38.15, 1.5,  90, "yellow"),
    ("g08", 3.65,  28.4,  1.5,  90, "yellow"),
    ("g09", 2.95,  21.05, 1.5,  90, "yellow"),   # drawn fainter in your image
    ("g10", 3.95,  16.6,  1.5,  90, "blue"),
    ("g11", 12.15, 4.9,   1.5,  90, "blue"),     # start/finish gate
]

# Start pose for the drone (just behind g11, facing north)
START = dict(x=12.15, y=2.5, z=0.2, yaw_rad=math.pi / 2)

# Cones (x, y, color) read off the top-down view
CONES = [
    (0.0, 33.7, "red"), (0.0, 17.5, "red"), (0.0, 1.2, "red"),
    (16.0, 50.0, "red"), (16.0, 33.7, "red"), (16.0, 17.5, "red"), (16.0, 1.2, "red"),
    (0.0, 50.0, "orange"), (0.0, 31.7, "orange"), (0.0, 13.5, "orange"), (0.0, -1.4, "orange"),
    (16.0, 31.7, "orange"), (16.0, 13.5, "orange"), (16.0, -1.4, "orange"),
]
TRACK_BBOX = (-2.0, -3.0, 28.0, 52.0)  # xmin, ymin, xmax, ymax (columns avoid this)

# Warehouse (estimates from your photo - measure the real thing if you can)
HALL = dict(x0=-30.0, x1=56.0, y0=-30.0, y1=80.0, h=12.0)
COLUMN_SPACING = 20.0
LIGHT_SPACING = 22.0

COLORS = {
    "yellow": (0.95, 0.68, 0.10, 1),
    "blue": (0.15, 0.55, 0.90, 1),
    "red": (0.88, 0.14, 0.08, 1),
    "orange": (0.95, 0.45, 0.05, 1),
    "concrete": (0.55, 0.55, 0.57, 1),
    "wall": (0.38, 0.40, 0.42, 1),
    "ceiling": (0.12, 0.12, 0.13, 1),
    "steel": (0.45, 0.50, 0.55, 1),
    "white": (1, 1, 1, 1),
}


def c(name):
    return " ".join(str(v) for v in COLORS[name])


def box_visual(name, size, pose, color, emissive=None, collision=True, rough=None):
    sx, sy, sz = size
    px, py, pz = pose[:3]
    yaw = pose[5] if len(pose) > 5 else 0.0
    pose_s = f"{px:.4f} {py:.4f} {pz:.4f} 0 0 {yaw:.5f}"
    em = f"<emissive>{c(emissive)}</emissive>" if emissive else ""
    pbr = ""
    if rough is not None:
        pbr = f"<pbr><metal><metalness>0.0</metalness><roughness>{rough}</roughness></metal></pbr>"
    col = ""
    if collision:
        col = (f"<collision name='{name}_col'><pose>{pose_s}</pose>"
               f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>")
    return (f"<visual name='{name}'><pose>{pose_s}</pose>"
            f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
            f"<material><ambient>{c(color)}</ambient><diffuse>{c(color)}</diffuse>{em}{pbr}</material>"
            f"</visual>{col}")


def gate_model(name, x, y, zc, yaw_deg, color, opening, bar, depth):
    """Rectangular frame in the local y-z plane, normal along local +x, on a floor stand."""
    W = H = opening
    T, D = bar, depth
    parts = [
        box_visual("left", (D, T, H + 2 * T), (0, -(W / 2 + T / 2), 0), color),
        box_visual("right", (D, T, H + 2 * T), (0, (W / 2 + T / 2), 0), color),
        box_visual("top", (D, W, T), (0, 0, H / 2 + T / 2), color),
        box_visual("bottom", (D, W, T), (0, 0, -(H / 2 + T / 2)), color),
        # emissive inner strips (the real gates have bright LED edges)
        box_visual("led_l", (D + 0.02, 0.03, H), (0, -(W / 2 - 0.015), 0), "white", emissive="white", collision=False),
        box_visual("led_r", (D + 0.02, 0.03, H), (0, (W / 2 - 0.015), 0), "white", emissive="white", collision=False),
    ]
    stand_h = zc - H / 2 - T
    if stand_h > 0.05:
        parts.append(box_visual("stand", (0.12, 0.12, stand_h), (0, 0, -(zc - stand_h / 2)), "steel"))
    return (f"<model name='{name}'><static>true</static>"
            f"<pose>{x} {y} {zc} 0 0 {math.radians(yaw_deg):.5f}</pose>"
            f"<link name='link'>{''.join(parts)}</link></model>")


def cone_model(name, x, y, color):
    r, l = 0.2, 0.7
    geom = f"<cone><radius>{r}</radius><length>{l}</length></cone>"
    return (f"<model name='{name}'><static>true</static><pose>{x} {y} {l / 2} 0 0 0</pose>"
            f"<link name='link'><visual name='v'><geometry>{geom}</geometry>"
            f"<material><ambient>{c(color)}</ambient><diffuse>{c(color)}</diffuse></material></visual>"
            f"<collision name='c'><geometry>{geom}</geometry></collision></link></model>")


def hall_model():
    x0, x1, y0, y1, h = (HALL[k] for k in ("x0", "x1", "y0", "y1", "h"))
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    lx, ly = x1 - x0, y1 - y0
    t = 0.3
    p = []
    # polished concrete floor (top surface at z=0)
    p.append(box_visual("floor", (lx, ly, 0.2), (cx, cy, -0.1), "concrete", rough=0.25))
    # walls
    p.append(box_visual("wall_s", (lx, t, h), (cx, y0, h / 2), "wall"))
    p.append(box_visual("wall_n", (lx, t, h), (cx, y1, h / 2), "wall"))
    p.append(box_visual("wall_w", (t, ly, h), (x0, cy, h / 2), "wall"))
    p.append(box_visual("wall_e", (t, ly, h), (x1, cy, h / 2), "wall"))
    # ceiling deck + trusses
    p.append(box_visual("ceiling", (lx, ly, 0.2), (cx, cy, h + 0.1), "ceiling"))
    n = 0
    y = y0 + 5
    while y < y1:
        p.append(box_visual(f"truss_y{n}", (lx, 0.25, 0.6), (cx, y, h - 0.3), "steel", collision=False))
        y += 6.0
        n += 1
    # columns (skip any that would land in the track area)
    bx0, by0, bx1, by1 = TRACK_BBOX
    n = 0
    xs = [x0 + 6 + i * COLUMN_SPACING for i in range(int((lx - 12) // COLUMN_SPACING) + 1)]
    ys = [y0 + 6 + j * COLUMN_SPACING for j in range(int((ly - 12) // COLUMN_SPACING) + 1)]
    for xx in xs:
        for yy in ys:
            if bx0 - 2 < xx < bx1 + 2 and by0 - 2 < yy < by1 + 2:
                continue
            p.append(box_visual(f"col{n}", (0.6, 0.6, h), (xx, yy, h / 2), "steel"))
            n += 1
    # ceiling light panels (emissive) + cheap point lights
    lights = []
    n = 0
    xx = x0 + LIGHT_SPACING / 2
    while xx < x1:
        yy = y0 + LIGHT_SPACING / 2
        while yy < y1:
            p.append(box_visual(f"panel{n}", (3.0, 0.4, 0.05), (xx, yy, h - 0.05), "white", emissive="white", collision=False))
            lights.append(
                f"<light name='lamp{n}' type='point'><pose>{xx} {yy} {h - 0.8}</pose>"
                f"<cast_shadows>false</cast_shadows><diffuse>1 1 0.95 1</diffuse><specular>0.2 0.2 0.2 1</specular>"
                f"<attenuation><range>35</range><constant>0.4</constant><linear>0.02</linear><quadratic>0.001</quadratic></attenuation>"
                f"</light>")
            n += 1
            yy += LIGHT_SPACING
        xx += LIGHT_SPACING
    model = (f"<model name='warehouse'><static>true</static><link name='link'>{''.join(p)}</link></model>")
    return model + "".join(lights)


HEADER = """<?xml version="1.0" ?>
<sdf version="1.9">
<world name="drone_track">
  <physics type="ode">
    <max_step_size>0.004</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>250</real_time_update_rate>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
  <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
  <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
  <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
  <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <gravity>0 0 -9.8</gravity>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
  <atmosphere type="adiabatic"/>
  <scene>
    <ambient>0.45 0.45 0.45 1</ambient>
    <background>0.2 0.2 0.22 1</background>
    <shadows>false</shadows>
  </scene>
  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.397971057728974</latitude_deg>
    <longitude_deg>8.546163739800146</longitude_deg>
    <elevation>289.0</elevation>
  </spherical_coordinates>
"""

OPEN_GROUND = """
  <model name="ground_plane"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
    <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
  </link></model>
  <light name="sun" type="directional"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction></light>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="drone_track.sdf")
    ap.add_argument("--gate-open", type=float, default=2.0, help="square inner opening, m")
    ap.add_argument("--bar", type=float, default=0.2, help="frame bar thickness, m")
    ap.add_argument("--depth", type=float, default=0.2, help="frame depth along flight direction, m")
    ap.add_argument("--no-hall", action="store_true")
    a = ap.parse_args()

    body = [HEADER]
    body.append(OPEN_GROUND if a.no_hall else hall_model())
    for name, x, y, z, yaw, col in GATES:
        body.append(gate_model(name, x, y, z, yaw, col, a.gate_open, a.bar, a.depth))
    for i, (x, y, col) in enumerate(CONES):
        body.append(cone_model(f"cone{i}", x, y, col))
    body.append("</world></sdf>")
    Path(a.out).write_text("\n".join(body))

    meta = {
        "start": START,
        "gate_opening_m": a.gate_open,
        "gates_in_order": [
            {"name": n, "x": x, "y": y, "z": z, "yaw_deg": yaw} for n, x, y, z, yaw, _ in GATES
        ],
    }
    Path("gates.json").write_text(json.dumps(meta, indent=2))
    yaw = START["yaw_rad"]
    print(f"wrote {a.out} and gates.json")
    print(f'PX4_GZ_MODEL_POSE="{START["x"]},{START["y"]},{START["z"]},0,0,{yaw:.3f}"')


if __name__ == "__main__":
    main()
