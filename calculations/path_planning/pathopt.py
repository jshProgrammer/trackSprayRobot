#!/usr/bin/env python3

import json
import math
import os
from pyproj import Transformer

START_LAT = 49.7913   # GPS Start, später ersetzen durch Robot GPS
START_LON = 9.9534 # Same here
START_HEADING = 0.0   # rad (0 = Osten, wichtig!)

# ---------------------------
# JSON laden
# ---------------------------
def load_waypoints(file_path):
    with open(file_path) as f:
        data = json.load(f)
    return [(p["x"], p["y"]) for p in data["waypoints"]]


# ---------------------------
# DISTANZ
# ---------------------------
def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------
# NEAREST NEIGHBOR
# ---------------------------
def optimize_path(points, start=(0, 0)):
    points = points.copy()
    path = []
    current = start

    while points:
        next_p = min(points, key=lambda p: dist(p, current))
        path.append(next_p)
        points.remove(next_p)
        current = next_p

    return path


# ---------------------------
# ROTATION (Heading)
# ---------------------------
def rotate_point(x, y, yaw):
    xr = x * math.cos(yaw) - y * math.sin(yaw)
    yr = x * math.sin(yaw) + y * math.cos(yaw)
    return xr, yr


# ---------------------------
# PROJ SETUP
# ---------------------------
def create_transformer(lat, lon):
    # WGS84 → lokales ENU
    proj_string = f"+proj=aeqd +lat_0={lat} +lon_0={lon} +units=m +ellps=WGS84"
    return Transformer.from_proj(proj_string, "epsg:4326", always_xy=True)


# ---------------------------
# LOKAL → GPS
# später auslagern, umrechnung erst zu beginn von Navigation, damit wir den richtige GPS Startpunkt haben
# außer wir senden aktuelle GPS Koordinaten ans Frontend für Visualisierung, dann können wir auch GPS Koordinaten in point.json speichern und können direkt umrechnen
# ---------------------------
def local_to_gps(transformer, x, y):
    lon, lat = transformer.transform(x, y)
    return lat, lon


# ---------------------------
# HAUPTPIPELINE
# ---------------------------
def process(file_path):
    points = load_waypoints(file_path)

    print(f"Original: {len(points)} Punkte")

    # 1. Optimieren
    points = optimize_path(points)

    # 2. Transformer erstellen
    transformer = create_transformer(START_LAT, START_LON)

    gps_points = []

    for x, y in points:
        # 3. Rotation (Heading)
        xr, yr = rotate_point(x, y, START_HEADING)

        # 4. Transformation
        lat, lon = local_to_gps(transformer, xr, yr)

        gps_points.append({"lat": lat, "lon": lon})

    return gps_points


# ---------------------------
# MAIN
# ---------------------------
if __name__ == "__main__":
    file_path = os.path.join(os.path.dirname(__file__), "points.json")
    result = process(file_path)

    output = {"waypoints_gps":result}
    output_file = os.path.join(os.path.dirname(__file__), "gps_points.json")
    with open(output_file, "w") as file:
        json.dump(output, file, indent=4)
    for p in result:
        print(p)
