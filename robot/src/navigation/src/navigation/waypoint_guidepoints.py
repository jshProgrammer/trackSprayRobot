"""Waypoints & virtuelle Leitpunkte.

Zwei zusammengehörige Aufgaben:
  * ``load_waypoints`` – Waypoints aus JSON-File laden (rosparam-Fallback)
  * ``generate_guide_points`` – virtuelle Leitpunkte ("Lotse") auf einer
    Bézier-Kurve zum Ziel (reine Geometrie, ROS-frei)
"""

import json
import math
import os

import rospy

from navigation.geo import bezier_point


def _coerce(p):
    # Dict-Format ({"lat": ..., "lon": ...}, z.B. aus shared_files/waypoints.json)
    # ebenso wie das alte Listen-Format ([lat, lon]) unterstützen.
    if isinstance(p, dict):
        return [float(p["lat"]), float(p["lon"])]
    return [float(p[0]), float(p[1])]


def load_waypoints(waypoints_file: str):
    """Lädt Waypoints aus dem JSON-File; fällt auf rosparam ~waypoints zurück.

    Akzeptiert sowohl eine reine Liste [[lat, lon], ...] als auch die Objektform
    {"waypoints": [[lat, lon], ...]}. Bei Fehlern wird der rosparam-Fallback genutzt,
    damit ein fehlendes/kaputtes File die Navigation nicht crasht.
    """
    fallback = rospy.get_param('~waypoints', [])

    if not os.path.exists(waypoints_file):
        rospy.loginfo(f"Kein Waypoints-File ({waypoints_file}) -> rosparam-Fallback "
                      f"({len(fallback)} Punkte)")
        return fallback

    try:
        with open(waypoints_file, "r") as f:
            data = json.load(f)
        if isinstance(data, dict) and 'waypoints' in data:
            raw = data['waypoints']
        elif isinstance(data, list):
            raw = data
        else:
            raise ValueError("Ungültiges Waypoints-Format (erwarte Liste oder {'waypoints': [...]})")

        # Einzelne fehlerhafte Wegpunkte überspringen (mit Warnung), statt das
        # gesamte File zu verwerfen und auf den rosparam-Fallback zu kippen.
        waypoints = []
        for i, p in enumerate(raw):
            try:
                waypoints.append(_coerce(p))
            except (KeyError, TypeError, ValueError, IndexError) as e:
                rospy.logwarn(f"Wegpunkt {i} übersprungen (ungültig: {e})")
        rospy.loginfo(f"Waypoints aus {waypoints_file} geladen: {len(waypoints)} Punkte")
        return waypoints
    except (json.JSONDecodeError, ValueError, TypeError, KeyError, IndexError, OSError) as e:
        rospy.logwarn(f"Waypoints-File ungültig ({e}) -> rosparam-Fallback "
                      f"({len(fallback)} Punkte)")
        return fallback


def generate_guide_points(start_x: float, start_y: float, start_heading: float,
                          goal_x: float, goal_y: float,
                          spacing: float, min_goal_dist: float, final_gap: float):
    """Erzeugt Leitpunkte auf einer kubischen Bézier-Kurve vom Roboter zum Ziel.

    Start-Tangente = aktuelles Roboter-Heading
    End-Tangente   = direkte Richtung aufs Ziel
    => Der Roboter wird sanft eingedreht und kommt am letzten Leitpunkt
       bereits ideal ausgerichtet an; die Endanfahrt ist eine kurze Gerade.

    spacing       – Abstand zwischen zwei Leitpunkten entlang der Kurve
    min_goal_dist – unterhalb dieser Zieldistanz werden keine Leitpunkte erzeugt
    final_gap     – Leitpunkte näher als dieser Abstand am Ziel werden weggelassen
    """
    dx = goal_x - start_x
    dy = goal_y - start_y
    dist = math.hypot(dx, dy)

    # Zu nah am Ziel -> Leitpunkte bringen nichts, direkt anfahren
    if dist < min_goal_dist:
        return []

    goal_heading = math.atan2(dy, dx)

    # Kontrollpunkte der Bézier-Kurve
    d = dist / 3.0
    p0 = (start_x, start_y)
    p1 = (start_x + d * math.cos(start_heading),
          start_y + d * math.sin(start_heading))
    p2 = (goal_x - d * math.cos(goal_heading),
          goal_y - d * math.sin(goal_heading))
    p3 = (goal_x, goal_y)

    # Kurve fein abtasten und Punkte im Abstand spacing entnehmen
    n_samples = max(20, int(dist * 4))
    guides = []
    acc = 0.0
    last = p0
    for i in range(1, n_samples + 1):
        t = i / n_samples
        pt = bezier_point(p0, p1, p2, p3, t)
        acc += math.hypot(pt[0] - last[0], pt[1] - last[1])
        last = pt
        if acc >= spacing:
            acc = 0.0
            # Leitpunkte zu nah am Ziel weglassen -> die letzte Anfahrt
            # bleibt frei und gerade (dort gilt wieder waypoint_tolerance)
            if math.hypot(goal_x - pt[0], goal_y - pt[1]) > final_gap:
                guides.append(pt)
    return guides
