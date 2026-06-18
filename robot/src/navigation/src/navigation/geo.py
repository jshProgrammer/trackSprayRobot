"""Reine Geometrie-/Koordinaten-Mathematik (kein ROS).

Diese Funktionen sind bewusst frei von rospy, damit sie ohne laufenden
ROS-Master unit-getestet werden können.
"""

import math

EARTH_RADIUS = 6_371_000.0   # m


def quaternion_to_yaw(q) -> float:
    """Extracts yaw angle (heading) from a quaternion. Returns angle in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Normalisiert einen Winkel auf das Intervall (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def latlon_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Approximate planar distance in meters between two lat/lon points."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    x = dlon * EARTH_RADIUS * math.cos(math.radians(lat1))
    y = dlat * EARTH_RADIUS
    return math.hypot(x, y)


def gps_to_xy(lat: float, lon: float, origin_lat: float, origin_lon: float):
    """Rechnet eine lat/lon-Position relativ zum Ursprung in lokale x/y-Meter um."""
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    x = dlon * EARTH_RADIUS * math.cos(math.radians(origin_lat))
    y = dlat * EARTH_RADIUS
    return x, y


def bezier_point(p0, p1, p2, p3, t: float):
    """Punkt auf einer kubischen Bézier-Kurve bei Parameter t in [0, 1]."""
    u = 1.0 - t
    x = (u**3) * p0[0] + 3 * (u**2) * t * p1[0] + 3 * u * (t**2) * p2[0] + (t**3) * p3[0]
    y = (u**3) * p0[1] + 3 * (u**2) * t * p1[1] + 3 * u * (t**2) * p2[1] + (t**3) * p3[1]
    return (x, y)
