#!/usr/bin/env python3
"""
Obstacle Avoidance Node

Diese Node übernimmt die Hinderniserkennung und berechnet Umfahrungsziele.
Sie empfängt eine Hinderniskarte in GPS-Koordinaten und aktuelle Navigationsanfragen
und gibt Bypass-Punkte als XY-Werte relativ zu einem gemeinsamen Ursprung zurück.
"""

import json
import math
import os
import rospy
from collections import namedtuple
from std_msgs.msg import String

EARTH_RADIUS = 6_371_000.0  # m
ObstacleBox = namedtuple('ObstacleBox', ['lat_min', 'lon_min', 'lat_max', 'lon_max'])


class ObstacleAvoidanceNode:
    def __init__(self):
        self._init_state()
        self._init_ros()
        rospy.loginfo("Obstacle Avoidance Node gestartet")

    def _init_state(self):
        # Default-Hindernis bleibt erhalten, falls weder File noch /obstacle_map vorhanden sind.
        self.obstacles = [
            ObstacleBox(
                lat_min=50.04371426666667,
                lon_min=10.207508833333334,
                lat_max=50.043730733333334,
                lon_max=10.207527733333333,
            ),
        ]
        self.has_custom_map = False
        self.obstacle_margin = rospy.get_param('~obstacle_margin', 1.0)
        # Hindernisse kommen primär aus einem JSON-File (vom Frontend auf den Pi geschrieben);
        # /obstacle_map bleibt zusätzlich als Live-Update-Topic erhalten.
        self.obstacles_file = rospy.get_param('~obstacles_file',
                                              '/home/ubuntu/trackSprayRobot/shared_files/obstacles.json')
        self._load_obstacles_file()

    def _init_ros(self):
        self.response_pub = rospy.Publisher('/obstacle_bypass_response', String, queue_size=1)
        rospy.Subscriber('/obstacle_map', String, self._obstacle_map_callback)
        rospy.Subscriber('/obstacle_bypass_request', String, self._request_callback)

    def _parse_obstacles(self, data):
        """Wandelt eine Liste von Hindernis-Dicts in normalisierte ObstacleBox-Tupel um.

        Einzelne fehlerhafte Einträge werden übersprungen (mit Warnung), statt das
        gesamte File/die Map zu verwerfen.
        """
        boxes = []
        for i, o in enumerate(data):
            try:
                boxes.append(ObstacleBox(
                    lat_min=min(o['lat_min'], o['lat_max']),
                    lon_min=min(o['lon_min'], o['lon_max']),
                    lat_max=max(o['lat_min'], o['lat_max']),
                    lon_max=max(o['lon_min'], o['lon_max']),
                ))
            except (KeyError, TypeError) as e:
                rospy.logwarn(f"Hindernis-Eintrag {i} übersprungen (ungültig: {e})")
        return boxes

    def _load_obstacles_file(self):
        #TODO: default list only for testing! to be removed in future version
        """Lädt Hindernisse beim Start aus dem JSON-File; behält bei Fehlern die Default-Liste."""
        if not os.path.exists(self.obstacles_file):
            rospy.loginfo(f"Kein Obstacles-File ({self.obstacles_file}) -> Default "
                          f"({len(self.obstacles)} Hindernisse)")
            return
        try:
            with open(self.obstacles_file, "r") as f:
                data = json.load(f)
            self.obstacles = self._parse_obstacles(data)
            self.has_custom_map = True
            rospy.loginfo(f"Obstacles aus {self.obstacles_file} geladen: "
                          f"{len(self.obstacles)} Hindernisse")
        except (json.JSONDecodeError, KeyError, TypeError, OSError) as e:
            rospy.logwarn(f"Obstacles-File ungültig ({e}) -> Default "
                          f"({len(self.obstacles)} Hindernisse)")

    def _obstacle_map_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.obstacles = self._parse_obstacles(data)
            self.has_custom_map = True
            rospy.loginfo_throttle(10, f"Obstacle Map aktualisiert: {len(self.obstacles)} Hindernisse")
        except (json.JSONDecodeError, KeyError, TypeError) as e:
            rospy.logwarn(f"Ungültige Obstacle Map: {e}")

    def _request_callback(self, msg: String):
        try:
            request = json.loads(msg.data)
            obstacle_margin = float(request.get('obstacle_margin', self.obstacle_margin))

            if 'robot_x' in request and 'robot_y' in request and 'goal_x' in request and 'goal_y' in request:
                robot_x = float(request['robot_x'])
                robot_y = float(request['robot_y'])
                goal_x = float(request['goal_x'])
                goal_y = float(request['goal_y'])
                origin_lat = float(request['origin_lat'])
                origin_lon = float(request['origin_lon'])
                bypass_targets = self._compute_bypass_xy(robot_x, robot_y, goal_x, goal_y,
                                                        origin_lat, origin_lon, obstacle_margin)
            else:
                robot_lat = float(request['robot_lat'])
                robot_lon = float(request['robot_lon'])
                goal_lat = float(request['goal_lat'])
                goal_lon = float(request['goal_lon'])
                origin_lat = float(request['origin_lat'])
                origin_lon = float(request['origin_lon'])
                bypass_targets = self._compute_bypass(robot_lat, robot_lon, goal_lat, goal_lon,
                                                      origin_lat, origin_lon, obstacle_margin)
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as e:
            rospy.logwarn(f"Ungültige Bypass-Anfrage: {e}")
            return
        response = {'bypass_targets': bypass_targets}
        self.response_pub.publish(String(data=json.dumps(response)))

    def _compute_bypass(self, robot_lat, robot_lon, goal_lat, goal_lon,
                        origin_lat, origin_lon, obstacle_margin):
        robot_x, robot_y = self._gps_to_xy(robot_lat, robot_lon, origin_lat, origin_lon)
        goal_x, goal_y = self._gps_to_xy(goal_lat, goal_lon, origin_lat, origin_lon)

        blocking = self._blocking_obstacle(robot_x, robot_y, goal_x, goal_y,
                                          origin_lat, origin_lon, obstacle_margin)
        if blocking is None:
            return []

        return self._compute_bypass_points(robot_x, robot_y, goal_x, goal_y,
                                          origin_lat, origin_lon, blocking, obstacle_margin)

    def _compute_bypass_xy(self, robot_x, robot_y, goal_x, goal_y,
                            origin_lat, origin_lon, obstacle_margin):
        blocking = self._blocking_obstacle_xy(robot_x, robot_y, goal_x, goal_y,
                                              origin_lat, origin_lon, obstacle_margin)
        if blocking is None:
            return []
        ox_min, oy_min = self._gps_to_xy(blocking.lat_min, blocking.lon_min, origin_lat, origin_lon)
        ox_max, oy_max = self._gps_to_xy(blocking.lat_max, blocking.lon_max, origin_lat, origin_lon)
        return self._compute_bypass_points_xy(robot_x, robot_y, goal_x, goal_y,
                                             ox_min, oy_min, ox_max, oy_max, obstacle_margin)

    def _blocking_obstacle(self, p1x, p1y, p2x, p2y,
                           origin_lat, origin_lon, obstacle_margin):
        for obs in self.obstacles:
            ox_min, oy_min = self._gps_to_xy(obs.lat_min, obs.lon_min, origin_lat, origin_lon)
            ox_max, oy_max = self._gps_to_xy(obs.lat_max, obs.lon_max, origin_lat, origin_lon)
            if self._segment_intersects_rect(p1x, p1y, p2x, p2y,
                                            ox_min - obstacle_margin, oy_min - obstacle_margin,
                                            ox_max + obstacle_margin, oy_max + obstacle_margin):
                return obs
        return None

    def _blocking_obstacle_xy(self, p1x, p1y, p2x, p2y,
                               origin_lat, origin_lon, obstacle_margin):
        for obs in self.obstacles:
            ox_min, oy_min = self._gps_to_xy(obs.lat_min, obs.lon_min, origin_lat, origin_lon)
            ox_max, oy_max = self._gps_to_xy(obs.lat_max, obs.lon_max, origin_lat, origin_lon)
            if self._segment_intersects_rect(p1x, p1y, p2x, p2y,
                                            ox_min - obstacle_margin, oy_min - obstacle_margin,
                                            ox_max + obstacle_margin, oy_max + obstacle_margin):
                return obs
        return None

    def _segment_intersects_rect(self, p1x, p1y, p2x, p2y,
                                  x_min, y_min, x_max, y_max) -> bool:
        dx = p2x - p1x
        dy = p2y - p1y
        p = [-dx,  dx, -dy,  dy]
        q = [p1x - x_min, x_max - p1x, p1y - y_min, y_max - p1y]

        t_min, t_max = 0.0, 1.0
        for pi, qi in zip(p, q):
            if pi == 0.0:
                if qi < 0.0:
                    return False
            elif pi < 0.0:
                t_min = max(t_min, qi / pi)
            else:
                t_max = min(t_max, qi / pi)

        return t_min <= t_max

    def _compute_bypass_points(self, robot_x, robot_y, goal_x, goal_y,
                               origin_lat, origin_lon, obs, obstacle_margin):
        ox_min, oy_min = self._gps_to_xy(obs.lat_min, obs.lon_min, origin_lat, origin_lon)
        ox_max, oy_max = self._gps_to_xy(obs.lat_max, obs.lon_max, origin_lat, origin_lon)
        return self._compute_bypass_points_xy(robot_x, robot_y, goal_x, goal_y,
                                             ox_min, oy_min, ox_max, oy_max, obstacle_margin)

    def _compute_bypass_points_xy(self, robot_x, robot_y, goal_x, goal_y,
                                  ox_min, oy_min, ox_max, oy_max, obstacle_margin):
        ox_min -= obstacle_margin
        oy_min -= obstacle_margin
        ox_max += obstacle_margin
        oy_max += obstacle_margin
        extra = max(0.2, obstacle_margin * 0.2)

        center_x = 0.5 * (ox_min + ox_max)
        center_y = 0.5 * (oy_min + oy_max)

        if abs(goal_x - robot_x) >= abs(goal_y - robot_y):
            if goal_y >= center_y:
                side_points = [(ox_min, oy_max + extra), (ox_max, oy_max + extra)]
            else:
                side_points = [(ox_min, oy_min - extra), (ox_max, oy_min - extra)]
        else:
            if goal_x >= center_x:
                side_points = [(ox_max + extra, oy_min), (ox_max + extra, oy_max)]
            else:
                side_points = [(ox_min - extra, oy_min), (ox_min - extra, oy_max)]

        p1, p2 = sorted(side_points, key=lambda p: (p[0] - robot_x) ** 2 + (p[1] - robot_y) ** 2)

        if self._segment_intersects_rect(p2[0], p2[1], goal_x, goal_y, ox_min, oy_min, ox_max, oy_max):
            alternative = self._alternative_bypass_points(robot_x, robot_y, goal_x, goal_y,
                                                         ox_min, oy_min, ox_max, oy_max, extra)
            if alternative is not None:
                return alternative

        return [p1, p2]

    def _alternative_bypass_points(self, robot_x, robot_y, goal_x, goal_y,
                                   ox_min, oy_min, ox_max, oy_max, extra):
        candidates = [
            [(ox_min, oy_max + extra), (ox_max, oy_max + extra)],
            [(ox_min, oy_min - extra), (ox_max, oy_min - extra)],
            [(ox_max + extra, oy_min), (ox_max + extra, oy_max)],
            [(ox_min - extra, oy_min), (ox_min - extra, oy_max)],
        ]
        best = None
        best_cost = None
        for pair in candidates:
            p1, p2 = sorted(pair, key=lambda p: (p[0] - robot_x) ** 2 + (p[1] - robot_y) ** 2)
            if self._segment_intersects_rect(p2[0], p2[1], goal_x, goal_y, ox_min, oy_min, ox_max, oy_max):
                continue
            cost = (math.hypot(p1[0] - robot_x, p1[1] - robot_y) +
                    math.hypot(p2[0] - p1[0], p2[1] - p1[1]) +
                    math.hypot(goal_x - p2[0], goal_y - p2[1]))
            if best is None or cost < best_cost:
                best = [p1, p2]
                best_cost = cost
        return best

    def _gps_to_xy(self, lat, lon, origin_lat, origin_lon):
        dlat = math.radians(lat - origin_lat)
        dlon = math.radians(lon - origin_lon)
        x = dlon * EARTH_RADIUS * math.cos(math.radians(origin_lat))
        y = dlat * EARTH_RADIUS
        return x, y


if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    ObstacleAvoidanceNode()
    rospy.spin()
