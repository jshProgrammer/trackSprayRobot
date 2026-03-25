#!/usr/bin/env python3
"""
interfaces.py
=============
Abstrakte Basisklassen fuer Drive, Actuator und Marker.
Sowohl Simulation- als auch Hardware-Backends implementieren diese.
"""

from abc import ABC, abstractmethod
from geometry_msgs.msg import Point
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class DriveInterface(ABC):
    """Fahrantrieb – sendet Geschwindigkeitsbefehle."""

    @abstractmethod
    def set_velocity(self, linear_x: float) -> None:
        """Setzt die Vorwaertsgeschwindigkeit in m/s (0 = stopp)."""
        ...


class ActuatorInterface(ABC):
    """Spray-Aktuator – bewegt den Stab hoch/runter."""

    @abstractmethod
    def move_to(self, position: float, duration: float) -> None:
        """
        Faehrt den Stab auf `position` (Meter).
        `duration` gibt an, wie lange die Bewegung dauern soll (s).
        """
        ...


class MarkerInterface(ABC):
    """Bodenmarkierung – platziert einen sichtbaren Farbfleck."""

    @abstractmethod
    def place_marker(self, position: Point, marker_id: int) -> None:
        """
        Setzt einen Marker an `position` (x/y in Weltkoordinaten).
        `marker_id` muss je Aufruf eindeutig sein.
        """
        ...