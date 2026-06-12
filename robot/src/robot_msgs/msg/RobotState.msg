# Aktueller Navigations-Lebenszustand fürs Frontend (edge + bei Wegpunktwechsel, latched).
uint8 STATE_IDLE=0           # wartet auf stabilen RTK-FIXED (intern WAITING_FOR_FIX)
uint8 STATE_CALIBRATING=1    # fährt 3 m geradeaus zur Heading-Kalibrierung
uint8 STATE_NAVIGATING=2     # fährt zum aktuellen Ziel-Wegpunkt
uint8 STATE_GOAL_REACHED=3   # alle Wegpunkte abgefahren

time    stamp
uint8   state            # einer der STATE_* Werte
string  state_name       # "IDLE" | "CALIBRATING" | "NAVIGATING" | "GOAL_REACHED"
uint16  waypoint_index   # 1-basiert (Wegpunkt N von total); 0 wenn n/a
uint16  waypoint_total   # Anzahl Ziel-Wegpunkte
float64 target_lat       # Zielkoordinate (nur im Zustand NAVIGATING gesetzt, sonst 0.0)
float64 target_lon
