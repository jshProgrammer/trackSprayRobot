# Elementares Status-/Fehler-Signal für das Frontend (edge-triggered, latched).
time   stamp     # Zeitpunkt des Events
string type      # "info" | "warn" | "error"
string source    # meldende Node, z.B. "gps_node"
string code      # maschinenlesbare Event-ID, z.B. "RTK_LOST"
string message   # menschenlesbarer Klartext (DE)
