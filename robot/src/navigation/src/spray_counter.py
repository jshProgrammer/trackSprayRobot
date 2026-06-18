import os

# Gemeinsamer Laufzeit-State fuer Robot und Frontend. Der alte Pfad bleibt als
# Lesefallback erhalten; geschrieben wird immer in die shared_files-Datei.
DEFAULT_DATEI = "/home/ubuntu/trackSprayRobot/shared_files/dosenstand.txt"
LEGACY_DATEI = "/home/ubuntu/dosenstand.txt"
DATEI = os.environ.get("DOSENSTAND_FILE", DEFAULT_DATEI)
GRAMM_PRO_STOSS = 0.25
STARTGEWICHT = 250.0


def _lese_datei():
    if os.path.exists(DATEI):
        return DATEI
    if DATEI == DEFAULT_DATEI and os.path.exists(LEGACY_DATEI):
        return LEGACY_DATEI
    return DATEI


def spray():
    gewicht = lade_gewicht()

    gewicht -= GRAMM_PRO_STOSS
    gewicht = max(0, gewicht)

    speichere_gewicht(gewicht)

    return int(gewicht / GRAMM_PRO_STOSS)


def lade_gewicht():
    datei = _lese_datei()
    if os.path.exists(datei):
        with open(datei, "r") as f:
            return float(f.read())

    return STARTGEWICHT


def speichere_gewicht(gewicht):
    os.makedirs(os.path.dirname(DATEI), exist_ok=True)
    with open(DATEI, "w") as f:
        f.write(str(gewicht))
