import os

DATEI = "/home/ubuntu/dosenstand.txt"
GRAMM_PRO_STOSS = 0.25
STARTGEWICHT = 250.0


def spray():
    gewicht = lade_gewicht()

    gewicht -= GRAMM_PRO_STOSS
    gewicht = max(0, gewicht)

    speichere_gewicht(gewicht)

    return int(gewicht / GRAMM_PRO_STOSS)


def lade_gewicht():
    if os.path.exists(DATEI):
        with open(DATEI, "r") as f:
            return float(f.read())

    return STARTGEWICHT


def speichere_gewicht(gewicht):
    with open(DATEI, "w") as f:
        f.write(str(gewicht))