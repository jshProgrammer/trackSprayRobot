import os

# Pfad zur Dosenstand-Datei. Default unverändert (/home/ubuntu/dosenstand.txt), damit der
# Frontend-Vertrag stabil bleibt; per Umgebungsvariable DOSENSTAND_FILE überschreibbar.
DATEI = os.environ.get("DOSENSTAND_FILE", "/home/ubuntu/dosenstand.txt")
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