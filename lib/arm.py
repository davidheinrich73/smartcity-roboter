#!/usr/bin/env python3
# Greifarm des M3 Pro (DOFBOT-Pro, 6 Servos).
#
# Schnittstelle (Quelle: oeffentlicher Treiber fuer den M3 Pro, siehe docs/greifarm.md,
# NOCH NICHT auf unserem Roboter geprueft -> scripts/arm_suchen.sh):
#   Topic /arm6_joints, Typ arm_msgs/msg/ArmJoints
#   Felder joint1..joint6 = Winkel in GANZEN GRAD, time = Dauer der Bewegung in ms
#   Servo 1-4: 0-180, Servo 5 (Handgelenk): 0-270, Servo 6 (Greifer): 30 = zu, 180 = auf
# Das Board meldet die aktuelle Armstellung NICHT zurueck. Man weiss also nur,
# was man zuletzt geschickt hat.
import os

GRENZEN = {1: (0, 180), 2: (0, 180), 3: (0, 180), 4: (0, 180), 5: (0, 270), 6: (30, 180)}
GREIFER_ZU, GREIFER_AUF = 30, 180
NAMEN = ['Servo 1 (Drehen unten)', 'Servo 2 (Schulter)', 'Servo 3 (Ellbogen)',
         'Servo 4 (Handgelenk kippen)', 'Servo 5 (Handgelenk drehen)', 'Servo 6 (Greifer)']
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
POSEN_DATEI = os.environ.get('SMARTCITY_ARM_YAML', os.path.join(REPO, 'config', 'arm.yaml'))


def pruefe(winkel, zeit_ms):
    """Fehlertext oder None. Werte ausserhalb werden abgelehnt, nicht gekuerzt."""
    if len(winkel) != 6:
        return 'Es muessen genau 6 Winkel sein'
    for i, w in enumerate(winkel, start=1):
        lo, hi = GRENZEN[i]
        if not isinstance(w, (int, float)) or not lo <= w <= hi:
            return f'{NAMEN[i - 1]}: {w} liegt ausserhalb {lo}..{hi} Grad'
    if not 300 <= zeit_ms <= 10000:
        return 'Zeit muss zwischen 300 und 10000 ms liegen (langsam = sicherer)'
    return None


def lade_posen():
    """Gespeicherte Posen aus config/arm.yaml: {name: [6 Winkel]} (fehlende = None)."""
    try:
        import yaml
        with open(POSEN_DATEI) as f:
            daten = yaml.safe_load(f) or {}
        return {k: v for k, v in (daten.get('posen') or {}).items()}
    except FileNotFoundError:
        return {}


def lade_einstellungen():
    try:
        import yaml
        with open(POSEN_DATEI) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def speichere_pose(name, winkel):
    import yaml
    daten = lade_einstellungen()
    daten.setdefault('posen', {})[name] = [int(w) for w in winkel]
    daten.setdefault('ki_darf_arm_bewegen', False)
    with open(POSEN_DATEI, 'w') as f:
        f.write('# Arm-Posen (Grad je Servo 1-6). Gespeichert ueber das Panel (Seite Arm).\n'
                '# ki_darf_arm_bewegen: true = KI darf bei unklarer LiDAR-Meldung im STAND\n'
                '# kurz in "pruefblick" schauen und danach zurueck in "fahrstellung".\n')
        yaml.safe_dump(daten, f, allow_unicode=True, sort_keys=False)


class Arm:
    """Schickt Posen an den Arm. Braucht einen laufenden ROS-Node."""

    def __init__(self, node):
        self.node = node
        self.letzte = None
        try:
            from rosidl_runtime_py.utilities import get_message
            self.typ = get_message('arm_msgs/msg/ArmJoints')
            self.pub = node.create_publisher(self.typ, '/arm6_joints', 10)
            self.echt = True
        except Exception:
            # Paket arm_msgs nicht gefunden (z. B. Simulation ohne Yahboom-Workspace):
            # dann als Text auf ein Test-Topic senden, damit man sieht, was passiert waere.
            from std_msgs.msg import String
            self.typ = String
            self.pub = node.create_publisher(String, '/arm6_joints_sim', 10)
            self.echt = False

    def fahre(self, winkel, zeit_ms=1500):
        """Rueckgabe: Fehlertext oder None."""
        winkel = [int(round(w)) for w in winkel]
        fehler = pruefe(winkel, zeit_ms)
        if fehler:
            return fehler
        if self.echt:
            msg = self.typ()
            for i, w in enumerate(winkel, start=1):
                setattr(msg, f'joint{i}', w)
            msg.time = int(zeit_ms)
        else:
            msg = self.typ(data=f'{winkel} {zeit_ms} ms')
        self.pub.publish(msg)
        self.letzte = winkel
        return None
