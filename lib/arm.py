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
# Standard-Posen (im Repository) und eigene, im Panel gespeicherte Posen (NICHT im Repository,
# damit "git pull" nie an gespeicherten Werten scheitert). Eigene Werte gelten vor Standardwerten.
STANDARD_DATEI = os.path.join(REPO, 'config', 'arm.yaml')
POSEN_DATEI = os.environ.get('SMARTCITY_ARM_YAML', os.path.join(REPO, 'config', 'lokal', 'arm.yaml'))
BLICK_DREHUNG = 45   # Grad, um die Servo 1 zum Umschauen gedreht wird (blick_links/blick_rechts)


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


def _lies(datei):
    try:
        import yaml
        with open(datei) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def lade_einstellungen():
    """Standard (config/arm.yaml) + eigene Werte (config/lokal/arm.yaml)."""
    daten = _lies(STANDARD_DATEI)
    eigene = _lies(POSEN_DATEI)
    posen = dict(daten.get('posen') or {})
    posen.update(eigene.get('posen') or {})
    daten.update(eigene)
    daten['posen'] = posen
    return daten


def lade_posen():
    """Alle Posen: {name: [6 Winkel]}. Fehlen blick_links/blick_rechts, werden sie aus der
    Fahrstellung berechnet (Servo 1 um BLICK_DREHUNG Grad gedreht)."""
    posen = {k: v for k, v in (lade_einstellungen().get('posen') or {}).items()}
    fahr = posen.get('fahrstellung')
    if fahr:
        for name, d in (('blick_links', BLICK_DREHUNG), ('blick_rechts', -BLICK_DREHUNG)):
            if not posen.get(name):
                posen[name] = [max(0, min(180, fahr[0] + d))] + list(fahr[1:])
    return posen


def _speichere(aenderung):
    import yaml
    eigene = _lies(POSEN_DATEI)
    aenderung(eigene)
    os.makedirs(os.path.dirname(POSEN_DATEI), exist_ok=True)
    with open(POSEN_DATEI, 'w') as f:
        f.write('# Eigene Arm-Posen dieses Roboters (gespeichert ueber das Panel, Seite Arm).\n'
                '# Nicht im Repository. Standardwerte: config/arm.yaml\n')
        yaml.safe_dump(eigene, f, allow_unicode=True, sort_keys=False)


def speichere_pose(name, winkel):
    _speichere(lambda d: d.setdefault('posen', {}).__setitem__(name, [int(w) for w in winkel]))


def speichere_einstellung(name, wert):
    _speichere(lambda d: d.__setitem__(name, wert))


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


class ArmBeobachter:
    """Hoert mit, wohin der Arm zuletzt geschickt wurde - egal von wem (KI, Panel, Gamepad).
    Damit wissen KI und Kartograf, wohin die Kamera (am Arm) gerade schaut."""

    def __init__(self, node):
        self.winkel, self.zeit, self.dauer = None, 0.0, 0.0
        from std_msgs.msg import String
        try:
            from rosidl_runtime_py.utilities import get_message
            typ = get_message('arm_msgs/msg/ArmJoints')
            node.create_subscription(typ, '/arm6_joints', self._echt, 10)
        except Exception:
            pass   # kein arm_msgs (Simulation)
        node.create_subscription(String, '/arm6_joints_sim', self._sim, 10)

    def _neu(self, winkel, dauer_ms):
        import time
        self.winkel, self.zeit, self.dauer = [float(w) for w in winkel], time.time(), float(dauer_ms) / 1000.0

    def _echt(self, m):
        self._neu([m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6], m.time)

    def _sim(self, m):
        import re
        t = re.match(r'\[([^\]]*)\]\s*(\d+)?', m.data)
        if t:
            self._neu([float(w) for w in t.group(1).split(',')], int(t.group(2) or 1000))

    def kamera(self, posen, jetzt):
        """(in_fahrstellung, gier_grad). gier_grad = Drehung von Servo 1 gegenueber der Fahrstellung,
        None = Kamera schaut irgendwohin (Arm bewegt sich oder andere Servos verstellt).
        Ohne Meldung seit dem Start: Fahrstellung angenommen (so startet der Roboter)."""
        fahr = posen.get('fahrstellung')
        if self.winkel is None or not fahr:
            return True, 0.0
        if jetzt - self.zeit < self.dauer + 0.3:
            return False, None                       # Arm faehrt noch
        gleich = [abs(self.winkel[i] - fahr[i]) <= 3 for i in range(5)]   # Greifer (6) egal
        if all(gleich):
            return True, 0.0
        if all(gleich[1:]):
            return False, self.winkel[0] - fahr[0]
        return False, None
