#!/usr/bin/env python3
# Prueft die Entscheidungen der KI-Zentrale mit ausgedachten Wahrnehmungen (kein Roboter noetig).
# Aufruf: python3 tests/test_entscheider.py
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ki'))
from entscheider import Entscheider  # noqa: E402

fehler = 0


class Lauf:
    """Spielt Bilder im Abstand von 0,1 s ab."""

    def __init__(self, **kw):
        self.e = Entscheider(**kw)
        self.t = 100.0
        self.nr = 0

    def schritt(self, n=1, szenario='normal', **w):
        befehl = None
        for _ in range(n):
            self.t += 0.1
            self.nr += 1
            daten = {'bild_nr': self.nr, 'bild_zeit': self.t, 'objekte': [], 'ampel': None,
                     'stoppschild': False, 'lidar': float('inf'), 'tiefe_hindernis': False}
            daten.update(w)
            befehl = self.e.update(self.t, daten, szenario)
        return befehl

    def bis(self, arm, max_schritte=40, **w):
        """Schritte machen, bis ein Arm-Befehl kommt (oder max_schritte erreicht)."""
        for _ in range(max_schritte):
            befehl = self.schritt(1, **w)
            if befehl['arm'] == arm:
                return befehl
        return befehl


def pruefe(name, befehl, aktion, arm='egal'):
    global fehler
    ok = befehl['aktion'] == aktion and (arm == 'egal' or befehl['arm'] == arm)
    fehler += not ok
    print(f"{'OK  ' if ok else 'FEHLER'} {name}: {befehl['aktion']} ({befehl['grund']}) arm={befehl['arm']}")


l = Lauf()
pruefe('freie Fahrt', l.schritt(3), 'fahren')
pruefe('Rot 1 Bild (noch nicht sicher)', l.schritt(1, ampel='rot'), 'fahren')
pruefe('Rot 2 Bilder -> Stopp', l.schritt(1, ampel='rot'), 'stopp')
pruefe('Ampel kurz verdeckt -> weiter warten', l.schritt(5, ampel=None), 'stopp')
pruefe('Gruen -> weiter', l.schritt(2, ampel='gruen'), 'fahren')
pruefe('Gelb -> Stopp', l.schritt(2, ampel='gelb'), 'stopp')
pruefe('Ampel weg, nach Timeout -> weiter', l.schritt(45, ampel=None), 'fahren')

l = Lauf()
pruefe('Stoppschild -> halten', l.schritt(2, stoppschild=True), 'stopp')
pruefe('nach 3 s weiter, obwohl Schild noch sichtbar', l.schritt(31, stoppschild=True), 'fahren')
pruefe('Schild lange sichtbar (langsam vorbei) -> kein 2. Halt', l.schritt(80, stoppschild=True), 'fahren')
l.schritt(25)
pruefe('naechstes Schild (nach 2,5 s ohne Schild) -> wieder halten', l.schritt(2, stoppschild=True), 'stopp')
pruefe('Einsatz: Rot wird ueberfahren', Lauf().schritt(3, szenario='einsatz', ampel='rot'), 'fahren')

l = Lauf()
pruefe('LiDAR meldet, KI-Bild aelter -> langsam (prueft)',
       l.e.update(l.t + 0.05, {'bild_nr': 0, 'bild_zeit': l.t, 'lidar': 0.3}, 'normal'), 'langsam')
pruefe('LiDAR + Kamera-KI sieht Person -> Stopp',
       l.schritt(1, lidar=0.3, objekte=[{'name': 'Person', 'im_weg': True}]), 'stopp')
pruefe('Person weg, LiDAR frei -> noch kurz warten', l.schritt(1), 'stopp')
pruefe('nach 1 s frei -> weiter', l.schritt(10), 'fahren')
pruefe('LiDAR + Tiefenkamera -> Stopp', l.schritt(1, lidar=0.3, tiefe_hindernis=True), 'stopp')
l = Lauf()
pruefe('LiDAR ohne Bestaetigung -> langsam', l.schritt(3, lidar=0.3), 'langsam')
pruefe('LiDAR ohne Bestaetigung, aber naeher als 20 cm -> halten', l.schritt(1, lidar=0.18), 'stopp')
pruefe('NOTBREMSE unter 12 cm, ohne KI', l.schritt(1, lidar=0.10), 'stopp')
pruefe('kein LiDAR -> Stopp', l.schritt(1, lidar=None), 'stopp')
l = Lauf()
l.schritt(1)
pruefe('KI-Bild veraltet -> Stopp',
       l.e.update(l.t + 3.0, {'bild_nr': l.nr, 'bild_zeit': l.t, 'lidar': float('inf')}, 'normal'), 'stopp')
pruefe('Einsatz: Person im Weg -> trotzdem Stopp',
       Lauf().schritt(1, szenario='einsatz', objekte=[{'name': 'Person', 'im_weg': True}]), 'stopp')

# Arm-Blick (nur wenn erlaubt und die Pose 'pruefblick' eingelernt ist)
PRUEF = {'fahrstellung': [90, 120, 10, 20, 90, 30], 'pruefblick': [90, 100, 20, 30, 90, 30]}
l = Lauf(arm_erlaubt=True, posen=PRUEF)
pruefe('unklar (1,5 s) -> Arm in Pruefblick', l.bis('pruefblick', lidar=0.3), 'stopp', 'pruefblick')
t0 = l.t
pruefe('Arm faehrt hin', l.schritt(10, lidar=0.3), 'stopp')
b = l.bis('fahrstellung', lidar=0.3)
pruefe('nach Schauen -> Arm zurueck', b, 'stopp', 'fahrstellung')
print(f'     (Arm-Blick dauerte {l.t - t0:.1f} s bis zum Zurueckfahren)')
pruefe('Arm noch unterwegs -> Stopp', l.schritt(5, lidar=0.3), 'stopp')
pruefe('Arm zurueck, nichts gefunden -> langsam', l.schritt(12, lidar=0.3), 'langsam')
pruefe('kein zweites Umschauen fuer dieselbe Meldung', l.schritt(30, lidar=0.3), 'langsam', None)
l = Lauf(arm_erlaubt=True, posen=PRUEF)
l.bis('pruefblick', lidar=0.3)
l.schritt(5, lidar=0.3)
pruefe('Notbremse waehrend Arm-Blick', l.schritt(1, lidar=0.1), 'stopp')
pruefe('danach frei -> Arm zuerst zurueck', l.schritt(1, lidar=1.0), 'stopp', 'fahrstellung')
pruefe('waehrend Arm zurueckfaehrt -> noch Stopp', l.schritt(5, lidar=1.0), 'stopp')
pruefe('danach weiter', l.schritt(20, lidar=1.0), 'fahren')
pruefe('ohne Erlaubnis kein Arm', Lauf().schritt(40, lidar=0.3), 'langsam', None)
pruefe('ohne Pose "pruefblick" kein Arm-Blick', Lauf(arm_erlaubt=True, posen={'fahrstellung': PRUEF['fahrstellung']})
       .schritt(40, lidar=0.3), 'langsam', None)

# ---------------- Zebrastreifen ----------------
POSEN = {'fahrstellung': [90, 120, 10, 20, 90, 30], 'pruefblick': [90, 100, 20, 30, 90, 30],
         'blick_links': [135, 120, 10, 20, 90, 30], 'blick_rechts': [45, 120, 10, 20, 90, 30],
         'greifen': [90, 60, 60, 40, 90, 30], 'greifen_hoch': [90, 120, 40, 40, 90, 30],
         'ablegen': [170, 70, 60, 40, 90, 30]}


def arm_befehle(lauf, n, **w):
    """n Schritte, sammelt alle Arm-Befehle."""
    arme, letzter = [], None
    for _ in range(n):
        letzter = lauf.schritt(1, **w)
        if letzter['arm']:
            arme.append(letzter['arm']['pose'] if isinstance(letzter['arm'], dict) else letzter['arm'])
    return arme, letzter


l = Lauf(arm_erlaubt=True, posen=POSEN)
pruefe('Zebrastreifen 50 cm -> noch fahren', l.schritt(2, zebra={'abstand': 0.5}), 'fahren')
pruefe('Zebrastreifen 30 cm -> anhalten', l.schritt(1, zebra={'abstand': 0.3}), 'stopp')
arme, b = arm_befehle(l, 80, zebra={'abstand': 0.3})
ok = arme == ['blick_links', 'blick_rechts', 'fahrstellung']
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} umschauen links, rechts, zurueck: {arme}")
pruefe('niemand da -> weiter', b, 'fahren')
pruefe('derselbe Zebrastreifen -> kein zweiter Halt', l.schritt(20, zebra={'abstand': 0.2}), 'fahren')
l = Lauf(arm_erlaubt=True, posen=POSEN)
l.schritt(1, zebra={'abstand': 0.3})
arme, b = arm_befehle(l, 80, zebra={'abstand': 0.3}, objekte=[{'name': 'Person', 'im_weg': False}])
pruefe('Person gesehen -> wartet weiter', b, 'stopp')
arme, b = arm_befehle(l, 120, zebra={'abstand': 0.3})
pruefe('Person weg -> weiter', b, 'fahren')
l = Lauf()
l.schritt(1, zebra={'abstand': 0.3})
arme, b = arm_befehle(l, 30, zebra={'abstand': 0.3}, lidar_breit=0.35)
ok = arme == [] and b['aktion'] == 'stopp'
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} ohne Arm-Freigabe: kein Arm, etwas auf dem Zebrastreifen -> wartet ({b['grund']})")
pruefe('ohne Arm, frei -> weiter', l.schritt(80, zebra={'abstand': 0.3}), 'fahren')
# LiDAR-Punkte: nur der Bereich am Zebrastreifen zaehlt (Haus/Wand daneben nicht)
import numpy as np  # noqa: E402
wand = np.array([[0.3 + 0.02 * k, -0.40] for k in range(20)])       # 40 cm rechts: Wand/Haus
fussgaenger = np.array([[0.36, -0.20], [0.37, -0.21]])               # wartet rechts am Zebrastreifen
l = Lauf(arm_erlaubt=True, posen=POSEN)
l.schritt(1, zebra={'abstand': 0.3}, lidar_xy=wand)
arme, b = arm_befehle(l, 80, zebra={'abstand': 0.3}, lidar_xy=wand)
pruefe('nur Wand neben der Strasse -> weiter', b, 'fahren')
l = Lauf(arm_erlaubt=True, posen=POSEN)
l.schritt(1, zebra={'abstand': 0.3}, lidar_xy=np.vstack([wand, fussgaenger]))
arme, b = arm_befehle(l, 80, zebra={'abstand': 0.3}, lidar_xy=np.vstack([wand, fussgaenger]))
pruefe('Fussgaenger am Zebrastreifen (LiDAR) -> wartet', b, 'stopp')
arme, b = arm_befehle(l, 120, zebra={'abstand': 0.3}, lidar_xy=wand)
pruefe('Fussgaenger weg -> weiter', b, 'fahren')

# ---------------- Einbahnstrasse ----------------
l = Lauf()
pruefe('Einfahrt verboten 1 Bild -> noch nichts', l.schritt(1, einfahrt_verboten=True), 'fahren')
pruefe('Einfahrt verboten 2 Bilder -> anhalten', l.schritt(1, einfahrt_verboten=True), 'stopp')
b = l.schritt(10, gier=0.0)
ok = b['aktion'] == 'manoever' and b['manoever']['dreh'] > 0
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} dreht auf der Stelle: {b['grund']}")
VORNE = {'kamera': True, 'kamera_quer': 0.01, 'kamera_kurs': 0.05}
pruefe('nach 100 Grad noch nicht fertig', l.schritt(1, gier=math.radians(100), linie=VORNE), 'manoever')
pruefe('160 Grad, Linie noch schraeg (40 Grad) -> weiter drehen',
       l.schritt(1, gier=math.radians(160), linie=dict(VORNE, kamera_kurs=math.radians(40))), 'manoever')
pruefe('170 Grad + Linie mittig vorne -> fertig, faehrt', l.schritt(1, gier=math.radians(170), linie=VORNE), 'fahren')
l = Lauf()
l.schritt(2, einfahrt_verboten=True)
l.schritt(10, gier=0.0)
pruefe('Gedaechtnis: 170 Grad, Linie unter dem Roboter gerade -> fertig',
       l.schritt(1, gier=math.radians(170), linie={'kamera': False, 'quer': 0.01, 'kurs': 0.1}), 'fahren')
pruefe('Schild noch sichtbar -> nicht nochmal wenden', l.schritt(5, gier=math.radians(170), einfahrt_verboten=True), 'fahren')

# ---------------- Aufheben ----------------
WUERFEL = {'vor': 0.31, 'seite': 0.02, 'breite': 0.045, 'hoehe': 0.045, 'quelle': 'tiefe'}
l = Lauf(arm_erlaubt=True, posen=POSEN)
b = l.schritt(1, objekt=WUERFEL)
ok = b['aktion'] == 'manoever' and b['manoever']['quer'] > 0
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} Wuerfel (Tiefenkamera) -> richtet sich seitlich aus: {b['grund']}")
mittig = dict(WUERFEL, seite=0.0, vor=0.30)
b = l.schritt(4, objekt=mittig)
pruefe('ausgerichtet -> faehrt heran', l.schritt(1, objekt=mittig), 'manoever')
arme, b = arm_befehle(l, 200)
soll = ['greifen', 'greifen', 'greifen_hoch', 'ablegen', 'ablegen', 'fahrstellung']
ok = arme == soll
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} Greif-Ablauf: {arme}")
pruefe('Weg frei -> weiter', b, 'fahren')
l = Lauf(arm_erlaubt=True, posen=POSEN)
l.schritt(5, objekt=mittig)
arme, b = arm_befehle(l, 200, objekt=mittig)
pruefe('Wuerfel liegt nach dem Greifen noch da -> wartet', b, 'stopp')
pruefe('kein zweiter Versuch, solange er da liegt', l.schritt(10, objekt=mittig), 'stopp')
l = Lauf(arm_erlaubt=True, posen=POSEN)
b = l.schritt(3, objekt=WUERFEL, objekte=[{'name': 'Person', 'im_weg': False}])
pruefe('Person in Sicht -> nie greifen, nur warten', b, 'stopp')
l = Lauf(arm_erlaubt=True, posen=POSEN)
pruefe('zu breit (20 cm) -> warten statt greifen', l.schritt(3, objekt=dict(WUERFEL, breite=0.2)), 'stopp')
l = Lauf(arm_erlaubt=False, posen=POSEN)
pruefe('ohne Arm-Freigabe -> warten', l.schritt(3, objekt=WUERFEL), 'stopp')

# ---------------- Umschauen fuer die Karte ----------------
l = Lauf(arm_erlaubt=True, posen=POSEN)
l.schritt(2)
print('     ', l.e.kommando(l.t, 'umschauen'))
arme, b = arm_befehle(l, 90)
ok = arme == ['blick_links', 'blick_rechts', 'fahrstellung'] and b['aktion'] == 'fahren'
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} Umschauen auf Wunsch: {arme}, danach {b['aktion']}")
l = Lauf()
ok = 'geht nicht' in l.e.kommando(l.t, 'umschauen')
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} Umschauen ohne Arm-Freigabe wird abgelehnt")

# ---------------- Aufheben: Sicherheitsregeln ----------------
l = Lauf(arm_erlaubt=True, posen=POSEN)
pruefe('Gegenstand am Strassenrand (12 cm neben der Linie) -> nicht greifen, warten',
       l.schritt(3, objekt=dict(WUERFEL, quer=0.12)), 'stopp', None)
l = Lauf(arm_erlaubt=True, posen=POSEN)
l.schritt(1, zebra={'abstand': 0.6})
b = l.schritt(2, objekt=WUERFEL)
ok = b['aktion'] == 'stopp' and l.e.ablauf is None
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} kurz nach Zebrastreifen -> nichts aufheben (Fussgaenger!): {b['grund']}")

# ---------------- STOPP / TEST: KI bewegt den Arm nie ----------------
l = Lauf(arm_erlaubt=True, posen=POSEN)
mittig = dict(WUERFEL, seite=0.0, vor=0.30)
l.schritt(1, objekt=mittig)
arme, b = arm_befehle(l, 60, objekt=mittig)       # ausrichten, heranfahren, Greifer auf
ok_vorher = 'greifen' in arme
l.e.fahrt_aktiv = False                          # STOPP gedrueckt
arme, b = arm_befehle(l, 60, objekt=mittig)
ok = ok_vorher and arme == [] and l.e.ablauf is None
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} STOPP waehrend Aufheben: Ablauf abgebrochen, kein Arm-Befehl mehr ({b['grund']})")
l.e.fahrt_aktiv = True                           # wieder START
b = l.schritt(1)
ok = b['arm'] == 'fahrstellung'
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} nach neuem START faehrt der Arm zuerst in die Fahrstellung: {b['grund']}")
l = Lauf(arm_erlaubt=True, posen=POSEN)
l.e.fahrt_aktiv = False                          # nur TEST
arme, b = arm_befehle(l, 80, zebra={'abstand': 0.3})
ok = arme == [] and b['aktion'] == 'stopp' or arme == []
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} TEST-Modus am Zebrastreifen: kein Arm ({b['grund']})")
ok = 'nur waehrend der Fahrt' in l.e.kommando(l.t, 'umschauen')
fehler += not ok
print(f"{'OK  ' if ok else 'FEHLER'} Umschauen im TEST-Modus abgelehnt")

print()
print('Alle Tests bestanden.' if not fehler else f'{fehler} Test(s) fehlgeschlagen.')
sys.exit(1 if fehler else 0)
