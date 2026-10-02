#!/usr/bin/env python3
# Testet, wann der Linienfolger faehrt und wann nicht - ohne ROS (mit Attrappen).
# Aufruf: python3 tests/test_line_follower.py
# Die Zusammenarbeit mit KI und Simulator testet: scripts/simulation.sh pruefen (braucht ROS 2)
import math
import os
import sys
import time
import types

HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HIER, 'attrappen'))
sys.path.insert(0, os.path.join(HIER, '..', 'line_follower'))
import line_follower as lf  # noqa: E402

fehler = 0


def check(name, ergebnis, faehrt):
    global fehler
    lin, ang, status = ergebnis
    ok = (abs(lin) > 0) == faehrt
    fehler += not ok
    print(f"{'OK  ' if ok else 'FEHLER'} {name}: {lin:.2f} m/s - {status}")


def scan(abstand, index_vorne=180):
    r = [2.0] * 360
    for k in range(index_vorne - 3, index_vorne + 4):
        r[k] = abstand
    return types.SimpleNamespace(ranges=r, angle_min=-math.pi, angle_increment=2 * math.pi / 360,
                                 range_min=0.05, range_max=12.0)


def neu():
    n = lf.LineFollower()
    jetzt = time.time()
    n.lenkung, n.bild_zeit = (0.15, 0.1), jetzt
    n.ki = (jetzt, {'aktion': 'fahren', 'faktor': 1.0, 'grund': 'Weg frei'})
    n.on_scan(scan(1.0), '/scan0', 0)
    return n


n = neu()
check('alles frei', n.entscheide(), True)
n.on_scan(scan(0.10), '/scan0', 0)
check('Notbremse 10 cm (eigene, ohne KI)', n.entscheide(), False)
n = neu()
n.on_scan(scan(0.10, index_vorne=0), '/scan0', 0)
check('10 cm HINTEN -> faehrt', n.entscheide(), True)
n = neu()
n.scans.clear()
check('kein LiDAR -> Stopp', n.entscheide(), False)
n = neu()
n.ki = (time.time() - 2, n.ki[1])
check('KI antwortet nicht -> Stopp', n.entscheide(), False)
n = neu()
n.ki = (time.time(), {'aktion': 'stopp', 'grund': 'Rote Ampel'})
check('KI sagt stopp', n.entscheide(), False)
n = neu()
n.ki = (time.time(), {'aktion': 'langsam', 'faktor': 0.5, 'grund': 'LiDAR unbestaetigt'})
lin, _, _ = n.entscheide()
check('KI sagt langsam (halbe Geschwindigkeit)', (lin, 0, f'{lin:.3f} m/s'), True)
if abs(lin - 0.075) > 0.001:
    fehler += 1
    print('FEHLER  langsam ist nicht halb so schnell')
n = neu()
n.lenkung = None
check('keine Linie -> Stopp', n.entscheide(), False)
n = neu()
n.bild_zeit = time.time() - 1
check('keine Kamerabilder -> Stopp', n.entscheide(), False)
n = neu()
n._p['ki_pflicht'] = False
n.ki = (0.0, None)
check('ohne KI-Pflicht (Test ohne KI)', n.entscheide(), True)
n = neu()
n.on_tempo(types.SimpleNamespace(data=0.9))
check('Tempo vom Panel wird auf 0,5 m/s begrenzt', (n.p('speed'), 0, f"speed={n.p('speed')}"), True)
if n.p('speed') != 0.5:
    fehler += 1
    print('FEHLER  Begrenzung greift nicht')

print('\nAlle Tests bestanden.' if not fehler else f'\n{fehler} Test(s) fehlgeschlagen.')
sys.exit(1 if fehler else 0)
