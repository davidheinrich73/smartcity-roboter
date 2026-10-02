#!/usr/bin/env python3
# Prueft die Entscheidungen der KI-Zentrale mit ausgedachten Wahrnehmungen (kein Roboter noetig).
# Aufruf: python3 tests/test_entscheider.py
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
pruefe('LiDAR ohne Bestaetigung (Haus am Rand) -> langsam', l.schritt(3, lidar=0.3), 'langsam')
pruefe('NOTBREMSE unter 12 cm, ohne KI', l.schritt(1, lidar=0.10), 'stopp')
pruefe('kein LiDAR -> Stopp', l.schritt(1, lidar=None), 'stopp')
l = Lauf()
l.schritt(1)
pruefe('KI-Bild veraltet -> Stopp',
       l.e.update(l.t + 3.0, {'bild_nr': l.nr, 'bild_zeit': l.t, 'lidar': float('inf')}, 'normal'), 'stopp')
pruefe('Einsatz: Person im Weg -> trotzdem Stopp',
       Lauf().schritt(1, szenario='einsatz', objekte=[{'name': 'Person', 'im_weg': True}]), 'stopp')

# Arm-Blick (nur wenn erlaubt)
l = Lauf(arm_erlaubt=True)
pruefe('unklar (1,5 s) -> Arm in Pruefblick', l.bis('pruefblick', lidar=0.3), 'stopp', 'pruefblick')
t0 = l.t
pruefe('Arm faehrt hin', l.schritt(10, lidar=0.3), 'stopp')
b = l.bis('fahrstellung', lidar=0.3)
pruefe('nach Schauen -> Arm zurueck', b, 'stopp', 'fahrstellung')
print(f'     (Arm-Blick dauerte {l.t - t0:.1f} s bis zum Zurueckfahren)')
pruefe('Arm noch unterwegs -> Stopp', l.schritt(5, lidar=0.3), 'stopp')
pruefe('Arm zurueck, nichts gefunden -> langsam', l.schritt(12, lidar=0.3), 'langsam')
pruefe('kein zweites Umschauen fuer dieselbe Meldung', l.schritt(30, lidar=0.3), 'langsam', None)
l = Lauf(arm_erlaubt=True)
l.bis('pruefblick', lidar=0.3)
l.schritt(5, lidar=0.3)
pruefe('Notbremse waehrend Arm-Blick', l.schritt(1, lidar=0.1), 'stopp')
pruefe('danach frei -> Arm zuerst zurueck', l.schritt(1, lidar=1.0), 'stopp', 'fahrstellung')
pruefe('waehrend Arm zurueckfaehrt -> noch Stopp', l.schritt(5, lidar=1.0), 'stopp')
pruefe('danach weiter', l.schritt(20, lidar=1.0), 'fahren')
pruefe('ohne Erlaubnis kein Arm', Lauf().schritt(40, lidar=0.3), 'langsam', None)

print()
print('Alle Tests bestanden.' if not fehler else f'{fehler} Test(s) fehlgeschlagen.')
sys.exit(1 if fehler else 0)
