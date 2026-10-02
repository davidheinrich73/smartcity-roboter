#!/usr/bin/env python3
# Ablaeufe der KI, die mehrere Schritte brauchen (ohne ROS, damit testbar):
#   Zebrastreifen  - anhalten, umschauen (Arm links/rechts), warten bis frei, weiter
#   Wenden         - Einbahnstrasse von der falschen Seite: auf der Stelle umdrehen
#   Aufheben       - kleines Hindernis greifen und neben die Strasse legen
#
# Jeder Ablauf hat schritt(jetzt, w) -> Befehl (dict) oder None (= fertig, Ergebnis in .ergebnis).
# Befehl: {'aktion': 'stopp' | 'manoever', 'grund': Text, 'arm': None | {'pose', 'greifer'},
#          'manoever': {'lin', 'quer', 'dreh', 'text'}}
# w = Wahrnehmung (siehe ki/entscheider.py).
import math

import numpy as np

LEBEWESEN = {'Person', 'Hund', 'Katze', 'Pferd', 'Vogel', 'Schaf', 'Kuh', 'Teddy'}


def befehl(aktion, grund, arm=None, manoever=None):
    return {'aktion': aktion, 'faktor': 0.0 if aktion == 'stopp' else 1.0, 'grund': grund,
            'arm': arm, 'manoever': manoever}


class Ablauf:
    name = ''
    braucht_arm = False     # bewegt dieser Ablauf den Arm? (dann nur, solange das Fahrprogramm faehrt)

    def __init__(self, jetzt, c):
        self.c = c
        self.phase, self.seit = None, jetzt
        self.ergebnis = None             # 'ok' | 'fehler'
        self.ereignisse = []             # (text, art)
        self.arm_bewegt = False          # steht der Arm evtl. nicht in Fahrstellung?

    def _phase(self, jetzt, phase):
        self.phase, self.seit = phase, jetzt

    def dauer(self, jetzt):
        return jetzt - self.seit


class Zebrastreifen(Ablauf):
    """Vor dem Zebrastreifen anhalten, umschauen, erst weiterfahren wenn niemand kommt."""
    name = 'zebra'

    def __init__(self, jetzt, c, posen, abstand=0.3):
        super().__init__(jetzt, c)
        self.mit_arm = bool(posen.get('blick_links') and posen.get('blick_rechts') and posen.get('fahrstellung'))
        self.braucht_arm = self.mit_arm
        self.abstand = abstand      # Zebrastreifen so weit voraus (m), gemessen beim Anhalten
        self.jemand = False
        self.runde = 0
        self._phase(jetzt, 'halt')

    def _beobachten(self, w):
        # Personen/Tiere irgendwo im Bild (nicht nur im Weg) ...
        if any(o['name'] in LEBEWESEN for o in w.get('objekte', [])):
            self.jemand = 'Person gesehen'
        # ... oder der LiDAR sieht etwas AUF oder direkt NEBEN dem Zebrastreifen (wartender Fussgaenger).
        # Nur dieser Bereich zaehlt, damit Haeuser und Waende nicht als Fussgaenger gelten.
        xy = w.get('lidar_xy')
        if xy is not None:
            xy = np.asarray(xy).reshape(-1, 2)
            zone = ((xy[:, 0] > self.abstand - 0.10) & (xy[:, 0] < self.abstand + self.c['zebra_tiefe'])
                    & (np.abs(xy[:, 1]) < self.c['zebra_seite']))
            if zone.any():
                x, y = xy[zone][np.argmin(xy[zone][:, 0])]
                seite = 'links' if y > 0 else 'rechts'
                self.jemand = f'etwas am Zebrastreifen, {abs(y) * 100:.0f} cm {seite} (LiDAR)'
        elif w.get('lidar_breit') is not None and w['lidar_breit'] < self.c['zebra_lidar']:
            self.jemand = f"etwas {w['lidar_breit'] * 100:.0f} cm vor mir (LiDAR)"

    def schritt(self, jetzt, w):
        c, d = self.c, self.dauer(jetzt)
        arm_ruhig = d > c['arm_zeit']
        if self.phase == 'halt':
            if d >= 0.5:
                self.jemand = False
                if self.mit_arm:
                    self.arm_bewegt = True
                    self._phase(jetzt, 'links')
                    return befehl('stopp', 'Zebrastreifen: schaue nach links', arm={'pose': 'blick_links'})
                self._phase(jetzt, 'geradeaus')
            return befehl('stopp', 'Zebrastreifen: anhalten')
        if self.phase in ('links', 'rechts', 'geradeaus'):
            if arm_ruhig or self.phase == 'geradeaus':
                self._beobachten(w)
            if self.phase == 'links' and d >= c['arm_zeit'] + c['zebra_schauen']:
                self._phase(jetzt, 'rechts')
                return befehl('stopp', 'Zebrastreifen: schaue nach rechts', arm={'pose': 'blick_rechts'})
            if self.phase == 'rechts' and d >= c['arm_zeit'] + c['zebra_schauen']:
                self._phase(jetzt, 'mitte')
                return befehl('stopp', 'Zebrastreifen: Kamera zurueck', arm={'pose': 'fahrstellung'})
            if self.phase == 'geradeaus' and d >= c['zebra_schauen'] + 1.0:
                return self._entscheiden(jetzt)
            return befehl('stopp', f'Zebrastreifen: schaue ({self.phase})')
        if self.phase == 'mitte':
            if d >= c['arm_zeit']:
                self.arm_bewegt = False
                self._beobachten(w)
                return self._entscheiden(jetzt)
            return befehl('stopp', 'Zebrastreifen: Kamera zurueck')
        if self.phase == 'warten':
            if d >= c['zebra_warten']:
                self._phase(jetzt, 'halt')
                self.seit = jetzt - 0.5
            return befehl('stopp', f'Zebrastreifen: warte ({self.grund_warten})')
        return None

    def _entscheiden(self, jetzt):
        if self.jemand:
            self.runde += 1
            self.grund_warten = self.jemand
            self.ereignisse.append((f'Zebrastreifen: {self.jemand} -> warte', 'stopp'))
            self._phase(jetzt, 'warten')
            return befehl('stopp', f'Zebrastreifen: {self.jemand} -> warte')
        self.ereignisse.append(('Zebrastreifen frei -> weiter', 'fahren'))
        self.ergebnis = 'ok'
        return None


class Umschauen(Ablauf):
    """Kurz anhalten und mit der Kamera (Arm) nach links und rechts schauen, z. B. fuer die Karte."""
    name = 'umschauen'
    braucht_arm = True

    def __init__(self, jetzt, c, grund='Umschauen'):
        super().__init__(jetzt, c)
        self.grund = grund
        self._phase(jetzt, 'halt')

    def schritt(self, jetzt, w):
        c, d = self.c, self.dauer(jetzt)
        schauen = c['arm_zeit'] + c['zebra_schauen']
        # Phase: (so lange, was gerade passiert, naechste Phase, Arm dafuer)
        folge = {'halt': (0.5, 'anhalten', 'links', {'pose': 'blick_links'}),
                 'links': (schauen, 'schaue nach links', 'rechts', {'pose': 'blick_rechts'}),
                 'rechts': (schauen, 'schaue nach rechts', 'mitte', {'pose': 'fahrstellung'}),
                 'mitte': (c['arm_zeit'], 'Kamera zurueck', None, None)}
        warte, text, naechste, arm = folge[self.phase]
        if d < warte:
            return befehl('stopp', f'{self.grund}: {text}')
        if naechste is None:
            self.arm_bewegt = False
            self.ergebnis = 'ok'
            return None
        self.arm_bewegt = True
        self._phase(jetzt, naechste)
        return befehl('stopp', f'{self.grund}: {folge[naechste][1]}', arm=arm)


class Wenden(Ablauf):
    """Auf der Stelle umdrehen (Einbahnstrasse von der falschen Seite), bis die Linie wieder vorne ist."""
    name = 'wenden'

    def __init__(self, jetzt, c, gier):
        super().__init__(jetzt, c)
        self.gier0 = gier
        self._phase(jetzt, 'halt')

    def schritt(self, jetzt, w):
        c, d = self.c, self.dauer(jetzt)
        if self.phase == 'halt':
            if d >= 0.5:
                self._phase(jetzt, 'drehen')
            return befehl('stopp', 'Einfahrt verboten -> wende')
        gedreht = abs(w.get('gier', self.gier0) - self.gier0)
        linie = w.get('linie') or {}
        # Linie wieder vorne: Roboter steht auf ihr und schaut (fast) in ihre Richtung.
        # Zuerst nach dem Linien-Gedaechtnis (kennt auch das Stueck direkt unter/hinter dem Roboter),
        # sonst nach dem Kamerabild.

        def passt(quer, kurs):
            return quer is not None and kurs is not None and abs(quer) < 0.06 and abs(kurs) < math.radians(20)
        linie_vorne = passt(linie.get('quer'), linie.get('kurs')) or (
            linie.get('kamera') and passt(linie.get('kamera_quer'), linie.get('kamera_kurs')))
        if (gedreht >= math.radians(150) and linie_vorne) or gedreht >= math.radians(215):
            self.ereignisse.append((f'Gewendet ({math.degrees(gedreht):.0f} Grad)', 'fahren'))
            self.ergebnis = 'ok'
            return None
        if d > c['wenden_max_zeit']:
            self.ereignisse.append(('Wenden hat nicht geklappt (Zeit abgelaufen)', 'stopp'))
            self.ergebnis = 'fehler'
            return None
        return befehl('manoever', f'Wende ({math.degrees(gedreht):.0f} von ca. 180 Grad)',
                      manoever={'lin': 0.0, 'quer': 0.0, 'dreh': c['wenden_dreh'], 'text': 'wenden'})


class Aufheben(Ablauf):
    """Kleines Hindernis greifen und neben die Strasse legen. Posen: greifen, greifen_hoch, ablegen."""
    name = 'aufheben'
    braucht_arm = True

    def __init__(self, jetzt, c, objekt):
        super().__init__(jetzt, c)
        self.objekt = dict(objekt)
        self.zuletzt_gesehen = jetzt
        self.ruhig = 0
        self._phase(jetzt, 'ausrichten')
        self.ereignisse.append((f"Hindernis {objekt['breite'] * 100:.0f} cm breit -> hebe es auf", 'info'))

    def schritt(self, jetzt, w):
        c, d = self.c, self.dauer(jetzt)
        obj = w.get('objekt') if w.get('neues_bild') else None
        if self.phase == 'ausrichten':
            if obj:
                self.objekt, self.zuletzt_gesehen = dict(obj), jetzt
                fehler_vor = obj['vor'] - c['ausricht_abstand']
                if abs(obj['seite']) < 0.01 and abs(fehler_vor) < 0.02:
                    self.ruhig += 1
                else:
                    self.ruhig = 0
            elif jetzt - self.zuletzt_gesehen > 2.0:
                return self._fehler('Hindernis nicht mehr zu sehen')
            if self.ruhig >= 3:
                self.strecke = self.objekt['vor'] + self.objekt['breite'] / 2 - c['greif_abstand']
                self._phase(jetzt, 'anfahren')
                return befehl('stopp', 'Aufheben: ausgerichtet')
            if d > 15:
                return self._fehler('Ausrichten dauert zu lange')
            o = self.objekt
            quer = max(-0.04, min(0.04, 0.8 * o['seite']))
            lin = max(-0.03, min(0.03, 0.5 * (o['vor'] - c['ausricht_abstand'])))
            return befehl('manoever', f"Aufheben: ausrichten (seitlich {o['seite'] * 100:+.1f} cm)",
                          manoever={'lin': lin, 'quer': quer, 'dreh': 0.0, 'text': 'zum Greifen ausrichten'})
        if self.phase == 'anfahren':
            dauer_soll = max(0.0, self.strecke) / c['anfahr_tempo']
            if d >= dauer_soll:
                self.arm_bewegt = True
                self._phase(jetzt, 'oeffnen')
                return befehl('stopp', 'Aufheben: Greifer auf', arm={'pose': 'greifen', 'greifer': 'auf'})
            return befehl('manoever', f'Aufheben: heranfahren ({self.strecke * 100:.0f} cm)',
                          manoever={'lin': c['anfahr_tempo'], 'quer': 0.0, 'dreh': 0.0, 'text': 'heranfahren'})
        schritte = [('oeffnen', c['arm_zeit'] * 1.3, 'zu', {'pose': 'greifen', 'greifer': 'zu'}, 'Greifer zu'),
                    ('zu', 1.2, 'hoch', {'pose': 'greifen_hoch', 'greifer': 'zu'}, 'Arm hoch'),
                    ('hoch', c['arm_zeit'], 'ablegen', {'pose': 'ablegen', 'greifer': 'zu'}, 'zur Seite'),
                    ('ablegen', c['arm_zeit'] + 0.3, 'loslassen', {'pose': 'ablegen', 'greifer': 'auf'}, 'loslassen'),
                    ('loslassen', 1.0, 'zurueck', {'pose': 'fahrstellung'}, 'Arm in Fahrstellung'),
                    ('zurueck', c['arm_zeit'], 'pruefen', None, 'pruefe, ob der Weg frei ist')]
        for phase, warte, naechste, arm, text in schritte:
            if self.phase == phase:
                if d >= warte:
                    self._phase(jetzt, naechste)
                    if naechste == 'pruefen':
                        self.arm_bewegt = False
                        self.noch_da = 0
                    return befehl('stopp', 'Aufheben: ' + text, arm=arm)
                return befehl('stopp', 'Aufheben: ' + text)
        if self.phase == 'pruefen':
            if obj and obj['vor'] < 0.35:
                self.noch_da += 1
            if d >= 1.5:
                if self.noch_da >= 2:
                    return self._fehler('Hindernis liegt noch da')
                self.ereignisse.append(('Hindernis beiseitegelegt -> weiter', 'fahren'))
                self.ergebnis = 'ok'
                return None
            return befehl('stopp', 'Aufheben: pruefe, ob der Weg frei ist')
        return None

    def _fehler(self, grund):
        self.ereignisse.append((f'Aufheben hat nicht geklappt: {grund} -> warte', 'stopp'))
        self.ergebnis = 'fehler'
        return None
