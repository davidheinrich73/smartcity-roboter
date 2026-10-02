#!/usr/bin/env python3
# Entscheidungslogik der KI-Zentrale (ohne ROS, damit testbar).
#
# Die KI bekommt laufend, was die "Sinne" melden, und entscheidet:
#   fahren   - Linienfolger darf fahren
#   langsam  - fahren, aber langsamer (z. B. LiDAR meldet etwas, Kamera sieht nichts)
#   stopp    - anhalten
# Jede Entscheidung hat einen Grund (Text), der im Panel angezeigt wird.
#
# Rangfolge (wichtigstes zuerst):
#   1. LiDAR sehr nah (Notbremse)      -> stopp, OHNE KI-Pruefung (Eigenschutz geht vor)
#   2. KI bekommt keine Bilder / kein LiDAR -> stopp (wer nichts sieht, faehrt nicht)
#   3. LiDAR meldet etwas im Pruefbereich -> KI prueft mit Kamera-KI und Tiefenkamera:
#        bestaetigt -> stopp, bis der Weg wieder frei ist
#        nicht bestaetigt -> langsam (ggf. vorher mit dem Arm genauer hinschauen),
#                            naeher als lidar_halt -> trotzdem stopp
#      Der LiDAR prueft nur den FAHRSCHLAUCH (Streifen entlang der Linie, siehe ki/zentrale.py).
#      Ist es klein, liegt mitten auf der Strasse, kein Lebewesen/Fahrzeug, kein Zebrastreifen in der Naehe
#      und der Arm freigegeben: AUFHEBEN und beiseitelegen
#   4. Tiefenkamera allein sieht etwas im Weg (z. B. Wuerfel unter der LiDAR-Ebene) -> stopp
#      bzw. aufheben (wie bei 3.)
#      KI sieht Person/Auto/... im Weg  -> stopp
#   5. Schild "Einfahrt verboten" (Einbahnstrasse) -> WENDEN
#   6. Zebrastreifen -> anhalten, umschauen, warten bis frei
#   7. Ampel rot/gelb -> stopp, bis gruen (oder Ampel nicht mehr zu sehen)
#   8. Stoppschild -> kurz halten, dann weiter
# Im Szenario "einsatz" (RTW) werden 7 und 8 uebergangen, alles andere gilt weiter.
# Mehrschrittige Ablaeufe (Zebrastreifen, Wenden, Aufheben) stehen in ki/ablaeufe.py.

import random

from ablaeufe import Zebrastreifen, Wenden, Aufheben, Umschauen, Ausweichen, LEBEWESEN

FAHRZEUGE = {'Auto', 'Bus', 'LKW', 'Motorrad', 'Fahrrad', 'Zug'}

STANDARD = {
    'notbremse_dist': 0.12,    # LiDAR naeher (m) -> sofort Stopp, ohne Rueckfrage
    'pruef_dist': 0.45,        # LiDAR naeher (m) -> KI prueft
    'lidar_halt': 0.20,        # LiDAR (im Fahrschlauch) naeher, aber nicht bestaetigt -> trotzdem halten
    'lidar_pflicht': True,     # ohne LiDAR-Daten nicht fahren
    'frei_zeit': 1.0,          # so lange muss der Weg frei sein, bis ein Hindernis-Stopp endet (s)
    'langsam_faktor': 0.5,     # Geschwindigkeit bei "langsam"
    'einsatz_faktor': 1.3,     # Geschwindigkeit im RTW-Einsatz
    'ki_max_alter': 1.5,       # aelteres KI-Bild = KI sieht nichts -> stopp (s)
    'ampel_bilder': 2,         # so viele KI-Bilder hintereinander dieselbe Farbe, bis sie gilt
    'ampel_timeout': 4.0,      # Rot/Gelb so lange nicht mehr gesehen -> weiter (falls Gruen unerkannt)
    'schild_bilder': 2,
    'schild_warten': 3.0,      # am Stoppschild so lange halten (s)
    'schild_weg': 2.0,         # nach dem Halten: Schild erst wieder beachten, wenn es so lange
                               # nicht mehr zu sehen war (sonst haelt er beim Vorbeifahren nochmal)
    'umschauen_nach': 1.5,     # LiDAR-Meldung so lange unbestaetigt -> mit dem Arm genauer schauen
    'arm_zeit': 1.5,           # so lange braucht der Arm fuer eine Bewegung (s)
    # Zebrastreifen
    'zebra_halt': 0.33,        # Zebrastreifen naeher als das (m, ab Robotermitte) -> anhalten
    'zebra_lidar': 0.5,        # ohne LiDAR-Punkte: etwas naeher als das (breit vorne) -> jemand kommt
    'zebra_seite': 0.30,       # LiDAR-Punkte bis so weit links/rechts der Fahrlinie gehoeren zum Zebrastreifen
    'zebra_tiefe': 0.25,       # ... und bis so weit hinter seiner vorderen Kante (m)
    'zebra_schauen': 0.8,      # so lange in jede Richtung schauen (s), nachdem der Arm steht
    'zebra_warten': 2.0,       # jemand da -> so lange warten, dann neu schauen (s)
    'zebra_weg': 2.0,          # nach dem Ueberqueren: erst wieder beachten, wenn so lange nicht gesehen
    # Einbahnstrasse
    'einfahrt_bilder': 2,      # so viele Bilder hintereinander "Einfahrt verboten" -> wenden
    'wenden_dreh': 0.8,        # Drehgeschwindigkeit beim Wenden (rad/s, + = links herum)
    'wenden_max_zeit': 14.0,
    # Hindernis aufheben (Posen 'greifen', 'greifen_hoch', 'ablegen' im Panel einlernen!)
    'tiefe_halt': 0.35,        # Tiefenkamera sieht etwas im Weg naeher als das (m) -> stopp
    'aufheben_max_breite': 0.08,
    'aufheben_max_hoehe': 0.12,
    'aufheben_max_quer': 0.08,  # nur aufheben, was so nah an der Linie liegt (m) - am Rand steht vielleicht jemand
    'zebra_kein_aufheben': 3.0,  # so lange nach einem gesehenen Zebrastreifen nichts aufheben (s): Fussgaenger!
    'ausricht_abstand': 0.30,  # so weit vor dem Gegenstand seitlich ausrichten (Kamera sieht ihn noch)
    'greif_abstand': 0.20,     # Robotermitte bis Mitte Gegenstand in der Pose 'greifen' (am Roboter messen!)
    'anfahr_tempo': 0.03,      # m/s beim letzten Stueck
    # Weg lange versperrt (z. B. zwei Roboter stehen sich gegenueber): nach einer ZUFAELLIGEN Wartezeit
    # weicht einer aus (zurueck + wenden). Zufall, damit nicht beide gleichzeitig ausweichen.
    'blockiert_min': 12.0,     # s
    'blockiert_max': 25.0,     # s
    'zurueck_strecke': 0.15,   # so weit zurueck (m), nur wenn hinten frei
    'zurueck_tempo': 0.05,     # m/s
    'hinten_frei': 0.25,       # LiDAR hinten mindestens so weit frei (m), sonst nicht zurueck
}

GREIF_POSEN = ('fahrstellung', 'greifen', 'greifen_hoch', 'ablegen')


class Entscheider:
    def __init__(self, cfg=None, arm_erlaubt=False, posen=None):
        self.c = dict(STANDARD)
        self.c.update(cfg or {})
        self.arm_freigabe = arm_erlaubt    # Einstellung: KI darf den Arm bewegen (Panel, Seite Arm)
        self.fahrt_aktiv = True            # faehrt das Fahrprogramm gerade? (setzt die Zentrale; TEST/aus = False)
        self.posen = posen or {}
        # Ablaeufe
        self.ablauf = None
        self.zebra_gesperrt, self.zebra_zuletzt = False, 0.0
        self.einfahrt_n, self.einfahrt_gesperrt_bis = 0, 0.0
        self.aufheben_gesperrt, self.weg_frei_seit = False, None
        self.umschauen_wunsch = None  # Zeit, zu der jemand (Kartograf/Panel) Umschauen gewuenscht hat
        self.blockiert_seit, self.blockiert_grenze = None, 0.0
        self.ereignisse = []          # (zeit, text, art) art: stopp | fahren | info
        self.letzte_bild_nr = -1
        # Ampel
        self.ampel_halt = False
        self.ampel_kand, self.ampel_n = None, 0
        self.ampel_zuletzt_rot = 0.0
        # Stoppschild
        self.schild_n = 0
        self.schild_bis = 0.0
        self.schild_gesperrt = False
        self.schild_zuletzt = 0.0
        # LiDAR / Hindernis
        self.hindernis_bis = 0.0
        self.meldung_seit = None      # seit wann meldet der LiDAR etwas im Pruefbereich
        self.geprueft_frei = False    # Arm hat genauer geschaut: nichts da
        # Arm
        self.arm_phase = None         # None | hin | schauen | zurueck
        self.arm_phase_seit = 0.0
        self.arm_bestaetigt = False
        self.arm_zurueck_noetig = False  # Arm steht evtl. nicht in Fahrstellung
        self.arm_warten_bis = 0.0
        self.letzte_aktion = None
        self.letzter_grund = None
        self.jetzt = 0.0

    @property
    def arm_erlaubt(self):
        """Arm nur bewegen, wenn freigegeben UND das Fahrprogramm wirklich faehrt (nicht im TEST, nicht nach STOPP)."""
        return self.arm_freigabe and self.fahrt_aktiv

    def kommando(self, jetzt, text):
        """Wunsch von aussen (Topic /ki/kommando). 'umschauen' = bei naechster Gelegenheit kurz anhalten
        und links/rechts schauen (fuer die Karte). Rueckgabe: Antworttext."""
        if text == 'umschauen':
            posen_da = all(self.posen.get(p) for p in ('blick_links', 'blick_rechts', 'fahrstellung'))
            if not (self.arm_freigabe and posen_da):
                return 'Umschauen geht nicht: Arm fuer die KI nicht freigegeben oder Posen fehlen'
            if not self.fahrt_aktiv:
                return 'Umschauen geht nur waehrend der Fahrt (START), nicht im TEST'
            self.umschauen_wunsch = jetzt
            return 'Umschauen vorgemerkt'
        return f'unbekanntes Kommando: {text}'

    def _ereignis(self, jetzt, text, art='info'):
        self.ereignisse.append((jetzt, text, art))

    # ------------------------------------------------------------------ #
    def update(self, jetzt, w, szenario='normal'):
        """w = Wahrnehmung (dict):
          bild_nr, bild_zeit   Nummer/Zeit des zuletzt von der KI ausgewerteten Bildes
          objekte              [{name, sicher, im_weg}, ...] (Kamera-KI)
          ampel, ampel_quelle  'rot' | 'gelb' | 'gruen' | None
          stoppschild, schild_quelle   True/False
          lidar                naechster Abstand vorne (m), inf = frei, None = keine Daten
          tiefe_hindernis      True/False, None = Tiefenkamera nicht bereit
          lidar_breit          wie lidar, aber breiter Bereich vorne (Zebrastreifen)
          objekt               naechster Gegenstand {vor, seite, breite, hoehe} (Tiefenkamera/LiDAR)
          zebra                None | {abstand} ; einfahrt_verboten True/False
          gier                 aufsummierte Drehung (rad, IMU) ; linie {kamera, quer} vom Linienfolger
        Rueckgabe: {'aktion', 'faktor', 'grund', 'arm', 'manoever'}
        """
        einsatz = szenario == 'einsatz'
        self.jetzt = jetzt
        if not self.fahrt_aktiv:
            # STOPP gedrueckt oder nur TEST: nichts am Arm bewegen, laufende Arm-Ablaeufe abbrechen.
            # Der Arm bleibt, wo er ist; beim naechsten START faehrt er zuerst in die Fahrstellung.
            if self.ablauf is not None and self.ablauf.braucht_arm:
                self._ereignis(jetzt, f'{self.ablauf.name}: abgebrochen (Fahrprogramm aus)', 'info')
                if self.ablauf.arm_bewegt or self.ablauf.name == 'aufheben':
                    self.arm_zurueck_noetig = True
                self.ablauf = None
            if self.arm_phase is not None:
                self._arm_abbrechen()
        neues_bild = w.get('bild_nr', -1) != self.letzte_bild_nr
        if neues_bild:
            self.letzte_bild_nr = w.get('bild_nr', -1)
            if self.ablauf is None and self.arm_in_fahrstellung:
                # Ampel/Schild nur zaehlen, wenn die Kamera normal nach vorne schaut (nicht beim
                # Umschauen oder Greifen). Sonst liefe z. B. die Haltezeit am Stoppschild waehrend
                # des Aufhebens ab. Nach dem Ablauf sieht die Kamera Ampel/Schild ja wieder.
                self._ampel_bild(jetzt, w)
                self._schild_bild(jetzt, w)
            self.einfahrt_n = self.einfahrt_n + 1 if w.get('einfahrt_verboten') else 0
            if w.get('zebra'):
                self.zebra_zuletzt = jetzt
        if self.zebra_gesperrt and self.ablauf is None and jetzt - self.zebra_zuletzt > self.c['zebra_weg']:
            self.zebra_gesperrt = False  # ueber den Zebrastreifen drueber
        self._ampel_zeit(jetzt)
        w = dict(w, neues_bild=neues_bild)

        befehl = self._entscheide(jetzt, w, einsatz)
        befehl = self._blockade(jetzt, w, befehl)
        # Arm zuerst zurueck in Fahrstellung, bevor wieder gefahren wird
        # (nicht waehrend einer Notbremse: dann ist etwas sehr nah am Roboter)
        if self.arm_zurueck_noetig and self.fahrt_aktiv and not befehl['grund'].startswith('NOTBREMSE'):
            self.arm_zurueck_noetig = False
            self.arm_warten_bis = jetzt + self.c['arm_zeit']
            befehl = self._b('stopp', 'Arm faehrt zurueck in Fahrstellung', arm='fahrstellung')
        elif jetzt < self.arm_warten_bis and befehl['aktion'] != 'stopp':
            befehl = self._b('stopp', 'Arm faehrt zurueck in Fahrstellung')
        if not self.fahrt_aktiv and befehl.get('arm'):
            befehl = dict(befehl, arm=None)    # ohne fahrendes Fahrprogramm bewegt die KI den Arm nie
        if befehl['aktion'] != self.letzte_aktion or befehl['grund'] != self.letzter_grund:
            if befehl['aktion'] != self.letzte_aktion:
                art = befehl['aktion'] if befehl['aktion'] in ('stopp', 'langsam') else 'fahren'
                self._ereignis(jetzt, f"{befehl['aktion'].upper()}: {befehl['grund']}", art)
            self.letzte_aktion, self.letzter_grund = befehl['aktion'], befehl['grund']
        return befehl

    # ------------------------------------------------------------------ #
    def _ampel_bild(self, jetzt, w):
        farbe = w.get('ampel')
        if farbe == self.ampel_kand:
            self.ampel_n += 1
        else:
            self.ampel_kand, self.ampel_n = farbe, 1
        if self.ampel_n < self.c['ampel_bilder']:
            return
        quelle = w.get('ampel_quelle') or 'KI'
        if farbe in ('rot', 'gelb'):
            self.ampel_zuletzt_rot = jetzt
            if not self.ampel_halt:
                self.ampel_halt = True
                self._ereignis(jetzt, f'{farbe.capitalize()}e Ampel erkannt ({quelle})', 'stopp')
        elif farbe == 'gruen' and self.ampel_halt:
            self.ampel_halt = False
            self._ereignis(jetzt, f'Gruene Ampel erkannt ({quelle}) -> darf weiter', 'fahren')

    def _ampel_zeit(self, jetzt):
        if self.ampel_halt and jetzt - self.ampel_zuletzt_rot > self.c['ampel_timeout']:
            self.ampel_halt = False
            self._ereignis(jetzt, 'Rote Ampel nicht mehr zu sehen -> weiter', 'fahren')

    def _schild_bild(self, jetzt, w):
        if w.get('stoppschild'):
            self.schild_zuletzt = jetzt
        elif self.schild_gesperrt and jetzt >= self.schild_bis and jetzt - self.schild_zuletzt > self.c['schild_weg']:
            self.schild_gesperrt = False  # am Schild vorbei
        if not w.get('stoppschild') or self.schild_gesperrt:
            self.schild_n = 0
            return
        self.schild_n += 1
        if self.schild_n >= self.c['schild_bilder']:
            self.schild_n = 0
            self.schild_gesperrt = True
            self.schild_bis = jetzt + self.c['schild_warten']
            self._ereignis(jetzt, f"Stoppschild erkannt ({w.get('schild_quelle') or 'KI'})", 'stopp')

    # ------------------------------------------------------------------ #
    def _entscheide(self, jetzt, w, einsatz):
        c = self.c
        fahrfaktor = c['einsatz_faktor'] if einsatz else 1.0
        d = w.get('lidar')
        ki_frisch = jetzt - w.get('bild_zeit', 0.0) <= c['ki_max_alter']
        im_weg = [o['name'] for o in w.get('objekte', []) if o.get('im_weg')]

        # 1. Notbremse
        if d is not None and d < c['notbremse_dist']:
            self._arm_abbrechen()
            self.hindernis_bis = jetzt + c['frei_zeit']
            return self._b('stopp', f'NOTBREMSE: LiDAR {d * 100:.0f} cm (ohne KI-Pruefung)')
        # 2. Blind?
        if d is None and c['lidar_pflicht']:
            return self._b('stopp', 'Kein LiDAR -> fahre nicht blind')
        if not ki_frisch:
            return self._b('stopp', 'KI bekommt keine Kamerabilder')

        # Laufender Ablauf (Zebrastreifen, Wenden, Aufheben) hat Vorrang
        if self.ablauf is not None:
            b = self.ablauf.schritt(jetzt, w)
            for text, art in self.ablauf.ereignisse:
                self._ereignis(jetzt, text, art)
            self.ablauf.ereignisse.clear()
            if b is not None:
                return self._mit_faktor(b)
            self._ablauf_ende(jetzt)

        obj = w.get('objekt')
        tiefe_im_weg = obj is not None and obj.get('quelle') == 'tiefe' and obj['vor'] < c['tiefe_halt']
        if obj is None and (d is None or d >= c['pruef_dist']):
            self.weg_frei_seit = self.weg_frei_seit or jetzt
            if jetzt - self.weg_frei_seit > 2.0:
                self.aufheben_gesperrt = False
        else:
            self.weg_frei_seit = None

        # 3. LiDAR-Meldung pruefen
        im_pruefbereich = d is not None and d < c['pruef_dist']
        if im_pruefbereich:
            if self.meldung_seit is None:
                self.meldung_seit = jetzt
                self._ereignis(jetzt, f'LiDAR meldet etwas in {d * 100:.0f} cm -> KI prueft', 'info')
            beweise = []
            if im_weg:
                beweise.append('Kamera-KI: ' + ', '.join(im_weg))
            if w.get('tiefe_hindernis'):
                beweise.append('Tiefenkamera')
            if self.arm_phase is not None:
                return self._arm_schritt(jetzt, d, beweise)
            if beweise:
                if jetzt >= self.hindernis_bis:
                    self._ereignis(jetzt, f"Hindernis bestaetigt ({' + '.join(beweise)})", 'stopp')
                self.hindernis_bis = jetzt + c['frei_zeit']
                if self._aufhebbar(w):
                    return self._ablauf_start(jetzt, Aufheben(jetzt, c, obj))
                return self._b('stopp', f"Hindernis {d * 100:.0f} cm, bestaetigt durch {' + '.join(beweise)}")
            if jetzt < self.hindernis_bis:
                return self._b('stopp', 'Hindernis war bestaetigt -> warte, bis der Weg frei ist')
            if w.get('bild_zeit', 0.0) < self.meldung_seit:
                return self._b('langsam', 'LiDAR meldet etwas -> KI schaut nach ...', c['langsam_faktor'])
            if (self.arm_erlaubt and self.posen.get('pruefblick') and not self.geprueft_frei
                    and jetzt - self.meldung_seit >= c['umschauen_nach']):
                return self._arm_starten(jetzt)
            if d < c['lidar_halt']:
                # Der LiDAR schaut nur in den Fahrschlauch: so nah = wuerde gleich anstossen
                return self._b('stopp', f'LiDAR: etwas {d * 100:.0f} cm im Weg, Kamera sieht nichts -> halte')
            grund = 'LiDAR-Meldung nicht bestaetigt -> langsam'
            if self.geprueft_frei:
                grund = 'Arm hat nachgeschaut: nichts im Weg -> langsam'
            return self._b('langsam', grund, c['langsam_faktor'])

        # LiDAR frei
        if self.meldung_seit is not None:
            self.meldung_seit = None
            self.geprueft_frei = False
            self._arm_abbrechen()
        # Tiefenkamera allein: etwas unter der LiDAR-Ebene im Weg (z. B. Holzwuerfel)
        if tiefe_im_weg:
            if jetzt >= self.hindernis_bis:
                self._ereignis(jetzt, f"Tiefenkamera: Hindernis {obj['vor'] * 100:.0f} cm voraus, "
                                      f"{obj['hoehe'] * 100:.0f} cm hoch (unter der LiDAR-Ebene)", 'stopp')
            self.hindernis_bis = jetzt + c['frei_zeit']
            if self._aufhebbar(w):
                return self._ablauf_start(jetzt, Aufheben(jetzt, c, obj))
            grund = ' (Aufheben hat nicht geklappt)' if self.aufheben_gesperrt else ''
            return self._b('stopp', f"Hindernis {obj['vor'] * 100:.0f} cm (Tiefenkamera) -> warte{grund}")

        if jetzt < self.hindernis_bis:
            return self._b('stopp', 'Hindernis war bestaetigt -> warte, bis der Weg frei ist')

        # 4. KI sieht etwas im Weg (auch ohne LiDAR, z. B. weiter weg oder unter der LiDAR-Scheibe)
        if im_weg:
            return self._b('stopp', 'KI sieht im Weg: ' + ', '.join(im_weg))
        # 5. Einbahnstrasse von der falschen Seite -> wenden
        if self.einfahrt_n >= c['einfahrt_bilder'] and jetzt >= self.einfahrt_gesperrt_bis:
            self.einfahrt_n = 0
            self._ereignis(jetzt, 'Schild "Einfahrt verboten" (Einbahnstrasse) -> wende', 'stopp')
            return self._ablauf_start(jetzt, Wenden(jetzt, c, w.get('gier', 0.0)))
        # 6. Zebrastreifen -> anhalten und umschauen
        z = w.get('zebra')
        if z and not self.zebra_gesperrt and z['abstand'] <= c['zebra_halt']:
            self._ereignis(jetzt, f"Zebrastreifen {z['abstand'] * 100:.0f} cm voraus -> anhalten, umschauen", 'stopp')
            posen = self.posen if self.arm_erlaubt else {}
            return self._ablauf_start(jetzt, Zebrastreifen(jetzt, c, posen, z['abstand']))
        # Umschauen (Wunsch von Kartograf/Panel): nur wenn sonst nichts los ist, Wunsch gilt 20 s
        if self.umschauen_wunsch is not None:
            if jetzt - self.umschauen_wunsch < 20.0 and not self.ampel_halt and jetzt >= self.schild_bis:
                self.umschauen_wunsch = None
                self._ereignis(jetzt, 'Schaue mich fuer die Karte um', 'info')
                return self._ablauf_start(jetzt, Umschauen(jetzt, c, 'Umschauen fuer die Karte'))
            if jetzt - self.umschauen_wunsch >= 20.0:
                self.umschauen_wunsch = None
        # 5./6. Verkehrsregeln
        if self.ampel_halt:
            if einsatz:
                return self._b('fahren', 'EINSATZ: rote Ampel wird ueberfahren (Sonderrechte)', fahrfaktor)
            return self._b('stopp', 'Rote Ampel -> warte auf Gruen')
        if jetzt < self.schild_bis:
            if einsatz:
                return self._b('fahren', 'EINSATZ: Stoppschild wird uebergangen', fahrfaktor)
            return self._b('stopp', f'Stoppschild -> halte noch {self.schild_bis - jetzt:.0f} s')
        return self._b('fahren', 'EINSATZFAHRT, Weg frei' if einsatz else 'Weg frei', fahrfaktor)

    BLOCKADE_GRUENDE = ('NOTBREMSE', 'Hindernis', 'LiDAR: etwas', 'KI sieht im Weg')

    def _blockade(self, jetzt, w, befehl):
        """Steht er schon lange vor einem Hindernis, weicht er nach zufaelliger Zeit aus."""
        blockiert = (befehl['aktion'] == 'stopp' and self.ablauf is None
                     and befehl['grund'].startswith(self.BLOCKADE_GRUENDE))
        if not blockiert or not self.fahrt_aktiv:
            self.blockiert_seit = None
            return befehl
        if self.blockiert_seit is None:
            self.blockiert_seit = jetzt
            self.blockiert_grenze = random.uniform(self.c['blockiert_min'], self.c['blockiert_max'])
        if jetzt - self.blockiert_seit < self.blockiert_grenze:
            return befehl
        self.blockiert_seit = None
        self._ereignis(jetzt, f'Weg seit {self.blockiert_grenze:.0f} s versperrt (vielleicht ein anderer Roboter) '
                              '-> weiche aus', 'stopp')
        self.hindernis_bis = 0.0
        return self._ablauf_start(jetzt, Ausweichen(jetzt, self.c, w.get('gier', 0.0)))

    # ------------------------------------------------------------------ #
    # Ablaeufe
    def _ablauf_start(self, jetzt, ablauf):
        self.ablauf = ablauf
        b = ablauf.schritt(jetzt, {})
        for text, art in ablauf.ereignisse:
            self._ereignis(jetzt, text, art)
        ablauf.ereignisse.clear()
        return self._mit_faktor(b) if b else self._b('stopp', ablauf.name)

    def _ablauf_ende(self, jetzt):
        a = self.ablauf
        self.ablauf = None
        if a.arm_bewegt:
            self.arm_zurueck_noetig = True
        if a.name == 'zebra':
            self.zebra_gesperrt = True
            self.zebra_zuletzt = jetzt   # beim Umschauen sah die Kamera den Zebrastreifen nicht
        elif a.name in ('wenden', 'ausweichen'):
            self.einfahrt_gesperrt_bis = jetzt + 6.0
            self.einfahrt_n = 0
        elif a.name == 'aufheben':
            self.hindernis_bis = 0.0
            if a.ergebnis != 'ok':
                self.aufheben_gesperrt = True

    def _aufhebbar(self, w):
        obj = w.get('objekt')
        if not (self.arm_erlaubt and obj and not self.aufheben_gesperrt):
            return False
        if not all(self.posen.get(p) for p in GREIF_POSEN):
            return False
        if any(o['name'] in LEBEWESEN | FAHRZEUGE for o in w.get('objekte', [])):
            return False   # Lebewesen und Fahrzeuge werden nie angefasst
        if self.zebra_gesperrt or self.jetzt - self.zebra_zuletzt < self.c['zebra_kein_aufheben']:
            return False   # am Zebrastreifen stehen Fussgaenger -> nichts anfassen, nur warten
        if obj.get('quer') is not None and obj['quer'] > self.c['aufheben_max_quer']:
            return False   # liegt am Rand, nicht mitten auf der Strasse
        return obj['breite'] <= self.c['aufheben_max_breite'] and obj.get('hoehe', 0) <= self.c['aufheben_max_hoehe']

    @staticmethod
    def _mit_faktor(b):
        b = dict(b)
        b.setdefault('faktor', 0.0)
        return b

    # ------------------------------------------------------------------ #
    # Arm: bei unklarer LiDAR-Meldung im Stand genauer hinschauen
    def _arm_starten(self, jetzt):
        self.arm_phase, self.arm_phase_seit, self.arm_bestaetigt = 'hin', jetzt, False
        self._ereignis(jetzt, 'Unklar -> KI dreht die Kamera (Arm), um genauer zu schauen', 'info')
        return self._b('stopp', 'KI schaut mit dem Arm genauer hin ...', arm='pruefblick')

    def _arm_schritt(self, jetzt, d, beweise):
        dauer = jetzt - self.arm_phase_seit
        if beweise:
            self.arm_bestaetigt = True
        if self.arm_phase == 'hin':
            if dauer >= self.c['arm_zeit']:
                self.arm_phase, self.arm_phase_seit = 'schauen', jetzt
            return self._b('stopp', 'KI schaut mit dem Arm genauer hin ...')
        if self.arm_phase == 'schauen':
            if dauer >= 1.0 or self.arm_bestaetigt:
                self.arm_phase, self.arm_phase_seit = 'zurueck', jetzt
                return self._b('stopp', 'Arm faehrt zurueck in Fahrstellung', arm='fahrstellung')
            return self._b('stopp', 'KI schaut mit dem Arm genauer hin ...')
        # zurueck
        if dauer < self.c['arm_zeit']:
            return self._b('stopp', 'Arm faehrt zurueck in Fahrstellung')
        self.arm_phase = None
        if self.arm_bestaetigt:
            self._ereignis(jetzt, 'Arm-Blick: Hindernis bestaetigt', 'stopp')
            self.hindernis_bis = jetzt + self.c['frei_zeit']
            return self._b('stopp', 'Hindernis bestaetigt (Arm-Blick)')
        self.geprueft_frei = True
        self._ereignis(jetzt, 'Arm-Blick: nichts im Weg -> langsam weiter', 'fahren')
        return self._b('langsam', 'Arm hat nachgeschaut: nichts im Weg -> langsam', self.c['langsam_faktor'])

    def _arm_abbrechen(self):
        # Bei Notbremse oder wenn der Weg frei wird: Arm-Ablauf beenden.
        # Stand der Arm gerade nicht in Fahrstellung, schickt die Zentrale ihn zurueck.
        if self.arm_phase in ('hin', 'schauen'):
            self.arm_zurueck_noetig = True
        self.arm_phase = None

    @property
    def arm_in_fahrstellung(self):
        """Schaut die Kamera (am Arm) normal nach vorne? Nur dann stimmen Draufsicht, Linie und
        Tiefenkamera-Bodenmodell. Waehrend der Arm zurueckfaehrt (arm_warten_bis) noch nicht."""
        return (self.arm_phase is None and not self.arm_zurueck_noetig and self.jetzt >= self.arm_warten_bis
                and not (self.ablauf is not None and self.ablauf.arm_bewegt))

    @staticmethod
    def _b(aktion, grund, faktor=None, arm=None):
        if faktor is None:
            faktor = 0.0 if aktion == 'stopp' else 1.0
        return {'aktion': aktion, 'faktor': round(float(faktor), 2), 'grund': grund, 'arm': arm, 'manoever': None}
