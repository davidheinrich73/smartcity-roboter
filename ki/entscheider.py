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
#        nicht bestaetigt -> langsam (ggf. vorher mit dem Arm genauer hinschauen)
#   4. KI sieht Person/Auto/... im Weg  -> stopp
#   5. Ampel rot/gelb -> stopp, bis gruen (oder Ampel nicht mehr zu sehen)
#   6. Stoppschild -> kurz halten, dann weiter
# Im Szenario "einsatz" (RTW) werden 5 und 6 uebergangen, 1-4 gelten weiter.

STANDARD = {
    'notbremse_dist': 0.12,    # LiDAR naeher (m) -> sofort Stopp, ohne Rueckfrage
    'pruef_dist': 0.45,        # LiDAR naeher (m) -> KI prueft
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
}


class Entscheider:
    def __init__(self, cfg=None, arm_erlaubt=False):
        self.c = dict(STANDARD)
        self.c.update(cfg or {})
        self.arm_erlaubt = arm_erlaubt
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
        Rueckgabe: {'aktion', 'faktor', 'grund', 'arm'}
        """
        einsatz = szenario == 'einsatz'
        neues_bild = w.get('bild_nr', -1) != self.letzte_bild_nr
        if neues_bild:
            self.letzte_bild_nr = w.get('bild_nr', -1)
            self._ampel_bild(jetzt, w)
            self._schild_bild(jetzt, w)
        self._ampel_zeit(jetzt)

        befehl = self._entscheide(jetzt, w, einsatz)
        # Arm zuerst zurueck in Fahrstellung, bevor wieder gefahren wird
        # (nicht waehrend einer Notbremse: dann ist etwas sehr nah am Roboter)
        if self.arm_zurueck_noetig and not befehl['grund'].startswith('NOTBREMSE'):
            self.arm_zurueck_noetig = False
            self.arm_warten_bis = jetzt + self.c['arm_zeit']
            befehl = self._b('stopp', 'Arm faehrt zurueck in Fahrstellung', arm='fahrstellung')
        elif jetzt < self.arm_warten_bis and befehl['aktion'] != 'stopp':
            befehl = self._b('stopp', 'Arm faehrt zurueck in Fahrstellung')
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
                return self._b('stopp', f"Hindernis {d * 100:.0f} cm, bestaetigt durch {' + '.join(beweise)}")
            if jetzt < self.hindernis_bis:
                return self._b('stopp', 'Hindernis war bestaetigt -> warte, bis der Weg frei ist')
            if w.get('bild_zeit', 0.0) < self.meldung_seit:
                return self._b('langsam', 'LiDAR meldet etwas -> KI schaut nach ...', c['langsam_faktor'])
            if (self.arm_erlaubt and not self.geprueft_frei
                    and jetzt - self.meldung_seit >= c['umschauen_nach']):
                return self._arm_starten(jetzt)
            grund = 'LiDAR-Meldung nicht bestaetigt (z. B. Haus am Rand) -> langsam'
            if self.geprueft_frei:
                grund = 'Arm hat nachgeschaut: nichts im Weg -> langsam'
            return self._b('langsam', grund, c['langsam_faktor'])

        # LiDAR frei
        if self.meldung_seit is not None:
            self.meldung_seit = None
            self.geprueft_frei = False
            self._arm_abbrechen()
        if jetzt < self.hindernis_bis:
            return self._b('stopp', 'Hindernis war bestaetigt -> warte, bis der Weg frei ist')

        # 4. KI sieht etwas im Weg (auch ohne LiDAR, z. B. weiter weg oder unter der LiDAR-Scheibe)
        if im_weg:
            return self._b('stopp', 'KI sieht im Weg: ' + ', '.join(im_weg))
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

    @staticmethod
    def _b(aktion, grund, faktor=None, arm=None):
        if faktor is None:
            faktor = 0.0 if aktion == 'stopp' else 1.0
        return {'aktion': aktion, 'faktor': round(float(faktor), 2), 'grund': grund, 'arm': arm}
