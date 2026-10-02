#!/usr/bin/env python3
# Linie finden und lenken (ohne ROS, damit testbar).
#
# 1. Untere Bildhaelfte in eine DRAUFSICHT umrechnen (Vogelperspektive, Kamerahoehe/Neigung/
#    Blickwinkel). Ohne diese Umrechnung taeuscht die Perspektive: eine gerade Linie neben dem
#    Roboter sieht im Bild schraeg aus. In der Draufsicht ist 1 Pixel = RASTER Meter.
# 2. Dunkle Flaechen suchen. Genommen wird die, die am besten zur bisherigen Linie passt
#    (nicht die groesste) -> Zebrastreifen und Kreuzungen stoeren weniger.
#    Mittellinie = "Grat" der Abstandstransformation (Punkte, die am weitesten vom Rand weg sind).
# 3. Gedaechtnis: Linienpunkte in Metern (x = vorne, y = links) merken und mit der eigenen Bewegung
#    mitverschieben. So ist die Linie auch direkt UNTER dem Roboter bekannt (die Kamera sieht dort
#    nichts) -> Abstand, Winkel und Kruemmung der Linie genau am Roboter.
# 4. Lenken: Kruemmung vorsteuern (wie eng die Kurve ist) + Winkel und Abstand nachregeln.
#    In Kurven langsamer. Linie weg: zur Seite drehen, wo sie zuletzt war, bis sie wieder da ist.
import math
import time
from collections import deque

import cv2
import numpy as np

STANDARD = {
    'threshold': 70,          # dunkler als das = Linie (0 schwarz - 255 weiss)
    'zeilen_oben': 0.5,       # Linie nur in der unteren Bildhaelfte suchen (ab 50 % Hoehe)
    'linie_max_breite': 0.06,  # dunkle Flaeche breiter als das (m) = keine Linie (z. B. Schatten)
    'linie_min_flaeche': 40,  # kleiner (Pixel der Draufsicht, 1 Pixel = 4x4 mm) = Rauschen
    # Kamera (am Arm in Fahrstellung). GESCHAETZT, am Roboter nachmessen:
    'kamera_hoehe': 0.22,     # Hoehe der Kamera ueber dem Boden (m)
    'kamera_neigung': 32.0,   # wie weit sie nach unten schaut (Grad)
    'kamera_fov': 70.0,       # waagrechter Blickwinkel (Grad)
    'kamera_x': 0.12,         # wie weit die Kamera vor der Robotermitte sitzt (m)
    # Lenkung
    'max_turn': 1.5,          # hoechste Drehgeschwindigkeit (rad/s)
    'kurven_bremse': 0.15,    # Tempo-Faktor = 1 / (1 + Bremse * Kruemmung)  (Kruemmung = 1/Radius)
    'vorausschau': 0.25,      # Notloesung (noch nichts unter dem Roboter gemerkt): Ziel so weit voraus
    'regel_abstand': 44.0,    # Nachregeln seitlicher Abstand (1/m^2), 'regel_richtung' ~ 2*Wurzel davon
    'regel_richtung': 13.0,   # Nachregeln Richtung (1/m) -> gedaempft, ohne Schlingern
    'min_tempo': 0.4,         # Kurve: mindestens so viel vom Tempo
    'such_zeit': 0.8,         # Linie weg: so lange (s) vorsichtig zur letzten Seite drehen (ca. 30 Grad), dann Stopp.
                              # Nicht laenger: blindes Drehen kann Aufbauten streifen (Zieldefinition 4.1)
    'such_dreh': 0.7,         # Drehgeschwindigkeit beim Suchen (rad/s)
}


def boden_punkt(u, v, breite, hoehe, c):
    """Bildpunkt (u, v) -> Bodenpunkt (x vorne, y links) in Metern oder None (ueber dem Horizont)."""
    f = (breite / 2) / math.tan(math.radians(c['kamera_fov']) / 2)
    du, dv = (u - breite / 2) / f, (v - hoehe / 2) / f
    p = math.radians(c['kamera_neigung'])
    rx = math.cos(p) - dv * math.sin(p)       # Strahl im Roboter-System
    ry = -du
    rz = -math.sin(p) - dv * math.cos(p)
    if rz > -1e-3:
        return None
    t = c['kamera_hoehe'] / -rz
    return c['kamera_x'] + t * rx, t * ry


def punkt_aus_tiefe(u_anteil, v_anteil, tiefe, c):
    """Bildpunkt (als Anteil der Bildbreite/-hoehe) + Tiefe (m, entlang der Blickachse)
    -> Punkt im Roboter-System (x vorne, y links, z hoch). Kamera in Fahrstellung."""
    f_anteil = 0.5 / math.tan(math.radians(c['kamera_fov']) / 2)   # Brennweite in Bildbreiten
    du = (u_anteil - 0.5) / f_anteil
    dv = (v_anteil - 0.5) * 0.75 / f_anteil                          # Bild 4:3
    p = math.radians(c['kamera_neigung'])
    x = c['kamera_x'] + tiefe * (math.cos(p) - dv * math.sin(p))
    y = -tiefe * du
    z = c['kamera_hoehe'] + tiefe * (-math.sin(p) - dv * math.cos(p))
    return x, y, z


class LinienSucher:
    """Findet die Mittellinie der schwarzen Linie in der Vogelperspektive (Draufsicht auf den Boden)."""

    RASTER = 0.004                 # Meter pro Pixel in der Draufsicht
    VORNE = (0.05, 0.85)           # Bereich vor dem Roboter (m)
    SEITE = 0.45                   # +- seitlich (m)

    def __init__(self, cfg=None):
        self.c = dict(STANDARD)
        self.c.update(cfg or {})
        self.groesse = None
        self.erwartung = None      # wo die Linie vermutlich ist (Roboter-System), vom Gedaechtnis

    def _vogel(self, w, h):
        """Umrechnung Kamerabild -> Draufsicht (Homographie, gilt fuer den ebenen Boden)."""
        if self.groesse == (w, h):
            return
        self.groesse = (w, h)
        c = self.c
        bild, boden = [], []
        oben = h * c['zeilen_oben']
        for u, v in ((0.05 * w, h * 0.98), (0.95 * w, h * 0.98), (0.05 * w, oben), (0.95 * w, oben)):
            b = boden_punkt(u, v, w, h, c)
            if b is not None:
                bild.append((u, v))
                boden.append(self._zu_raster(*b))
        self.H = cv2.getPerspectiveTransform(np.float32(bild), np.float32(boden))
        bw = int(2 * self.SEITE / self.RASTER)
        bh = int((self.VORNE[1] - self.VORNE[0]) / self.RASTER)
        self.vogel_groesse = (bw, bh)

    def _zu_raster(self, x, y):
        return ((self.SEITE - y) / self.RASTER, (self.VORNE[1] - x) / self.RASTER)

    def _zu_boden(self, px, py):
        return self.VORNE[1] - py * self.RASTER, self.SEITE - px * self.RASTER

    def suche(self, img):
        """Rueckgabe: dict gefunden, boden (Liste Mittellinien-Punkte x, y in m), punkte (Bild) fuer die Anzeige."""
        h, w = img.shape[:2]
        c = self.c
        self._vogel(w, h)
        oben = int(h * c['zeilen_oben'])
        gray = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        maske = np.zeros((h, w), np.uint8)
        maske[oben:] = (gray[oben:] < c['threshold']).astype(np.uint8) * 255
        vogel = cv2.warpPerspective(maske, self.H, self.vogel_groesse, flags=cv2.INTER_NEAREST)
        erg = {'gefunden': False, 'boden': [], 'punkte': [], 'fehler': None, 'zeilen': (oben, h - 1), 'vogel': vogel}
        n, etiketten, stat, _ = cv2.connectedComponentsWithStats(vogel)
        if n <= 1:
            return erg
        abstand = cv2.distanceTransform(vogel, cv2.DIST_L2, 3)
        ziel = self.erwartung if self.erwartung is not None else (0.3, 0.0)
        zx, zy = self._zu_raster(*ziel)
        beste, beste_d = None, None
        for i in range(1, n):
            if stat[i, cv2.CC_STAT_AREA] < c['linie_min_flaeche']:
                continue
            teil = etiketten == i
            dicke = 2 * abstand[teil].max() * self.RASTER
            if dicke > c['linie_max_breite']:   # zu breit -> Flaeche, keine Linie
                continue
            ys, xs = np.nonzero(teil)
            d = np.min(np.hypot(xs - zx, ys - zy))
            if beste_d is None or d < beste_d:
                beste, beste_d = i, d
        if beste is None:
            return erg
        teil = etiketten == beste
        a = abstand * teil
        grat = teil & (a >= 0.6 * a.max())                     # Mittellinie (Grat der Abstandskarte)
        ys, xs = np.nonzero(grat)
        if len(xs) < 3:
            return erg
        schritt = max(1, len(xs) // 60)
        boden = [self._zu_boden(px, py) for px, py in zip(xs[::schritt], ys[::schritt])]
        inv = np.linalg.inv(self.H)
        auswahl = zip(xs[::schritt * 3], ys[::schritt * 3])
        pts = cv2.perspectiveTransform(np.float32([[[px, py]] for px, py in auswahl]), inv)
        erg.update({'gefunden': True, 'boden': boden, 'punkte': [(float(p[0][0]), int(p[0][1])) for p in pts]})
        naechster = min(boden, key=lambda b: b[0])
        erg['fehler'] = max(-1.0, min(1.0, -naechster[1] / 0.15))
        # Gerade durch die Punkte: wo laege die Linie am Roboter (quer) und in welche Richtung zeigt sie (kurs)?
        bx, by = np.array(boden).T
        if np.ptp(bx) > 0.05:
            steigung, quer = np.polyfit(bx, by, 1)
            erg['quer'], erg['kurs'] = float(quer), float(math.atan(steigung))
        return erg


class Gedaechtnis:
    """Merkt sich gesehene Linienpunkte und rechnet sie mit der eigenen Bewegung mit.

    So kennt der Roboter die Linie auch direkt UNTER sich (die Kamera sieht nur 25-60 cm voraus)
    und kann zwischen zwei Kamerabildern weiterlenken.
    Punkte altern nur beim FAHREN (im Stand und beim Drehen auf der Stelle 10x langsamer): Haelt der
    Roboter mitten in einer engen Kurve (Kamera sieht dort die Linie nicht) oder wendet er, weiss er
    danach trotzdem noch, wo sie ist.
    """

    def __init__(self, max_alter=6.0):
        self.pose = [0.0, 0.0, 0.0]     # eigene Position durch Mitrechnen (nur kurzfristig genau)
        self.punkte = np.zeros((0, 3))  # x, y (im Mitrechen-System), Fahrzeit beim Sehen
        self.max_alter = max_alter      # Sekunden FAHRZEIT
        self.uhr = 0.0                  # Fahrzeit (laeuft im Stand/beim Drehen nur 10x langsamer)
        self.verlauf = deque(maxlen=200)  # (Zeit, Pose) der letzten ~10 s: wo war der Roboter wann?

    def bewegen(self, v, dreh, dt, quer=0.0, jetzt=None):
        x, y, w = self.pose
        self.pose = [x + (v * math.cos(w) - quer * math.sin(w)) * dt,
                     y + (v * math.sin(w) + quer * math.cos(w)) * dt, w + dreh * dt]
        faehrt = abs(v) >= 0.005 or abs(quer) >= 0.005
        self.uhr += dt * (1.0 if faehrt else 0.1)
        if jetzt is not None:
            self.verlauf.append((jetzt, tuple(self.pose)))

    def pose_bei(self, zeit):
        """Wo war der Roboter (Mitrechen-System) zur Zeit 'zeit'? Fuer Kamerabilder, die verspaetet ankommen."""
        if zeit is None or not self.verlauf or zeit >= self.verlauf[-1][0]:
            return tuple(self.pose)
        frueher = self.pose
        for t, pose in reversed(self.verlauf):
            if t <= zeit:
                return pose
            frueher = pose
        return tuple(frueher)            # aelter als der Verlauf: aelteste bekannte Pose

    def _zu_roboter(self, p):
        x, y, w = self.pose
        dx, dy = p[:, 0] - x, p[:, 1] - y
        return np.stack([math.cos(w) * dx + math.sin(w) * dy, -math.sin(w) * dx + math.cos(w) * dy], axis=1)

    def _frisch(self):
        return self.punkte[self.punkte[:, 2] > self.uhr - self.max_alter]

    def hinzufuegen(self, boden, aufnahme=None):
        """boden: Linienpunkte (x, y) in m, so wie die Kamera sie gesehen hat.
        aufnahme: Zeitpunkt der Aufnahme. Kommt das Bild verspaetet an, werden die Punkte dort eingetragen,
        wo der Roboter BEIM FOTOGRAFIEREN stand (sonst laege die Linie um die inzwischen gefahrene Strecke falsch)."""
        x, y, w = self.pose_bei(aufnahme)
        alt = self._frisch()
        if len(alt):
            r = self._zu_roboter(alt)
            alt = alt[r[:, 0] > -0.30]          # weit hinter dem Roboter -> vergessen (30 cm bleiben: Wenden)
        if len(boden) > 25:   # die Punkte liegen dicht -> 25 je Bild reichen
            boden = [boden[i] for i in np.linspace(0, len(boden) - 1, 25).astype(int)]
        neu = np.array([[x + math.cos(w) * bx - math.sin(w) * by, y + math.sin(w) * bx + math.cos(w) * by, self.uhr]
                        for bx, by in boden]) if boden else np.zeros((0, 3))
        if len(neu) and len(alt):
            # Was die Kamera gerade sieht, ersetzt alte Punkte im selben Bereich
            r = self._zu_roboter(alt)
            naechster_neu = self._zu_roboter(neu)[:, 0].min()
            alt = alt[r[:, 0] < naechster_neu]   # naeher als das, was sie gerade sieht
        self.punkte = np.vstack([alt, neu])[-3000:]

    def lokal(self):
        """Linie direkt am Roboter: Abstand, Richtung und Kruemmung, oder None (zu wenig Punkte).

        Die Punkte um den Roboter werden zuerst entlang ihrer Hauptrichtung ausgerichtet
        (Hauptkomponente), dann wird eine Parabel hineingelegt -> klappt in jeder Lage der Linie.
        """
        if len(self.punkte) < 5:
            return None
        r = self._zu_roboter(self._frisch())
        nah = r[(np.hypot(r[:, 0], r[:, 1]) < 0.18) & (r[:, 0] > -0.12)]
        if len(nah) < 5:
            return None
        mitte = nah.mean(axis=0)
        _, _, vt = np.linalg.svd(nah - mitte)
        t = vt[0] if vt[0][0] >= 0 else -vt[0]                 # Richtung der Linie (nach vorne)
        n = np.array([-t[1], t[0]])                             # links davon
        xs, ys = nah @ t, nah @ n
        if np.ptp(xs) < 0.08 or xs.max() < 0.0:
            return None
        c2, b, a = np.polyfit(xs, ys, 2)
        return {'quer': float(a), 'kurs': float(math.atan2(t[1], t[0]) + math.atan(b)),
                'kruemmung': float(2 * c2 / (1 + b * b) ** 1.5)}

    def weg(self, bis=0.8, schritt=0.05):
        """Linie vor dem Roboter als Punktfolge [(x, y), ...] (alle 'schritt' m Abstand vom Roboter ein Punkt),
        z. B. fuer den LiDAR-Fahrschlauch der KI. Leer, wenn keine Linie bekannt ist."""
        frisch = self._frisch()
        if len(frisch) == 0:
            return []
        r = self._zu_roboter(frisch)
        r = r[r[:, 0] > -0.05]
        d = np.hypot(r[:, 0], r[:, 1])
        aus = []
        for ring in np.arange(schritt, bis + 1e-6, schritt):
            im_ring = r[np.abs(d - ring) < schritt / 2]
            if len(im_ring):
                p = np.median(im_ring, axis=0)
                aus.append((round(float(p[0]), 3), round(float(p[1]), 3)))
        return aus

    def linie(self, vorausschau=0.12):
        """Zielpunkt auf der gemerkten Linie, ca. 'vorausschau' Meter vom Roboter entfernt.
        Rueckgabe: dict gefunden, ziel (x, y im Roboter-System), alter (s Fahrzeit)."""
        if len(self.punkte) == 0:
            return {'gefunden': False}
        frisch = self._frisch()
        if len(frisch) == 0:
            return {'gefunden': False}
        r = self._zu_roboter(frisch)
        vorne = r[r[:, 0] > 0.02]
        if len(vorne) == 0:
            return {'gefunden': False}
        d = np.hypot(vorne[:, 0], vorne[:, 1])
        band = np.abs(d - vorausschau) < 0.03
        if band.any():
            kand = vorne[band]
            ziel = kand[np.argmin(np.abs(np.arctan2(kand[:, 1], kand[:, 0])))]  # am ehesten geradeaus
        elif (d > vorausschau).any():
            ziel = vorne[d > vorausschau][np.argmin(d[d > vorausschau])]       # naechster dahinter
        else:
            ziel = vorne[np.argmax(d)]                                          # weitester, den es gibt
        alter = float(self.uhr - frisch[:, 2].max())
        return {'gefunden': True, 'ziel': (float(ziel[0]), float(ziel[1])), 'alter': alter}


class Lenkung:
    def __init__(self, cfg=None):
        self.c = dict(STANDARD)
        self.c.update(cfg or {})
        self.zuletzt_gesehen = 0.0
        self.suche_seite = 0.0   # +1 = Linie war links, -1 = rechts

    def berechne(self, erg, tempo, jetzt=None, lokal=None):
        """erg: Zielpunkt aus Gedaechtnis.linie(), lokal: Gedaechtnis.lokal() (genauer, falls vorhanden).
        Rueckgabe: (vorwaerts m/s, drehen rad/s, Text)."""
        jetzt = time.time() if jetzt is None else jetzt
        c = self.c
        if (erg is None or not erg['gefunden'] or erg.get('alter', 0) > 1.0) and lokal is None:
            weg = jetzt - self.zuletzt_gesehen
            if weg < c['such_zeit'] and self.zuletzt_gesehen > 0:
                return 0.0, c['such_dreh'] * self.suche_seite, f'Linie weg -> suche ({weg:.1f} s)'
            return 0.0, 0.0, 'KEINE LINIE -> STOPP'
        self.zuletzt_gesehen = jetzt
        if lokal is not None:
            # Linie unter dem Roboter bekannt: Kruemmung vorsteuern, Abstand + Richtung nachregeln
            k = max(-8.0, min(8.0, lokal['kruemmung']))
            v = tempo * max(c['min_tempo'], 1.0 / (1.0 + c['kurven_bremse'] * abs(k)))
            korrektur = c['regel_richtung'] * lokal['kurs'] + c['regel_abstand'] * lokal['quer']
            dreh = v * (k + korrektur)
            self.suche_seite = 1.0 if (lokal['quer'] + 0.1 * lokal['kurs']) > 0 else -1.0
            text = f"Linie: {lokal['quer'] * 100:+.1f} cm, {math.degrees(lokal['kurs']):+.0f} Grad"
        else:
            zx, zy = erg['ziel']
            k = 2.0 * zy / max(zx * zx + zy * zy, 0.004)   # Bogen zum Zielpunkt (Pure Pursuit)
            v = tempo * max(c['min_tempo'], 1.0 / (1.0 + c['kurven_bremse'] * abs(k)))
            dreh = v * k
            self.suche_seite = 1.0 if zy > 0 else -1.0
            text = f'Linie: Ziel {zx * 100:.0f}/{zy * 100:+.0f} cm'
        if abs(dreh) > c['max_turn']:        # enger als moeglich -> langsamer statt schneiden
            v *= c['max_turn'] / abs(dreh)
            dreh = math.copysign(c['max_turn'], dreh)
        if abs(k) > 0.5:
            text += f', Kurve r={1 / abs(k):.2f} m'
        return v, dreh, text


def zeichne(view, erg):
    if erg is None:
        return
    h, w = view.shape[:2]
    oben, unten = erg['zeilen']
    cv2.rectangle(view, (0, oben), (w - 1, unten), (255, 120, 0), 1)   # blau: Suchbereich
    cv2.line(view, (w // 2, oben), (w // 2, h), (0, 255, 255), 1)        # gelb: Bildmitte
    for u, v in erg['punkte']:
        cv2.circle(view, (int(u), v), 5, (0, 255, 0), -1)                # gruen: Linienpunkte
    if len(erg['punkte']) >= 2:
        cv2.polylines(view, [np.array(erg['punkte'], np.int32)], False, (0, 0, 255), 2)
