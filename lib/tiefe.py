#!/usr/bin/env python3
# Tiefenkamera: Ragt etwas aus dem Boden vor dem Roboter?
#
# Die Kamera schaut schraeg auf die Strasse. Sie misst also immer den Boden in
# einem bestimmten Abstand. Deshalb lernt dieses Modul, wie der Boden "normal"
# aussieht (Abstand je Bildbereich), solange der LiDAR vorne nichts meldet.
# Ist ein Bereich spaeter deutlich NAEHER als gelernt, steht dort etwas.
# UNGETESTET mit der echten Kamera.
import numpy as np


class Bodenmodell:
    def __init__(self, spalten=8, zeilen=6, links=0.25, rechts=0.75, oben=0.3,
                 differenz=0.06, min_zellen=2, lern_bilder=10, lernrate=0.1):
        self.spalten, self.zeilen = spalten, zeilen
        self.links, self.rechts, self.oben = links, rechts, oben
        self.differenz = differenz        # so viel naeher als der Boden (m) = Hindernis
        self.min_zellen = min_zellen      # so viele Bereiche muessen naeher sein
        self.lern_bilder = lern_bilder    # erst danach ist das Modell bereit
        self.lernrate = lernrate
        self.boden = None
        self.gelernt = 0

    def _zellen(self, tiefe_m):
        """Median-Abstand je Zelle im Fahrweg (unterer, mittlerer Bildteil). 0 = keine Messung."""
        h, w = tiefe_m.shape
        y0, x0, x1 = int(h * self.oben), int(w * self.links), int(w * self.rechts)
        teil = tiefe_m[y0:, x0:x1]
        th, tw = teil.shape
        werte = np.zeros((self.zeilen, self.spalten), np.float32)
        for zi in range(self.zeilen):
            for si in range(self.spalten):
                z = teil[zi * th // self.zeilen:(zi + 1) * th // self.zeilen,
                         si * tw // self.spalten:(si + 1) * tw // self.spalten]
                gueltig = z[z > 0.05]
                if gueltig.size > z.size * 0.3:
                    werte[zi, si] = np.median(gueltig)
        return werte

    def pruefe(self, tiefe_m, lernen):
        """tiefe_m: Tiefenbild in Metern (0 = keine Messung).
        lernen: True, wenn der Weg frei ist (LiDAR meldet nichts) -> Boden merken.
        Rueckgabe: (hindernis True/False oder None = noch nicht bereit, naechster Abstand in m oder None)
        """
        z = self._zellen(tiefe_m)
        if self.boden is None:
            self.boden = z.copy()
        bereit = self.gelernt >= self.lern_bilder
        hindernis, naechster = None, None
        if bereit:
            beide = (z > 0) & (self.boden > 0)
            naeher = beide & (z < self.boden - self.differenz)
            hindernis = int(naeher.sum()) >= self.min_zellen
            if hindernis:
                naechster = float(z[naeher].min())
        if lernen and not hindernis:
            neu = (self.boden == 0) & (z > 0)
            self.boden[neu] = z[neu]
            beide = (self.boden > 0) & (z > 0)
            self.boden[beide] += self.lernrate * (z[beide] - self.boden[beide])
            self.gelernt += 1
        return hindernis, naechster
