#!/usr/bin/env python3
# LiDAR-Auswertung (ohne ROS-Abhaengigkeit, damit testbar).
#
# Ein LiDAR misst in einer DUENNEN WAAGRECHTEN SCHEIBE auf seiner eigenen Hoehe
# den Abstand rundherum. Ein Schuh erscheint deshalb nur als kurzer Bogen
# (seine Vorderkante), nicht als Flaeche. Was niedriger oder hoeher als die
# Scheibe ist, sieht er gar nicht.
import math

import numpy as np


def winkel_und_abstand(msg, front_deg=0.0):
    """Gueltige Messpunkte eines LaserScan.

    Rueckgabe: (winkel, abstand) als numpy-Arrays. Winkel in Bogenmass relativ
    zu "vorne" (0 = vorne, positiv = links), Abstand in Metern.
    """
    r = np.asarray(msg.ranges, dtype=np.float32)
    a = msg.angle_min + np.arange(len(r), dtype=np.float32) * msg.angle_increment - math.radians(front_deg)
    gut = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
    a = np.arctan2(np.sin(a[gut]), np.cos(a[gut]))  # auf -pi..pi bringen
    return a, r[gut]


def naechster_vorne(msg, front_deg=0.0, halb_deg=30.0, min_abstand=0.08, rueckwaerts=False):
    """Kleinster Abstand im Sektor vorne (+- halb_deg). inf = nichts gesehen.

    min_abstand: noch naeher = Teile des eigenen Roboters, ignorieren.
    rueckwaerts: True = Sektor hinten pruefen (Roboter faehrt rueckwaerts).
    """
    a, r = winkel_und_abstand(msg, front_deg + (180.0 if rueckwaerts else 0.0))
    im_sektor = (np.abs(a) <= math.radians(halb_deg)) & (r > min_abstand)
    return float(r[im_sektor].min()) if np.any(im_sektor) else float('inf')


def punkte(msg, front_deg=0.0, max_punkte=360):
    """Punkte fuer die Anzeige: Liste [x, y] in Metern, x = vorne, y = links."""
    a, r = winkel_und_abstand(msg, front_deg)
    if len(r) > max_punkte:
        schritt = int(math.ceil(len(r) / max_punkte))
        a, r = a[::schritt], r[::schritt]
    x, y = r * np.cos(a), r * np.sin(a)
    return [[round(float(px), 3), round(float(py), 3)] for px, py in zip(x, y)]


def naechstes_objekt(msg, front_deg=0.0, halb_deg=30.0, min_abstand=0.08):
    """Naechster Gegenstand vorne: dict vor, seite (m, Mitte), breite (m) oder None.
    Breite = Punkte, die hoechstens 4 cm hinter dem naechsten Punkt liegen."""
    a, r = winkel_und_abstand(msg, front_deg)
    im_sektor = (np.abs(a) <= math.radians(halb_deg)) & (r > min_abstand)
    if not np.any(im_sektor):
        return None
    a, r = a[im_sektor], r[im_sektor]
    i = int(np.argmin(r))
    x, y = r * np.cos(a), r * np.sin(a)
    teil = np.hypot(x - x[i], y - y[i]) < 0.04
    return {'vor': float(x[i]), 'seite': float(np.mean(y[teil])),
            'breite': float(np.ptp(y[teil]) + 0.01), 'abstand': float(r[i])}
