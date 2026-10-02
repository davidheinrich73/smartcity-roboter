#!/usr/bin/env python3
# KI-Zentrale: behaelt alles im Blick und sagt dem Linienfolger, ob er fahren darf.
#
# Start: scripts/ki.sh  (das Panel startet sie automatisch)
#
# Sinne:
#   Kamera      -> YOLO-KI (Personen, Autos, Ampeln, Stoppschilder, ...)
#                  + Farbe der gefundenen Ampel (welche Lampe leuchtet)
#                  + Ersatz: LED-Erkennung und Achteck-Form, falls die KI die kleinen
#                    Modell-Ampeln/Schilder nicht als solche erkennt
#   LiDAR       -> Abstand nach vorne
#   Tiefenkamera-> ragt etwas aus dem Boden?
# Die Entscheidung trifft ki/entscheider.py.
#
# Sendet:
#   /ki/befehl              std_msgs/String  JSON {aktion, faktor, grund, ...}  (10x pro Sekunde)
#   /ki/ereignis            std_msgs/String  JSON {zeit, text, art}
#   /ki/bild/compressed     Kamerabild mit allem, was die KI sieht (fuer das Panel)
# Empfaengt:
#   /ki/szenario            std_msgs/String  'normal' | 'einsatz'
import json
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
for _d in ('lib', 'line_follower', 'erkennung', 'ki'):
    sys.path.insert(0, os.path.join(REPO, _d))
import ampel          # noqa: E402
import schilder       # noqa: E402
import lidar          # noqa: E402
import arm as armlib  # noqa: E402
from tiefe import Bodenmodell        # noqa: E402
from entscheider import Entscheider  # noqa: E402

# Diese Dinge fuehren zum Anhalten, wenn sie im Weg sind
STOPP_NAMEN = ['Person', 'Fahrrad', 'Auto', 'Motorrad', 'Bus', 'LKW', 'Hund', 'Katze', 'Teddy',
               'Ball', 'Flasche', 'Tasse', 'Rucksack', 'Koffer', 'Stuhl']
FARBE_BGR = {'rot': (0, 0, 255), 'gelb': (0, 220, 255), 'gruen': (0, 200, 0)}
AKTION_BGR = {'fahren': (60, 170, 60), 'langsam': (0, 170, 230), 'stopp': (40, 40, 220)}


def lade_ampel_werte():
    """Eingestellte Ampelwerte aus config/ampel.yaml (erzeugt mit ampel_kalibrieren.sh)."""
    try:
        import yaml
        with open(os.path.join(REPO, 'config', 'ampel.yaml')) as f:
            daten = yaml.safe_load(f) or {}
        werte = {}
        for teil in daten.values():
            werte.update((teil or {}).get('ros__parameters', {}))
        return {k: v for k, v in werte.items() if k in ampel.STANDARD}
    except FileNotFoundError:
        return {}


class Zentrale(Node):
    def __init__(self):
        super().__init__('ki_zentrale')
        dp = self.declare_parameter
        dp('model', os.path.join(REPO, 'models', 'yolov8n.onnx'))
        dp('imgsz', 320)
        dp('kerne', 2)                  # Prozessorkerne fuer die KI
        dp('rate', 8.0)                 # KI-Bilder pro Sekunde (hoechstens)
        dp('min_conf', 0.35)            # Mindest-Sicherheit der KI
        dp('image_topic', '/camera/color/image_raw')
        dp('depth_topic', '/camera/depth/image_raw')
        dp('tiefe_nutzen', True)
        dp('led_ersatz', True)          # LED-Erkennung, wenn die KI keine Ampel findet
        dp('form_ersatz', True)         # Achteck-Erkennung, wenn die KI kein Stoppschild findet
        dp('ampel_min_hoehe', 0.03)     # KI-Ampel muss mind. so hoch sein (Anteil Bildhoehe) = nah genug
        dp('schild_min_hoehe', 0.12)    # KI-Stoppschild muss mind. so hoch sein = nah genug
        dp('weg_links', 0.2)            # "im Weg" = Kasten mittig ...
        dp('weg_rechts', 0.8)
        dp('weg_unten_min', 0.5)        # ... Unterkante in der unteren Bildhaelfte ...
        dp('weg_min_hoehe', 0.15)       # ... und gross genug (= nah)
        dp('scan_topics', ['/scan0', '/scan1'])
        dp('scan_front_deg', [0.0, 0.0])
        dp('obstacle_half_deg', 30.0)
        dp('obstacle_min_range', 0.08)
        dp('szenario', 'normal')

        einst = armlib.lade_einstellungen()
        self.posen = armlib.lade_posen()
        arm_erlaubt = bool(einst.get('ki_darf_arm_bewegen')) and all(
            self.posen.get(n) for n in ('fahrstellung', 'pruefblick'))
        self.entscheider = Entscheider(arm_erlaubt=arm_erlaubt)
        self.arm = armlib.Arm(self) if arm_erlaubt else None

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.bild = None                # neuestes Kamerabild (ROS-Nachricht)
        self.tiefe = None
        self.scans = {}
        self.wahrnehmung = {'bild_nr': 0, 'bild_zeit': 0.0, 'objekte': [], 'ampel': None,
                            'stoppschild': False, 'tiefe_hindernis': None}
        self.befehl = {'aktion': 'stopp', 'faktor': 0.0, 'grund': 'KI startet ...', 'arm': None}
        self.ki_ms = 0.0
        self.szenario = self.p('szenario')
        self.boden = Bodenmodell()
        self.ampel_werte = lade_ampel_werte()

        self.yolo, self.yolo_fehler = None, ''
        try:
            from yolo import Yolo
            if os.path.exists(self.p('model')):
                self.yolo = Yolo(self.p('model'), self.p('imgsz'), kerne=self.p('kerne'))
            else:
                self.yolo_fehler = 'Modell fehlt: ' + self.p('model')
        except Exception as e:  # z. B. onnxruntime fehlt und OpenCV zu alt
            self.yolo_fehler = f'KI-Modell laesst sich nicht laden: {e}'
        if self.yolo_fehler:
            self.get_logger().error(self.yolo_fehler + '  -> nur Ersatz-Erkennung (LED, Form, Tiefe)')

        nur_neuestes = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.p('image_topic'), self._bild, nur_neuestes)
        if self.p('tiefe_nutzen'):
            self.create_subscription(Image, self.p('depth_topic'), self._tiefe, nur_neuestes)
        for i, t in enumerate(self.p('scan_topics')):
            self.create_subscription(LaserScan, t, lambda m, t=t, i=i: self._scan(m, t, i),
                                     qos_profile_sensor_data)
        self.create_subscription(String, '/ki/szenario', self._szenario, 10)
        self.pub_befehl = self.create_publisher(String, '/ki/befehl', 10)
        self.pub_ereignis = self.create_publisher(String, '/ki/ereignis', 50)
        self.pub_bild = self.create_publisher(CompressedImage, '/ki/bild/compressed', 1)
        self.create_timer(0.1, self._entscheiden)
        self.laeuft = True
        threading.Thread(target=self._ki_schleife, daemon=True).start()
        backend = self.yolo.backend if self.yolo else 'ohne KI-Modell'
        self.get_logger().info(f'KI-Zentrale laeuft ({backend}), Arm erlaubt: {arm_erlaubt}')

    def p(self, name):
        return self.get_parameter(name).value

    # ---------------- Eingaenge ----------------
    def _bild(self, msg):
        with self.lock:
            self.bild = msg

    def _tiefe(self, msg):
        with self.lock:
            self.tiefe = msg

    def _scan(self, msg, topic, i):
        fronts = self.p('scan_front_deg')
        d = lidar.naechster_vorne(msg, fronts[i] if i < len(fronts) else 0.0,
                                  self.p('obstacle_half_deg'), self.p('obstacle_min_range'))
        with self.lock:
            self.scans[topic] = (time.time(), d)

    def _szenario(self, msg):
        if msg.data in ('normal', 'einsatz') and msg.data != self.szenario:
            self.szenario = msg.data
            self._sende_ereignis(time.time(), f'Szenario: {msg.data}', 'info')

    def lidar_abstand(self):
        jetzt = time.time()
        with self.lock:
            frisch = [d for z, d in self.scans.values() if jetzt - z < 1.0]
        return min(frisch) if frisch else None

    # ---------------- Kamera auswerten (eigener Thread) ----------------
    def _ki_schleife(self):
        letzte = None
        while self.laeuft and rclpy.ok():
            start = time.time()
            with self.lock:
                msg, tmsg = self.bild, self.tiefe
            if msg is None or msg is letzte:
                time.sleep(0.02)
                continue
            letzte = msg
            try:
                self._auswerten(msg, tmsg)
            except Exception as e:
                self.get_logger().error(f'KI-Fehler: {e}', throttle_duration_sec=5.0)
            rest = 1.0 / self.p('rate') - (time.time() - start)
            if rest > 0:
                time.sleep(rest)

    def _auswerten(self, msg, tmsg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        t0 = time.time()
        objekte = self.yolo.erkenne(img, self.p('min_conf')) if self.yolo else []

        ampel_farbe, ampel_quelle, schild, schild_quelle = None, None, False, None
        for o in objekte:
            x, y, bw, bh = o['box']
            mitte = (x + bw / 2) / w
            o['im_weg'] = bool(o['name'] in STOPP_NAMEN and self.p('weg_links') <= mitte <= self.p('weg_rechts')
                               and (y + bh) / h >= self.p('weg_unten_min') and bh / h >= self.p('weg_min_hoehe'))
            if o['name'] == 'Ampel' and bh / h >= self.p('ampel_min_hoehe'):
                o['farbe'] = ampel.farbe_in_box(img, o['box'])
                if o['farbe'] and ampel_farbe is None:
                    ampel_farbe, ampel_quelle = o['farbe'], f"KI {o['sicher']:.0%}"
            if o['name'] == 'Stoppschild' and bh / h >= self.p('schild_min_hoehe'):
                schild, schild_quelle = True, f"KI {o['sicher']:.0%}"

        led = None
        if ampel_farbe is None and self.p('led_ersatz'):
            led = ampel.finde_ampel(img, self.ampel_werte)
            if led['farbe']:
                ampel_farbe, ampel_quelle = led['farbe'], 'LED-Erkennung'
        form = None
        if not schild and self.p('form_ersatz'):
            form = schilder.finde_stoppschild(img, {})
            if form['treffer'] is not None:
                schild, schild_quelle = True, 'Form-Erkennung (Achteck)'

        tiefe_hindernis = None
        # Waehrend des Arm-Blicks schaut die Tiefenkamera aus einem anderen Winkel: Das gelernte
        # Bodenbild passt dann nicht -> Tiefenkamera nicht werten (nur die Kamera-KI zaehlt).
        if tmsg is not None and self.entscheider.arm_phase is None:
            d = self.bridge.imgmsg_to_cv2(tmsg, 'passthrough').astype(np.float32)
            if tmsg.encoding != '32FC1':
                d /= 1000.0  # Millimeter -> Meter
            lid = self.lidar_abstand()
            frei = lid is None or lid >= self.entscheider.c['pruef_dist']
            tiefe_hindernis, _ = self.boden.pruefe(d, lernen=frei)

        self.ki_ms = (time.time() - t0) * 1000
        with self.lock:
            nr = self.wahrnehmung['bild_nr'] + 1
            self.wahrnehmung = {'bild_nr': nr, 'bild_zeit': time.time(), 'objekte': objekte,
                                'ampel': ampel_farbe, 'ampel_quelle': ampel_quelle,
                                'stoppschild': schild, 'schild_quelle': schild_quelle,
                                'tiefe_hindernis': tiefe_hindernis}
            befehl = dict(self.befehl)
        self._bild_senden(msg, img, objekte, led, form, ampel_farbe, ampel_quelle, schild, befehl)

    def _bild_senden(self, msg, img, objekte, led, form, ampel_farbe, ampel_quelle, schild, befehl):
        view = img
        h, w = view.shape[:2]
        x0, x1 = int(w * self.p('weg_links')), int(w * self.p('weg_rechts'))
        cv2.rectangle(view, (x0, int(h * self.p('weg_unten_min'))), (x1, h - 1), (255, 0, 255), 1)
        for o in objekte:
            x, y, bw, bh = o['box']
            farbe = (0, 0, 255) if o.get('im_weg') else FARBE_BGR.get(o.get('farbe'), (0, 255, 0))
            cv2.rectangle(view, (x, y), (x + bw, y + bh), farbe, 2)
            text = f"{o['name']} {o['sicher']:.0%}" + (f" {o['farbe']}" if o.get('farbe') else '')
            cv2.putText(view, text, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, farbe, 2)
        if led and led['farbe']:
            x, y, bw, bh = led['lampen'][led['farbe']]['treffer']
            cv2.circle(view, (x + bw // 2, y + bh // 2), max(bw, bh) + 6, FARBE_BGR[led['farbe']], 3)
        if form and form['treffer'] is not None:
            schilder.zeichne(view, form)
        # Kopfzeile: Entscheidung
        farbe = AKTION_BGR.get(befehl['aktion'], (80, 80, 80))
        cv2.rectangle(view, (0, 0), (w, 34), farbe, -1)
        cv2.putText(view, f"{befehl['aktion'].upper()}: {befehl['grund']}"[:70], (8, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        if w > 640:
            view = cv2.resize(view, (640, int(h * 640 / w)))
        ok, jpg = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 65])
        if ok:
            self.pub_bild.publish(CompressedImage(header=msg.header, format='jpeg', data=jpg.tobytes()))

    # ---------------- Entscheiden (10x pro Sekunde) ----------------
    def _entscheiden(self):
        jetzt = time.time()
        with self.lock:
            w = dict(self.wahrnehmung)
        w['lidar'] = self.lidar_abstand()
        befehl = self.entscheider.update(jetzt, w, self.szenario)
        if befehl['arm'] and self.arm is not None:
            fehler = self.arm.fahre(self.posen[befehl['arm']], 1200)
            if fehler:
                self.get_logger().error(f'Arm: {fehler}')
        with self.lock:
            self.befehl = befehl
        for zeit, text, art in self.entscheider.ereignisse:
            self._sende_ereignis(zeit, text, art)
        self.entscheider.ereignisse.clear()
        info = dict(befehl)
        info.update({
            'zeit': jetzt, 'szenario': self.szenario,
            'ki': self.yolo.backend if self.yolo else self.yolo_fehler, 'ki_ms': round(self.ki_ms),
            'bild_alter': round(jetzt - w['bild_zeit'], 2) if w['bild_zeit'] else None,
            'lidar': None if w['lidar'] is None else (99.0 if w['lidar'] == float('inf') else round(w['lidar'], 3)),
            'ampel': w['ampel'], 'ampel_quelle': w.get('ampel_quelle'),
            'stoppschild': w['stoppschild'], 'schild_quelle': w.get('schild_quelle'),
            'tiefe_hindernis': w['tiefe_hindernis'],
            'objekte': [{'name': o['name'], 'sicher': o['sicher'], 'im_weg': o.get('im_weg', False),
                         'farbe': o.get('farbe')} for o in w['objekte']],
            'arm_erlaubt': self.arm is not None,
        })
        self.pub_befehl.publish(String(data=json.dumps(info)))

    def _sende_ereignis(self, zeit, text, art):
        self.get_logger().info(text)
        self.pub_ereignis.publish(String(data=json.dumps({'zeit': zeit, 'text': text, 'art': art})))


def laeuft_schon(name):
    """Gibt es schon ein Programm mit diesem ROS-Namen? (zwei KIs wuerden sich widersprechen)"""
    pruefer = rclpy.create_node('pruefe_' + name + f'_{os.getpid()}')
    time.sleep(1.5)  # ROS braucht kurz, bis es die anderen kennt
    gefunden = name in pruefer.get_node_names()
    pruefer.destroy_node()
    return gefunden


def signale_abfangen():
    """Strg+C, Beenden und Fenster-zu immer als KeyboardInterrupt behandeln (sauberes Ende).
    Noetig, weil mit '&' aus Skripten gestartete Programme Strg+C sonst ignorieren."""
    def ende(*_):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, ende)


def main():
    signale_abfangen()
    rclpy.init()
    if laeuft_schon('ki_zentrale'):
        print('FEHLER: Es laeuft schon eine KI-Zentrale. Zwei wuerden sich widersprechen -> Abbruch.')
        rclpy.shutdown()
        sys.exit(1)
    node = Zentrale()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.laeuft = False
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
