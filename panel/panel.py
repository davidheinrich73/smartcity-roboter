#!/usr/bin/env python3
# Control-Panel fuer den Roboter: Webseite mit grossen Knoepfen.
# Start: scripts/panel.sh  (oeffnet die Seite im Vollbild auf dem Roboter-Display)
#
# Standard: nur auf dem Roboter selbst erreichbar (127.0.0.1).
# Mit --netz auch vom Laptop:  http://<IP-des-Roboters>:8080
#   ACHTUNG: dann kann JEDER im Netz den Roboter starten.
#
# Was das Panel tut:
#   Start/Test/Stopp  -> startet/beendet line_follower.py als eigenen Prozess
#   Stopp             -> beendet den Prozess UND sendet sofort "Stillstand" an /cmd_vel
#   Ansichten         -> Kamera, KI-Erkennung, LiDAR, Tiefenkamera als Live-Bild (MJPEG)
# Der Greifarm wird hier NICHT bewegt (siehe docs/greifarm.md).
import argparse
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
HIER = os.path.dirname(os.path.abspath(__file__))

# Szenarien: Name -> Beschreibung + ROS-Parameter fuer den Linienfolger.
# Neue Szenarien einfach hier ergaenzen.
SZENARIEN = {
    'normal': {'titel': 'Normal', 'text': 'Faehrt der Linie nach, haelt bei Rot, Stoppschild und Hindernis.',
               'params': {'szenario': 'normal'}},
    'einsatz': {'titel': 'RTW-Einsatz', 'text': 'Einsatzfahrt: darf bei Rot und am Stoppschild weiterfahren, '
                'etwas schneller. Notbremse bei Hindernis bleibt AN.',
                'params': {'szenario': 'einsatz'}},
    'langsam': {'titel': 'Langsam', 'text': 'Wie Normal, aber halbe Geschwindigkeit (zum Testen).',
                'params': {'szenario': 'normal', 'speed': 0.04}},
}


# Sensoren fuer die Statusseite: (Anzeigename, Topic, Mindestrate in Hz oder None = nur Info)
def sensorliste(args):
    return [
        ('Akku', '/battery', 0.5),
        ('IMU (Lagesensor)', '/imu/data_raw', 5),
        ('Odometrie (Radzaehler)', '/odom_raw', 5),
        ('LiDAR scan0', '/scan0', 3),
        ('LiDAR scan1', '/scan1', 3),
        ('Kamera Farbe', args.kamera_topic, 5),
        ('Kamera Tiefe', args.tiefe_topic, 5),
        ('Gamepad', '/joy', None),
    ]


# Akku: 3 Li-Ion-Zellen (voll ca. 12,6 V). Prozent ist nur eine grobe SCHAETZUNG.
AKKU_VOLL, AKKU_LEER, AKKU_WARNUNG = 12.6, 10.5, 11.0


def lade_roboter_yaml():
    try:
        import yaml
        with open(os.path.join(REPO, 'config', 'roboter.yaml')) as f:
            return yaml.safe_load(f)['line_follower']['ros__parameters']
    except Exception:
        return {}


class Prozesse:
    """Startet und beendet Programme (Linienfolger, Kamera, KI) als Kindprozesse."""

    def __init__(self):
        self.p = {}
        self.info = {}

    def laeuft(self, name):
        pr = self.p.get(name)
        return pr is not None and pr.poll() is None

    def start(self, name, befehl, info=''):
        if self.laeuft(name):
            return False
        log = open(f'/tmp/panel_{name}.log', 'w')
        # eigene Prozessgruppe, damit beim Beenden auch alle Unterprozesse beendet werden
        self.p[name] = subprocess.Popen(befehl, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.info[name] = info
        return True

    def stopp(self, name, warte=3.0):
        pr = self.p.get(name)
        if pr is None or pr.poll() is not None:
            return
        try:
            os.killpg(pr.pid, signal.SIGINT)   # wie Strg+C
            pr.wait(warte)
        except subprocess.TimeoutExpired:
            os.killpg(pr.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def alle_stoppen(self):
        for name in list(self.p):
            self.stopp(name)


class PanelNode(Node):
    def __init__(self, args):
        super().__init__('control_panel')
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.kamera = (0.0, None)       # (zeit, ROS-Bild)
        self.lf_bild = (0.0, None)      # (zeit, jpeg bytes) vom Linienfolger
        self.ki_bild = (0.0, None)
        self.tiefe = (0.0, None)
        self.scans = {}                 # topic -> (zeit, msg)
        self.lf_status = (0.0, {})
        self.ki_objekte = (0.0, [])
        self.akku = (0.0, None)
        self.zaehler = {}               # topic -> Zeitpunkte der letzten Nachrichten (fuer Hz)
        self.sensoren = sensorliste(args)
        self.agent = (0.0, False)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, args.kamera_topic, lambda m: self._set('kamera', m), 1)
        self.create_subscription(CompressedImage, '/line_follower/bild/compressed',
                                 lambda m: self._set('lf_bild', bytes(m.data)), 1)
        self.create_subscription(CompressedImage, '/erkennung/bild/compressed',
                                 lambda m: self._set('ki_bild', bytes(m.data)), 1)
        self.create_subscription(Image, args.tiefe_topic, lambda m: self._set('tiefe', m), qos_profile_sensor_data)
        self.create_subscription(String, '/line_follower/status', lambda m: self._set('lf_status', json.loads(m.data)), 10)
        self.create_subscription(String, '/erkennung/objekte', lambda m: self._set('ki_objekte', json.loads(m.data)), 10)
        cfg = lade_roboter_yaml()
        self.scan_topics = cfg.get('scan_topics', ['/scan0', '/scan1'])
        self.scan_front = cfg.get('scan_front_deg', [0.0, 0.0])
        self.obst_dist = cfg.get('obstacle_dist', 0.30)
        self.obst_half = cfg.get('obstacle_half_deg', 30.0)
        for t in self.scan_topics:
            self.create_subscription(LaserScan, t, lambda m, t=t: self._scan(t, m), qos_profile_sensor_data)
        self.dyn_subs = {}
        self.create_timer(2.0, self._dynamisch_abonnieren)
        self.create_timer(3.0, self._agent_pruefen)

    def _set(self, name, wert):
        with self.lock:
            setattr(self, name, (time.time(), wert))
        zuordnung = {'kamera': self.args.kamera_topic, 'tiefe': self.args.tiefe_topic}
        if name in zuordnung:
            self.zaehle(zuordnung[name])

    def _scan(self, topic, msg):
        with self.lock:
            self.scans[topic] = (time.time(), msg)
        self.zaehle(topic)

    def zaehle(self, topic):
        jetzt = time.time()
        with self.lock:
            d = self.zaehler.setdefault(topic, deque(maxlen=200))
            d.append(jetzt)

    def hz(self, topic):
        jetzt = time.time()
        with self.lock:
            zeiten = [z for z in self.zaehler.get(topic, ()) if jetzt - z < 2.0]
        return len(zeiten) / 2.0

    def _dynamisch_abonnieren(self):
        # Fuer Sensoren ohne festes Abo (Akku, IMU, Odometrie, Gamepad) den Nachrichtentyp
        # zur Laufzeit nachsehen. So muss man den Typ nicht vorher wissen.
        fest = set(self.scan_topics) | {self.args.kamera_topic, self.args.tiefe_topic}
        try:
            from rosidl_runtime_py.utilities import get_message
            typen = dict(self.get_topic_names_and_types())
            for _, topic, _ in self.sensoren:
                if topic in fest or topic in self.dyn_subs or not typen.get(topic):
                    continue
                typ = get_message(typen[topic][0])
                if topic == '/battery':
                    cb = lambda m: (self.zaehle('/battery'), self._akku(m))
                else:
                    cb = lambda m, t=topic: self.zaehle(t)
                self.dyn_subs[topic] = self.create_subscription(typ, topic, cb, qos_profile_sensor_data)
        except Exception as e:
            self.get_logger().warn(f'Sensor-Abo: {e}', throttle_duration_sec=30)

    def _agent_pruefen(self):
        try:
            ok = subprocess.run(['pgrep', '-f', 'micro_ros_agent'], capture_output=True).returncode == 0
        except Exception:
            ok = False
        self._set('agent', ok)

    def sensor_status(self):
        liste = []
        for name, topic, min_hz in self.sensoren:
            hz = self.hz(topic)
            liste.append({'name': name, 'topic': topic, 'hz': round(hz, 1),
                          'ok': None if min_hz is None else hz >= min_hz})
        knoten = self.get_node_names()
        liste.insert(0, {'name': 'micro-ROS-Agent (Verbindung zum Board)', 'topic': 'Prozess',
                         'hz': None, 'ok': self.agent[1]})
        liste.insert(1, {'name': 'Motorboard (/YB_Node)', 'topic': 'Node', 'hz': None, 'ok': 'YB_Node' in knoten})
        return liste

    def _akku(self, msg):
        wert = getattr(msg, 'data', None)
        if wert is None:
            wert = getattr(msg, 'voltage', None)
        self._set('akku', wert)

    def stillstand(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.02)

    # ---------- Bilder fuer die Ansichten (JPEG) ----------
    def jpeg_kamera(self):
        with self.lock:
            zeit_lf, lf = self.lf_bild
            zeit_k, roh = self.kamera
        if lf is not None and time.time() - zeit_lf < 1.0:
            return lf  # Bild mit Markierungen vom Linienfolger
        if roh is None or time.time() - zeit_k > 2.0:
            return self._text_bild('Keine Kamerabilder. Kamera starten?')
        img = self.bridge.imgmsg_to_cv2(roh, 'bgr8')
        cv2.putText(img, 'Rohbild (Fahrprogramm laeuft nicht)', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()

    def jpeg_ki(self):
        with self.lock:
            zeit, b = self.ki_bild
        if b is None or time.time() - zeit > 2.0:
            return self._text_bild('KI-Erkennung laeuft nicht. Knopf "KI starten".')
        return b

    def jpeg_tiefe(self):
        with self.lock:
            zeit, roh = self.tiefe
        if roh is None or time.time() - zeit > 2.0:
            return self._text_bild(f'Keine Tiefenbilder auf {self.args.tiefe_topic}')
        d = self.bridge.imgmsg_to_cv2(roh, 'passthrough').astype(np.float32)
        if roh.encoding == '32FC1':
            d = d * 1000.0  # Meter -> Millimeter
        nah, fern = 150.0, 2000.0  # mm
        norm = np.clip((d - nah) / (fern - nah), 0, 1)
        bild = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
        bild[d == 0] = 0  # kein Messwert = schwarz
        h, w = d.shape
        mitte = d[h // 2, w // 2]
        cv2.drawMarker(bild, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(bild, f'Mitte: {mitte / 10:.0f} cm   rot = nah, blau = fern', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return cv2.imencode('.jpg', bild, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()

    def jpeg_lidar(self):
        groesse, mpp = 600, 1 / 150.0  # 150 Pixel pro Meter -> +-2 m sichtbar
        bild = np.full((groesse, groesse, 3), 25, np.uint8)
        c = groesse // 2
        for r_m in (0.5, 1.0, 1.5, 2.0):
            cv2.circle(bild, (c, c), int(r_m / mpp), (70, 70, 70), 1)
            cv2.putText(bild, f'{r_m:.1f} m', (c + 4, c - int(r_m / mpp) + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
        # Notbrems-Sektor (vorne = oben)
        r = int(self.obst_dist / mpp)
        cv2.ellipse(bild, (c, c), (r, r), -90, -self.obst_half, self.obst_half, (0, 0, 120), -1)
        cv2.rectangle(bild, (c - 8, c - 12), (c + 8, c + 12), (255, 255, 255), 2)  # Roboter
        cv2.arrowedLine(bild, (c, c), (c, c - 30), (255, 255, 255), 2, tipLength=0.4)
        farben = [(0, 0, 255), (255, 170, 0), (0, 255, 0), (255, 0, 255)]
        jetzt = time.time()
        with self.lock:
            scans = dict(self.scans)
        zeile = 20
        for i, t in enumerate(self.scan_topics):
            front = math.radians(self.scan_front[i] if i < len(self.scan_front) else 0.0)
            zeit, msg = scans.get(t, (0.0, None))
            alt = msg is None or jetzt - zeit > 1.0
            text = f'{t}: ' + ('keine Daten' if alt else f'{len(msg.ranges)} Punkte, vorne = {math.degrees(front):.0f} Grad')
            cv2.putText(bild, text, (10, zeile), cv2.FONT_HERSHEY_SIMPLEX, 0.5, farben[i % 4], 1)
            zeile += 20
            if alt:
                continue
            r_arr = np.array(msg.ranges, np.float32)
            a_arr = msg.angle_min + np.arange(len(r_arr)) * msg.angle_increment - front
            gut = np.isfinite(r_arr) & (r_arr > msg.range_min) & (r_arr < msg.range_max)
            x = r_arr[gut] * np.cos(a_arr[gut])   # vorne
            y = r_arr[gut] * np.sin(a_arr[gut])   # links
            px = (c - y / mpp).astype(int)
            py = (c - x / mpp).astype(int)
            innen = (px >= 0) & (px < groesse) & (py >= 0) & (py < groesse)
            bild[py[innen], px[innen]] = farben[i % 4]
        cv2.putText(bild, 'oben = vorne (laut config/roboter.yaml)   dunkelrot = Notbrems-Bereich',
                    (10, groesse - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        return cv2.imencode('.jpg', bild, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()

    @staticmethod
    def _text_bild(text):
        bild = np.full((480, 640, 3), 40, np.uint8)
        cv2.putText(bild, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return cv2.imencode('.jpg', bild)[1].tobytes()


def topics_mit(woerter):
    """ros2 topic list, gefiltert (fuer die Greifarm-Diagnose). Nur lesen."""
    try:
        aus = subprocess.run(['ros2', 'topic', 'list', '-t'], capture_output=True, text=True, timeout=10).stdout
    except Exception as e:
        return str(e)
    zeilen = [z for z in aus.splitlines() if any(w in z.lower() for w in woerter)]
    return '\n'.join(zeilen) or 'keine passenden Topics gefunden'


def mache_handler(node, prozesse, beenden=None):
    lf_skript = os.path.join(REPO, 'line_follower', 'line_follower.py')

    def lf_befehl(drive, szenario):
        cmd = ['python3', lf_skript, '--ros-args', '--params-file', os.path.join(REPO, 'config', 'roboter.yaml')]
        ampel = os.path.join(REPO, 'config', 'ampel.yaml')
        if os.path.exists(ampel):
            cmd += ['--params-file', ampel]
        cmd += ['-p', f'drive:={"true" if drive else "false"}', '-p', 'show:=false']
        for k, v in SZENARIEN[szenario]['params'].items():
            cmd += ['-p', f'{k}:={v}']
        return cmd

    def aktion(name, daten):
        if name == 'stopp':
            node.stillstand()                 # sofort Stillstand senden ...
            prozesse.stopp('fahren')          # ... dann Programm beenden
            node.stillstand()
            return 'Gestoppt'
        if name in ('start', 'test'):
            sz = daten.get('szenario', 'normal')
            if sz not in SZENARIEN:
                return f'Unbekanntes Szenario {sz}'
            if prozesse.laeuft('fahren'):
                return 'Laeuft schon. Erst Stopp druecken.'
            prozesse.start('fahren', lf_befehl(name == 'start', sz),
                           f"{'FAEHRT' if name == 'start' else 'TESTMODUS'} - {SZENARIEN[sz]['titel']}")
            return 'Gestartet'
        if name == 'kamera_start':
            if node.count_publishers(node.args.kamera_topic) > 0:
                return 'Kamera laeuft schon'
            prozesse.start('kamera', ['bash', os.path.join(REPO, 'scripts', 'kamera.sh')])
            return 'Kamera wird gestartet (ca. 10 s)'
        if name == 'ki_start':
            prozesse.start('ki', ['python3', os.path.join(REPO, 'erkennung', 'objekte.py')])
            return 'KI wird gestartet. Log: /tmp/panel_ki.log'
        if name == 'ki_stopp':
            prozesse.stopp('ki')
            return 'KI gestoppt'
        if name == 'update':
            if prozesse.laeuft('fahren'):
                return 'Erst STOPP druecken.'
            r = subprocess.run(['git', '-C', REPO, 'pull'], capture_output=True, text=True, timeout=60)
            return (r.stdout + r.stderr).strip() + '\nPanel neu starten, damit Aenderungen gelten.'
        if name == 'beenden' and beenden:
            threading.Thread(target=beenden, daemon=True).start()
            return 'Panel wird beendet'
        if name == 'arm_diagnose':
            return topics_mit(['arm', 'servo', 'joint', 'grip', 'claw'])
        return f'Unbekannte Aktion {name}'

    def status():
        jetzt = time.time()
        with node.lock:
            zs, st = node.lf_status
            za, akku = node.akku
            zo, obj = node.ki_objekte
            zk = node.kamera[0]
            scans = {t: jetzt - z < 1.0 for t, (z, _) in node.scans.items()}
        return {
            'fahren': prozesse.laeuft('fahren'), 'fahren_info': prozesse.info.get('fahren', ''),
            'ki': prozesse.laeuft('ki'),
            'status': st.get('status', '') if jetzt - zs < 1.5 else '',
            'akku': akku if jetzt - za < 5 else None,
            'akku_prozent': None if akku is None or jetzt - za >= 5 else
            max(0, min(100, round((akku - AKKU_LEER) / (AKKU_VOLL - AKKU_LEER) * 100))),
            'akku_warnung': AKKU_WARNUNG,
            'sensoren': node.sensor_status(),
            'kamera': jetzt - zk < 2.0,
            'lidar': scans,
            'objekte': obj if jetzt - zo < 2.0 else [],
            'szenarien': {k: {'titel': v['titel'], 'text': v['text']} for k, v in SZENARIEN.items()},
        }

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass  # keine Zeile pro Anfrage ins Terminal

        def _senden(self, code, typ, daten):
            self.send_response(code)
            self.send_header('Content-Type', typ)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(daten)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                with open(os.path.join(HIER, 'index.html'), 'rb') as f:
                    self._senden(200, 'text/html; charset=utf-8', f.read())
            elif self.path == '/api/status':
                self._senden(200, 'application/json', json.dumps(status()).encode())
            elif self.path.startswith('/stream/'):
                quelle = {'kamera': node.jpeg_kamera, 'ki': node.jpeg_ki, 'lidar': node.jpeg_lidar,
                          'tiefe': node.jpeg_tiefe}.get(self.path.split('/')[2].split('?')[0])
                if quelle is None:
                    return self._senden(404, 'text/plain', b'?')
                # MJPEG: ein "Video" aus vielen JPEG-Bildern hintereinander
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=bild')
                self.end_headers()
                try:
                    while True:
                        jpg = quelle()
                        self.wfile.write(b'--bild\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._senden(404, 'text/plain', b'Nicht gefunden')

        def do_POST(self):
            if not self.path.startswith('/api/'):
                return self._senden(404, 'text/plain', b'?')
            laenge = int(self.headers.get('Content-Length', 0))
            try:
                daten = json.loads(self.rfile.read(laenge) or b'{}')
            except ValueError:
                daten = {}
            antwort = aktion(self.path[5:], daten)
            self._senden(200, 'application/json', json.dumps({'antwort': antwort}).encode())

    return Handler


def main():
    ap = argparse.ArgumentParser(description='Control-Panel')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--netz', action='store_true', help='auch aus dem Netzwerk erreichbar (Vorsicht!)')
    ap.add_argument('--kamera-topic', default='/camera/color/image_raw')
    ap.add_argument('--tiefe-topic', default='/camera/depth/image_raw')
    ap.add_argument('--autostart', action='store_true', help='Kamera und KI beim Start mitstarten')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = PanelNode(args)
    prozesse = Prozesse()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    adresse = '0.0.0.0' if args.netz else '127.0.0.1'
    server = None

    def beenden():
        time.sleep(0.3)
        server.shutdown()
    server = ThreadingHTTPServer((adresse, args.port), mache_handler(node, prozesse, beenden))
    server.daemon_threads = True
    print(f'Control-Panel: http://{"<IP-des-Roboters>" if args.netz else "localhost"}:{args.port}  (Strg+C = Ende)')
    if args.netz:
        print('ACHTUNG: Panel ist im ganzen Netz erreichbar. Jeder kann den Roboter starten.')
    if args.autostart:
        time.sleep(2.0)  # kurz warten, bis ROS die anderen Nodes kennt
        if node.count_publishers(args.kamera_topic) == 0:
            print('Starte Kamera ...')
            prozesse.start('kamera', ['bash', os.path.join(REPO, 'scripts', 'kamera.sh')])
        if os.path.exists(os.path.join(REPO, 'models', 'yolov8n.onnx')):
            print('Starte KI-Erkennung ...')
            prozesse.start('ki', ['python3', os.path.join(REPO, 'erkennung', 'objekte.py')])
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # Beim Beenden des Panels: Roboter anhalten und alle gestarteten Programme beenden
        node.stillstand()
        prozesse.alle_stoppen()
        node.stillstand()
        server.server_close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
