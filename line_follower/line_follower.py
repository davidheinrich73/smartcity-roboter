#!/usr/bin/env python3
# Linienfolger: folgt der schwarzen Linie und gehorcht der KI-Zentrale.
#
# Kamera muss laufen:  scripts/kamera.sh       KI-Zentrale: scripts/ki.sh
# Testmodus (faehrt NICHT): scripts/test.sh    Fahren: scripts/fahren.sh (oder Panel)
#
# Wer entscheidet was?
#   KI-Zentrale (ki/zentrale.py): ob gefahren werden darf (Ampel, Schild, Hindernis ...)
#                                 und wie schnell (Faktor)
#   Linienfolger (diese Datei):   wohin gelenkt wird (Linie), und eine eigene
#                                 LiDAR-Notbremse als zweite Sicherheit
#
# Sicherheit:
#   - Fahrbefehle gehen 20x pro Sekunde raus. Das Motorboard stoppt selbst, wenn
#     0,3 s lang kein Befehl kommt (Watchdog).
#   - Ohne Antwort der KI (ki_pflicht), ohne Kamerabilder oder ohne Linie: Stopp.
#   - Notbremse ueber LiDAR, unabhaengig von der KI.
#
# Einstellungen: -p name:=wert beim Start oder  ros2 param set /line_follower name wert
import json
import os
import signal
import sys
import threading
import time

import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(REPO, 'lib'))
import lidar  # noqa: E402


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        dp = self.declare_parameter
        # ---- Allgemein ----
        dp('drive', False)          # False = nur anschauen, nicht fahren
        dp('show', True)            # Fenster anzeigen (braucht DISPLAY)
        dp('publish_image', True)   # Bild mit Linie fuers Panel senden
        dp('image_topic', '/camera/color/image_raw')
        dp('rate', 20.0)            # Fahrbefehle pro Sekunde (Board-Watchdog: 0,3 s)
        dp('ki_pflicht', True)      # ohne Befehl der KI-Zentrale nicht fahren
        # ---- Linie ----
        dp('speed', 0.15)           # m/s (vorher 0.08 = sehr langsam)
        dp('direction', 1.0)        # -1.0 = andersherum fahren
        dp('steer_gain', 0.004)     # Lenkstaerke, Vorzeichen = Lenkrichtung
        dp('max_turn', 1.0)         # maximale Drehgeschwindigkeit (rad/s)
        dp('threshold', 70)         # dunkler als das = Linie (0 schwarz - 255 weiss)
        dp('strip_start', 0.75)     # Linie nur ab 75 % Bildhoehe (unten) suchen
        dp('min_area', 500)         # kleinere schwarze Flecken ignorieren
        # ---- LiDAR-Notbremse (zweite Sicherheit, unabhaengig von der KI) ----
        # Welche Richtung je LiDAR "vorne" ist: config/roboter.yaml (Panel -> LiDAR)
        dp('obstacle_check', True)
        dp('scan_topics', ['/scan0', '/scan1'])
        dp('scan_front_deg', [0.0, 0.0])
        dp('obstacle_half_deg', 30.0)
        dp('notbremse_dist', 0.12)  # naeher (m) -> sofort Stopp
        dp('obstacle_min_range', 0.08)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.lenkung = None              # (linear, angular) aus dem letzten Bild, None = keine Linie
        self.bild_zeit = 0.0
        self.bild_zaehler, self.fps, self.fps_zeit = 0, 0.0, time.time()
        self.scans = {}
        self.ki = (0.0, None)            # (zeit, befehl)
        self.last_image_pub = 0.0
        self.status_text = 'startet ...'
        self.letzter_cmd = (0.0, 0.0)
        self.anzeige = None              # (Bild, Maske) fuer die Fenster (zeigt der Haupt-Thread)

        # Kamera in eigener Gruppe: eine langsame Bildauswertung blockiert nie die Fahrbefehle
        bild_gruppe = MutuallyExclusiveCallbackGroup()
        rest_gruppe = MutuallyExclusiveCallbackGroup()
        nur_neuestes = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, self.p('image_topic'), self.on_image, nur_neuestes,
                                 callback_group=bild_gruppe)
        for i, t in enumerate(self.p('scan_topics')):
            self.create_subscription(LaserScan, t, lambda m, t=t, i=i: self.on_scan(m, t, i),
                                     qos_profile_sensor_data, callback_group=rest_gruppe)
        self.create_subscription(String, '/ki/befehl', self.on_ki, 10, callback_group=rest_gruppe)
        self.create_subscription(Float32, '/line_follower/tempo', self.on_tempo, 10, callback_group=rest_gruppe)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/line_follower/status', 10)
        self.pub_image = self.create_publisher(CompressedImage, '/line_follower/bild/compressed', 1)
        self.create_timer(1.0 / self.p('rate'), self.steuern, callback_group=rest_gruppe)
        self.get_logger().info('Linienfolger gestartet ' + ('(FAEHRT)' if self.p('drive') else '(TESTMODUS)'))

    def p(self, name):
        return self.get_parameter(name).value

    # ---------------- Eingaenge ----------------
    def on_scan(self, msg, topic, i):
        fronts = self.p('scan_front_deg')
        d = lidar.naechster_vorne(msg, fronts[i] if i < len(fronts) else 0.0, self.p('obstacle_half_deg'),
                                  self.p('obstacle_min_range'), rueckwaerts=self.p('direction') < 0)
        self.scans[topic] = (time.time(), d)

    def on_ki(self, msg):
        try:
            self.ki = (time.time(), json.loads(msg.data))
        except ValueError:
            pass

    def on_tempo(self, msg):
        # Geschwindigkeit vom Panel (Regler). Begrenzt auf 0..0,5 m/s.
        speed = max(0.0, min(0.5, float(msg.data)))
        self.set_parameters([Parameter('speed', value=speed)])

    def notbremse(self):
        """Grund (Text) oder None."""
        if not self.p('obstacle_check'):
            return None
        jetzt = time.time()
        frisch = [(t, d) for t, (z, d) in self.scans.items() if jetzt - z < 1.0]
        if not frisch:
            return 'KEIN LIDAR (obstacle_check:=false zum Abschalten)'
        t, d = min(frisch, key=lambda x: x[1])
        if d < self.p('notbremse_dist'):
            return f'NOTBREMSE {d * 100:.0f} cm ({t})'
        return None

    # ---------------- Kamera: Linie suchen ----------------
    def find_line(self, img, h, w):
        y0 = int(h * self.p('strip_start'))
        gray = cv2.cvtColor(img[y0:h, :], cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(gray, self.p('threshold'), 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(biggest) >= self.p('min_area'):
                m = cv2.moments(biggest)
                if m['m00'] > 0:
                    return y0, mask, biggest, int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
        return y0, mask, None, None, None

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        y0, line_mask, biggest, cx, cy = self.find_line(img, h, w)
        if cx is not None:
            error = (w / 2) - cx  # positiv = Linie liegt links von der Bildmitte
            limit = self.p('max_turn')
            lenkung = (self.p('speed') * self.p('direction'), max(-limit, min(limit, error * self.p('steer_gain'))))
        else:
            lenkung = None
        jetzt = time.time()
        with self.lock:
            self.lenkung, self.bild_zeit = lenkung, jetzt
            self.bild_zaehler += 1
            if jetzt - self.fps_zeit >= 1.0:
                self.fps = self.bild_zaehler / (jetzt - self.fps_zeit)
                self.bild_zaehler, self.fps_zeit = 0, jetzt

        want_pub = self.p('publish_image') and jetzt - self.last_image_pub > 0.1  # max. 10 Bilder/s
        if not (self.p('show') or want_pub):
            return
        view = img.copy()
        cv2.line(view, (0, y0), (w, y0), (255, 0, 0), 2)               # blau: Beginn Suchbereich Linie
        cv2.line(view, (w // 2, y0), (w // 2, h), (0, 255, 255), 1)    # gelb: Bildmitte
        if cx is not None:
            cv2.drawContours(view[y0:h, :], [biggest], -1, (0, 255, 0), 2)  # gruen: erkannte Linie
            cv2.circle(view, (cx, y0 + cy), 8, (0, 0, 255), -1)              # rot: Zielpunkt
        mode = 'FAEHRT' if self.p('drive') else 'TESTMODUS (faehrt nicht)'
        cv2.putText(view, mode, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(view, self.status_text[:70], (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        if want_pub:
            self.last_image_pub = jetzt
            klein = cv2.resize(view, (640, int(h * 640 / w))) if w > 640 else view
            ok, jpg = cv2.imencode('.jpg', klein, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if ok:
                out = CompressedImage(format='jpeg', data=jpg.tobytes())
                out.header = msg.header
                self.pub_image.publish(out)
        if self.p('show'):
            self.anzeige = (view, line_mask)  # Fenster duerfen nur im Haupt-Thread geoeffnet werden

    # ---------------- Fahrbefehl (20x pro Sekunde) ----------------
    def entscheide(self):
        """Rueckgabe: (linear, angular, status)."""
        jetzt = time.time()
        with self.lock:
            lenkung, bild_zeit = self.lenkung, self.bild_zeit
        grund = self.notbremse()
        if grund:
            return 0.0, 0.0, grund + ' -> STOPP'
        if jetzt - bild_zeit > 0.5:
            return 0.0, 0.0, 'KEINE KAMERABILDER -> STOPP'
        ki_zeit, ki = self.ki
        faktor = 1.0
        if self.p('ki_pflicht'):
            if ki is None or jetzt - ki_zeit > 0.6:
                return 0.0, 0.0, 'KI-ZENTRALE ANTWORTET NICHT -> STOPP'
            if ki.get('aktion') == 'stopp':
                return 0.0, 0.0, 'KI: ' + ki.get('grund', 'stopp')
            faktor = float(ki.get('faktor', 1.0))
        if lenkung is None:
            return 0.0, 0.0, 'KEINE LINIE -> STOPP'
        lin, ang = lenkung
        lin *= faktor
        status = f'Linie: Lenkung {ang:+.2f}, {abs(lin):.2f} m/s'
        if ki and ki.get('aktion') == 'langsam':
            status += ' (langsam: ' + ki.get('grund', '') + ')'
        return lin, ang, status

    def steuern(self):
        lin, ang, status = self.entscheide()
        self.status_text = status
        if self.p('drive'):
            t = Twist()
            t.linear.x, t.angular.z = lin, ang
            self.pub.publish(t)
        self.letzter_cmd = (lin, ang)
        self.pub_status.publish(String(data=json.dumps({
            'status': status, 'drive': self.p('drive'), 'linear': round(lin, 3), 'angular': round(ang, 3),
            'fps': round(self.fps, 1), 'speed': self.p('speed'), 'linie': self.lenkung is not None})))
        self.get_logger().info(status, throttle_duration_sec=1.0)


def laeuft_schon(name):
    """Gibt es schon ein Programm mit diesem ROS-Namen? (zwei Linienfolger = zwei Fahrbefehle)"""
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
    if laeuft_schon('line_follower'):
        print('FEHLER: Es laeuft schon ein Linienfolger (scripts/stopp.sh beendet ihn) -> Abbruch.')
        rclpy.shutdown()
        sys.exit(1)
    node = LineFollower()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        while rclpy.ok():
            if node.anzeige is not None:
                view, mask = node.anzeige
                node.anzeige = None
                cv2.imshow('Linienfolger', view)
                cv2.imshow('Maske Linie', mask)
                cv2.waitKey(1)
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.p('drive'):
                for _ in range(5):
                    node.pub.publish(Twist())
                    time.sleep(0.05)
        except Exception:
            pass
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
