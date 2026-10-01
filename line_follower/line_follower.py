#!/usr/bin/env python3
# Linienfolger (schwarze Linie) + Ampel + Stoppschild + LiDAR-Notbremse
# Kamera muss laufen:  scripts/kamera.sh
# Testmodus (faehrt NICHT): scripts/test.sh
# Fahren:                   scripts/fahren.sh   (oder Control-Panel)
#
# Alle Einstellungen koennen beim Start mit  -p name:=wert  oder waehrend des
# Laufens mit  ros2 param set /line_follower name wert  geaendert werden.
#
# Wann haelt er an? (wichtigstes zuerst)
#   1. LiDAR sieht etwas zu nah vor dem Roboter (oder LiDAR liefert nichts)
#   2. Objekterkennung (erkennung/objekte.py) meldet Fussgaenger/Auto im Weg
#   3. Ampel rot              (nicht im Szenario "einsatz")
#   4. Stoppschild: kurz halten, dann weiter  (nicht im Szenario "einsatz")
#   5. keine Linie / keine Kamerabilder
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2

import ampel     # liegen im selben Ordner
import schilder


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        # ---- Allgemein ----
        self.declare_parameter('drive', False)        # False = nur anschauen, nicht fahren
        self.declare_parameter('show', True)          # Fenster anzeigen (braucht DISPLAY)
        self.declare_parameter('publish_image', True)  # Bild mit Markierungen fuers Panel senden
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('szenario', 'normal')  # normal | einsatz (RTW: darf bei Rot fahren)
        # ---- Linie ----
        self.declare_parameter('speed', 0.08)         # m/s
        self.declare_parameter('einsatz_faktor', 1.3)  # Geschwindigkeit im Einsatz = speed * faktor
        self.declare_parameter('direction', 1.0)      # -1.0 = andersherum fahren
        self.declare_parameter('steer_gain', 0.004)   # Lenkstaerke, Vorzeichen = Lenkrichtung
        self.declare_parameter('max_turn', 0.8)       # maximale Drehgeschwindigkeit (rad/s)
        self.declare_parameter('threshold', 70)       # dunkler als das = Linie (0 schwarz - 255 weiss)
        self.declare_parameter('strip_start', 0.75)   # Linie nur ab 75 % Bildhoehe (unten) suchen
        self.declare_parameter('min_area', 500)       # kleinere schwarze Flecken ignorieren
        # ---- Ampel ----
        # Erkennung steht in ampel.py. Alle Werte dort in STANDARD, hier als
        # ROS-Parameter verfuegbar. Eingestellte Werte: config/ampel.yaml
        self.declare_parameter('red_check', True)     # Ampelerkennung an/aus
        self.declare_parameter('red_frames', 3)       # so viele Bilder rot hintereinander -> Stopp
        for name, wert in ampel.STANDARD.items():
            self.declare_parameter(name, wert)
        # ---- Stoppschild (schilder.py) ----
        self.declare_parameter('sign_check', True)
        self.declare_parameter('sign_frames', 3)      # so viele Bilder hintereinander -> Stopp
        self.declare_parameter('sign_wait', 3.0)      # so lange stehen bleiben (s)
        self.declare_parameter('sign_cooldown', 6.0)  # danach so lange Schilder ignorieren (vorbeifahren)
        for name, wert in schilder.STANDARD.items():
            self.declare_parameter(name, wert)
        # ---- LiDAR-Notbremse ----
        # Welche Richtung bei jedem LiDAR "vorne" ist, ist NOCH NICHT GEPRUEFT.
        # Einstellen: Panel -> LiDAR-Ansicht, Hand vor den Roboter halten.
        self.declare_parameter('obstacle_check', True)
        self.declare_parameter('scan_topics', ['/scan0', '/scan1'])
        self.declare_parameter('scan_front_deg', [0.0, 0.0])  # Winkel "vorne" je LiDAR (Grad)
        self.declare_parameter('obstacle_half_deg', 30.0)    # Sektor vorne: +- so viel Grad
        self.declare_parameter('obstacle_dist', 0.30)        # naeher als das (m) -> Stopp
        self.declare_parameter('obstacle_min_range', 0.08)   # naeher als das = eigener Roboter, ignorieren
        # ---- Objekterkennung (erkennung/objekte.py, optional) ----
        self.declare_parameter('object_check', True)

        self.red_hits = 0
        self.red_stop = False
        self.sign_hits = 0
        self.sign_stop_until = 0.0
        self.sign_ignore_until = 0.0
        self.last_image = time.time()
        self.scans = {}            # topic -> (zeit, naechster Abstand vorne)
        self.object_block = (0.0, False, '')
        self.bridge = CvBridge()
        topic = self.p('image_topic')
        self.sub = self.create_subscription(Image, topic, self.on_image, 10)
        for i, t in enumerate(self.p('scan_topics')):
            self.create_subscription(LaserScan, t, lambda msg, t=t, i=i: self.on_scan(msg, t, i),
                                     qos_profile_sensor_data)
        self.create_subscription(Bool, '/erkennung/hindernis', self.on_object, 10)
        self.create_subscription(String, '/erkennung/objekte', self.on_object_info, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/line_follower/status', 10)
        self.pub_image = self.create_publisher(CompressedImage, '/line_follower/bild/compressed', 1)
        self.last_image_pub = 0.0
        self.object_info = ''
        # Sicherheit: kommen keine Kamerabilder mehr, Stopp senden
        self.create_timer(0.5, self.watchdog)
        self.get_logger().info(f'Linienfolger gestartet. Warte auf Bilder von {topic} ...')

    def p(self, name):
        return self.get_parameter(name).value

    def watchdog(self):
        if self.p('drive') and time.time() - self.last_image > 1.0:
            self.pub.publish(Twist())
            self.get_logger().warn('Keine Kamerabilder -> Stopp', throttle_duration_sec=2.0)
            self.send_status('KEINE KAMERABILDER -> Stopp', 0.0, 0.0)

    # ---------- LiDAR ----------
    def on_scan(self, msg, topic, index):
        fronts = self.p('scan_front_deg')
        front = math.radians(fronts[index] if index < len(fronts) else 0.0)
        if self.p('direction') < 0:
            front += math.pi  # rueckwaerts fahren -> hinten ist "vorne"
        half = math.radians(self.p('obstacle_half_deg'))
        lo = max(self.p('obstacle_min_range'), msg.range_min)
        nearest = float('inf')
        for i, r in enumerate(msg.ranges):
            if not (lo < r < msg.range_max) or math.isinf(r) or math.isnan(r):
                continue
            a = msg.angle_min + i * msg.angle_increment
            diff = math.atan2(math.sin(a - front), math.cos(a - front))
            if abs(diff) <= half and r < nearest:
                nearest = r
        self.scans[topic] = (time.time(), nearest)

    def obstacle(self):
        """Rueckgabe: Grund (Text) oder None."""
        if not self.p('obstacle_check'):
            return None
        now = time.time()
        fresh = [(t, d) for t, (zeit, d) in self.scans.items() if now - zeit < 1.0]
        if not fresh:
            return 'KEIN LIDAR (obstacle_check:=false zum Abschalten)'
        t, d = min(fresh, key=lambda x: x[1])
        if d < self.p('obstacle_dist'):
            return f'HINDERNIS {d * 100:.0f} cm ({t})'
        return None

    # ---------- Objekterkennung ----------
    def on_object(self, msg):
        self.object_block = (time.time(), msg.data)

    def on_object_info(self, msg):
        self.object_info = msg.data

    def object_stop(self):
        zeit, block = self.object_block[:2]
        if self.p('object_check') and block and time.time() - zeit < 0.7:
            try:
                namen = [o['name'] for o in json.loads(self.object_info) if o.get('im_weg')]
            except Exception:
                namen = []
            return 'OBJEKT IM WEG: ' + (', '.join(namen) or '?')
        return None

    # ---------- Kamera ----------
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

    def update_sign(self, sign_box):
        """Stoppschild: Rueckgabe True, solange gewartet wird."""
        now = time.time()
        if now < self.sign_stop_until:
            return True
        if now < self.sign_ignore_until or sign_box is None:
            self.sign_hits = 0
            return False
        self.sign_hits += 1
        if self.sign_hits >= self.p('sign_frames'):
            self.sign_hits = 0
            self.sign_stop_until = now + self.p('sign_wait')
            self.sign_ignore_until = self.sign_stop_until + self.p('sign_cooldown')
            return True
        return False

    def send_status(self, status, lin, ang):
        self.pub_status.publish(String(data=json.dumps({
            'status': status, 'drive': self.p('drive'), 'szenario': self.p('szenario'),
            'linear': round(lin, 3), 'angular': round(ang, 3)})))

    def on_image(self, msg):
        self.last_image = time.time()
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        einsatz = self.p('szenario') == 'einsatz'

        y0, line_mask, biggest, cx, cy = self.find_line(img, h, w)

        red, red_box = None, None
        if self.p('red_check'):
            red = ampel.finde_rot(img, {name: self.p(name) for name in ampel.STANDARD})
            red_box = red['treffer']
        sign = None
        if self.p('sign_check'):
            sign = schilder.finde_stoppschild(img, {name: self.p(name) for name in schilder.STANDARD})

        # Entprellen: erst nach mehreren roten Bildern stoppen, erst ohne Rot wieder fahren
        if red_box is not None:
            self.red_hits = min(self.red_hits + 1, 10)
        else:
            self.red_hits = max(self.red_hits - 1, 0)
        if self.red_hits >= self.p('red_frames'):
            self.red_stop = True
        elif self.red_hits == 0:
            self.red_stop = False
        sign_stop = self.update_sign(sign['treffer'] if sign else None)

        twist = Twist()
        grund = self.obstacle() or self.object_stop()
        if grund:
            status = grund + ' -> STOPP'
        elif self.red_stop and not einsatz:
            status = 'AMPEL ROT -> STOPP'
        elif sign_stop and not einsatz:
            status = f'STOPPSCHILD -> warte {max(0.0, self.sign_stop_until - time.time()):.1f} s'
        elif cx is not None:
            error = (w / 2) - cx  # positiv = Linie liegt links von der Bildmitte
            turn = error * self.p('steer_gain')
            limit = self.p('max_turn')
            speed = self.p('speed') * (self.p('einsatz_faktor') if einsatz else 1.0)
            twist.linear.x = speed * self.p('direction')
            twist.angular.z = max(-limit, min(limit, turn))
            status = f'Linie x={cx}  Abweichung={error:+.0f}  Lenkung={twist.angular.z:+.2f}'
            if einsatz:
                status = 'EINSATZ  ' + status + ('  (Rot ignoriert)' if self.red_stop else '')
        else:
            status = 'KEINE LINIE -> Stopp'

        if self.p('drive'):
            self.pub.publish(twist)
        self.send_status(status, twist.linear.x, twist.angular.z)
        self.get_logger().info(status, throttle_duration_sec=0.5)

        want_pub = self.p('publish_image') and time.time() - self.last_image_pub > 0.1  # max. 10 Bilder/s
        if not (self.p('show') or want_pub):
            return
        view = img.copy()
        cv2.line(view, (0, y0), (w, y0), (255, 0, 0), 2)               # blau: Beginn Suchbereich Linie
        cv2.line(view, (w // 2, y0), (w // 2, h), (0, 255, 255), 1)    # gelb: Bildmitte
        if cx is not None:
            cv2.drawContours(view[y0:h, :], [biggest], -1, (0, 255, 0), 2)  # gruen: erkannte Linie
            cv2.circle(view, (cx, y0 + cy), 8, (0, 0, 255), -1)              # rot: Zielpunkt
        if red is not None:
            ampel.zeichne(view, red)  # lila Suchbereich, orange verworfen, rot = Ampel
        if sign is not None:
            schilder.zeichne(view, sign)
        mode = 'FAEHRT' if self.p('drive') else 'TESTMODUS (faehrt nicht)'
        cv2.putText(view, mode, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(view, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        if want_pub:
            self.last_image_pub = time.time()
            ok, jpg = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                out = CompressedImage(format='jpeg', data=jpg.tobytes())
                out.header = msg.header
                self.pub_image.publish(out)
        if self.p('show'):
            cv2.imshow('Linienfolger', view)
            cv2.imshow('Maske Linie', line_mask)
            if red is not None and red['maske'] is not None:
                cv2.imshow('Maske Rot', red['maske'])
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = LineFollower()
    try:
        rclpy.spin(node)
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
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
