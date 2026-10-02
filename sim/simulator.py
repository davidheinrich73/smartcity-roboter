#!/usr/bin/env python3
# Simulator: ersetzt Roboter, Kamera und LiDAR, damit man ALLES ohne Roboter testen kann.
# Braucht ROS 2 (z. B. auf einem Laptop mit ROS 2 Humble). FAEHRT KEINEN ECHTEN ROBOTER:
# er benutzt ROS_DOMAIN_ID aus der Umgebung -> NIE mit Domain 30 im Roboter-Netz starten!
#   scripts/simulation.sh  (setzt eine eigene Domain-ID)
#
# Die Strecke ist ein Rundkurs von 6 m. Ereignisse nach gefahrener Strecke s (Meter):
#   0.6 - 1.0  Ampel (wird rot, sobald der Roboter in die Naehe kommt; nach 3 s Stand gruen)
#   2.0 - 2.4  Stoppschild
#   3.2 - 3.6  Hindernis auf der Fahrbahn (LiDAR + Tiefenkamera), geht nach 3 s Stand weg
#   4.5 - 5.0  Haus am Strassenrand: nur der LiDAR sieht es, sonst nichts
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

RUNDE = 6.0
AMPEL = (0.6, 1.0)
SCHILD = (2.0, 2.4)
HINDERNIS = (3.2, 3.6)
HAUS = (4.5, 5.0)


class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        self.bridge = CvBridge()
        self.s = 0.0            # gefahrene Strecke in der Runde (m)
        self.gesamt = 0.0
        self.e = 0.0            # seitliche Abweichung von der Linie
        self.v, self.w = 0.0, 0.0
        self.cmd_zeit = 0.0
        self.stand_seit = None
        self.ampel = 'aus'      # aus | rot | gruen
        self.hindernis_da = True
        self.runde = 0
        self.create_subscription(Twist, '/cmd_vel', self.cmd, 10)
        self.pub_bild = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.pub_tiefe = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.pub_info_c = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.pub_info_d = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.pub_scan = [self.create_publisher(LaserScan, t, qos_profile_sensor_data) for t in ('/scan0', '/scan1')]
        self.pub_akku = self.create_publisher(Float32, '/battery', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_raw', 10)
        self.pub_zustand = self.create_publisher(String, '/sim/zustand', 10)
        self.letzte = time.time()
        self.create_timer(0.05, self.physik)
        self.create_timer(1 / 15, self.kamera)
        self.create_timer(0.1, self.tiefe)
        self.create_timer(1 / 7, self.lidar)
        self.create_timer(1.0, lambda: self.pub_akku.publish(Float32(data=12.3)))
        self.create_timer(0.05, self.imu_odom)
        self.get_logger().info('Simulator laeuft')

    def cmd(self, msg):
        self.v, self.w, self.cmd_zeit = msg.linear.x, msg.angular.z, time.time()

    # ---------------- Welt ----------------
    def physik(self):
        jetzt = time.time()
        dt = jetzt - self.letzte
        self.letzte = jetzt
        if jetzt - self.cmd_zeit > 0.3:     # Watchdog wie beim echten Board
            self.v, self.w = 0.0, 0.0
        kruemmung = 0.8 * math.sin(self.s * 2 * math.pi / 3.0)  # Kurven
        self.e += (kruemmung * self.v + self.w) * dt
        self.e = max(-1.0, min(1.0, self.e))
        self.s += self.v * dt
        self.gesamt += self.v * dt
        if self.s >= RUNDE:
            self.s -= RUNDE
            self.runde += 1
            self.ampel, self.hindernis_da = 'aus', True
        steht = abs(self.v) < 0.005
        self.stand_seit = (self.stand_seit or jetzt) if steht else None
        stand = jetzt - self.stand_seit if self.stand_seit else 0.0
        # Ampel: rot, wenn der Roboter kommt; gruen nach 3 s Stand (wie die Leitstelle)
        if AMPEL[0] - 0.3 <= self.s <= AMPEL[1]:
            if self.ampel == 'aus':
                self.ampel = 'rot'
            elif self.ampel == 'rot' and stand > 3.0:
                self.ampel = 'gruen'
        elif self.s > AMPEL[1]:
            self.ampel = 'aus'
        # Hindernis geht weg, wenn der Roboter 3 s davor gestanden hat
        if self.hindernis_da and HINDERNIS[0] - 0.3 <= self.s <= HINDERNIS[1] and stand > 3.0:
            self.hindernis_da = False
        self.pub_zustand.publish(String(data=json.dumps({
            'zeit': jetzt, 's': round(self.s, 3), 'gesamt': round(self.gesamt, 3), 'runde': self.runde,
            'v': round(self.v, 3), 'w': round(self.w, 3), 'e': round(self.e, 3), 'ampel': self.ampel,
            'hindernis': self.hindernis_abstand()})))

    def hindernis_abstand(self):
        """Abstand zum Hindernis vor dem Roboter (m) oder None."""
        if self.hindernis_da and self.s <= HINDERNIS[1]:
            d = HINDERNIS[1] - self.s
            if d < 1.5:
                return d
        return None

    def naehe(self, bereich):
        """0 (weit weg) .. 1 (direkt davor) fuer Dinge, die bei bereich[1] stehen."""
        d = bereich[1] - self.s
        if d < 0 or d > 1.2:
            return None
        return 1.0 - d / 1.2

    # ---------------- Sensoren ----------------
    def kamera(self):
        h, w = 480, 640
        img = np.full((h, w, 3), 205, np.uint8)
        cv2.randn(img, (205, 205, 205), (6, 6, 6))
        # Linie: unten bei der Abweichung, nach oben zur Kurve hin
        unten = int(w / 2 + self.e * 300)
        oben = int(unten + 160 * math.sin((self.s + 0.5) * 2 * math.pi / 3.0))
        cv2.line(img, (unten, h), (oben, int(h * 0.45)), (25, 25, 25), 34)
        # Ampel: schwarzes Gehaeuse, LEDs nebeneinander: links gruen, Mitte gelb, rechts rot
        n = self.naehe(AMPEL)
        if n is not None and self.ampel != 'aus':
            r = int(4 + 8 * n)
            x, y = int(w * 0.62), int(h * 0.18)
            cv2.rectangle(img, (x - 2 * r, y - 2 * r), (x + 8 * r, y + 2 * r), (20, 20, 20), -1)
            led = {'gruen': (x, (60, 255, 60)), 'gelb': (x + 3 * r, (60, 230, 255)), 'rot': (x + 6 * r, (60, 60, 255))}
            for farbe, (lx, bgr) in led.items():
                if farbe == self.ampel:
                    cv2.circle(img, (lx, y), r + 2, bgr, -1)
                    cv2.circle(img, (lx, y), max(1, r - 2), (250, 250, 250), -1)
                else:
                    cv2.circle(img, (lx, y), r, (40, 40, 40), -1)
        # Stoppschild
        n = self.naehe(SCHILD)
        if n is not None:
            r = int(15 + 60 * n)
            cx, cy = int(w * 0.8), int(h * 0.25)
            pts = np.array([[cx + r * math.cos(math.pi / 8 + k * math.pi / 4),
                             cy + r * math.sin(math.pi / 8 + k * math.pi / 4)] for k in range(8)], np.int32)
            cv2.fillPoly(img, [pts], (30, 30, 200))
            cv2.putText(img, 'STOP', (cx - int(r * 0.7), cy + int(r * 0.2)), cv2.FONT_HERSHEY_SIMPLEX,
                        r / 40, (255, 255, 255), max(1, r // 15))
        # Hindernis (brauner Klotz auf der Fahrbahn)
        d = self.hindernis_abstand()
        if d is not None:
            groesse = int(40 + 200 * max(0.0, 1 - d / 1.5))
            unten_y = int(h * 0.5 + h * 0.5 * max(0.0, 1 - d / 1.5))
            cv2.rectangle(img, (w // 2 - groesse // 2, unten_y - groesse), (w // 2 + groesse // 2, unten_y),
                          (40, 80, 130), -1)
        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color'
        self.pub_bild.publish(msg)
        self.pub_info_c.publish(CameraInfo(header=msg.header, width=w, height=h))

    def tiefe(self):
        h, w = 240, 320
        # Boden: unten nah (0,25 m), oben fern (0,9 m)
        zeilen = np.linspace(0.9, 0.25, h, dtype=np.float32)[:, None]
        d = np.repeat(zeilen, w, axis=1)
        d += np.random.normal(0, 0.005, d.shape).astype(np.float32)
        a = self.hindernis_abstand()
        if a is not None and a < 0.9:
            # Hindernis ragt aus dem Boden: Mitte des Bildes, Abstand = a
            y0 = int(h * 0.35)
            d[y0:, int(w * 0.35):int(w * 0.65)] = np.minimum(d[y0:, int(w * 0.35):int(w * 0.65)], max(0.1, a))
        msg = self.bridge.cv2_to_imgmsg((d * 1000).astype(np.uint16), '16UC1')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_tiefe.publish(msg)
        self.pub_info_d.publish(CameraInfo(header=msg.header, width=w, height=h))

    def lidar(self):
        n = 360
        winkel = -math.pi + np.arange(n) * 2 * math.pi / n
        r = np.full(n, 1.6, np.float32)
        r += 0.3 * np.cos(2 * winkel)     # Strassenraender/Haeuser in der Ferne
        a = self.hindernis_abstand()
        if a is not None:
            vorne = np.abs(winkel) < math.radians(8)   # schmaler Gegenstand vorne
            r[vorne] = np.minimum(r[vorne], a + 0.12)  # +12 cm: LiDAR sitzt hinter der Stossstange
        if HAUS[0] <= self.s <= HAUS[1]:
            ecke = np.abs(winkel - math.radians(25)) < math.radians(4)  # Hausecke schraeg vorne links
            r[ecke] = 0.35
        for i, pub in enumerate(self.pub_scan):
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'laser{i}'
            msg.angle_min, msg.angle_max = -math.pi, math.pi - 2 * math.pi / n
            msg.angle_increment = 2 * math.pi / n
            msg.range_min, msg.range_max = 0.05, 12.0
            msg.ranges = (r if i == 0 else np.full(n, 2.5, np.float32)).tolist()
            pub.publish(msg)

    def imu_odom(self):
        self.pub_imu.publish(Imu())
        o = Odometry()
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.pub_odom.publish(o)


def main():
    rclpy.init()
    node = Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
