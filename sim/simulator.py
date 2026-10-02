#!/usr/bin/env python3
# Simulator: ersetzt Roboter, Kamera, Tiefenkamera, LiDAR und Arm, damit man ALLES ohne
# Roboter testen kann. Braucht ROS 2. FAEHRT KEINEN ECHTEN ROBOTER, aber er benutzt die
# ROS_DOMAIN_ID der Umgebung -> NIE mit Domain 30 im Roboter-Netz starten! (scripts/simulation.sh)
#
# Welt: sim/welt.py (Rundkurs mit engen Kurven, Ampel, Zebrastreifen mit Fussgaenger,
# Holzwuerfel auf der Fahrbahn, Stoppschild, Einbahnstrasse, Haeuser).
#   Ampel: wird rot, wenn der Roboter kommt; gruen nach 3 s Stand
#   Fussgaenger: geht weg, wenn der Roboter 4 s vor dem Zebrastreifen gewartet hat
#   Wuerfel: kann mit dem Arm gegriffen und abgelegt werden (Posen aus config/arm.yaml)
import json
import math
import os
import re
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HIER)
sys.path.insert(0, os.path.join(HIER, '..', 'lib'))
from welt import Welt  # noqa: E402
import arm as armlib   # noqa: E402

GREIF_WEITE = 0.20      # so weit vor der Robotermitte greift der Arm in der Pose "greifen"


class Simulator(Node):
    def __init__(self):
        super().__init__('simulator')
        self.declare_parameter('einbahn', True)
        self.declare_parameter('wuerfel', True)
        self.declare_parameter('fussgaenger', True)
        self.welt = Welt(self.get_parameter('einbahn').value, self.get_parameter('wuerfel').value,
                         self.get_parameter('fussgaenger').value)
        self.bridge = CvBridge()
        self.rng = np.random.default_rng(7)
        x, y, w = self.welt.pose_bei(0.0)
        self.pose = [x, y, w]                 # wahre Position
        self.odom_pose = [x, y, w]            # was die Raeder meinen (mit Fehlern)
        self.v = self.quer = self.w = 0.0
        self.cmd_zeit = 0.0
        self.stand_seit = None
        self.arm = [90, 120, 10, 20, 90, 30]  # Fahrstellung
        self.haelt = None                     # gegriffener Zylinder
        self.abgelegt = None
        self.posen = armlib.lade_posen()
        self.gefahren = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cmd, 10)
        self.create_subscription(String, '/arm6_joints_sim', self.arm_befehl, 10)
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
        self.create_timer(0.02, self.physik)
        self.create_timer(1 / 15, self.kamera)
        self.create_timer(0.1, self.tiefe)
        self.create_timer(1 / 7, self.lidar)
        self.create_timer(1.0, lambda: self.pub_akku.publish(Float32(data=12.3)))
        self.get_logger().info(f'Simulator laeuft, Strecke {self.welt.laenge:.2f} m')

    # ---------------- Befehle ----------------
    def cmd(self, msg):
        self.v, self.quer, self.w = msg.linear.x, msg.linear.y, msg.angular.z
        self.cmd_zeit = time.time()

    def arm_befehl(self, msg):
        m = re.match(r'\[([^\]]*)\]', msg.data)
        if not m:
            return
        neu = [int(v) for v in m.group(1).split(',')]
        alt = self.arm
        self.arm = neu
        greifer_zu = alt[5] > 100 and neu[5] < 60
        greifer_auf = alt[5] < 60 and neu[5] > 100
        aehnlich = lambda pose: pose and all(abs(a - b) <= 5 for a, b in zip(neu[:5], pose[:5]))
        if greifer_zu and self.haelt is None and aehnlich(self.posen.get('greifen')):
            x, y, w = self.pose
            for z in self.welt.zylinder:
                if z['name'] != 'wuerfel':
                    continue
                dx, dy = z['x'] - x, z['y'] - y
                vor, seite = math.cos(w) * dx + math.sin(w) * dy, -math.sin(w) * dx + math.cos(w) * dy
                if abs(vor - GREIF_WEITE) < 0.04 and abs(seite) < 0.03:
                    self.haelt = z
                    self.welt.zylinder.remove(z)
                    self.get_logger().info('Wuerfel gegriffen')
                else:
                    self.get_logger().info(f'Greifen daneben: vor {vor:.3f} m, seitlich {seite:.3f} m')
        if greifer_auf and self.haelt is not None:
            x, y, w = self.pose
            a = w + math.radians(neu[0] - 90)
            self.haelt['x'] = x + 0.05 * math.cos(w) + 0.2 * math.cos(a)
            self.haelt['y'] = y + 0.05 * math.sin(w) + 0.2 * math.sin(a)
            self.welt.zylinder.append(self.haelt)
            self.abgelegt = self.haelt
            self.haelt = None
            self.get_logger().info('Wuerfel abgelegt')

    # ---------------- Welt ----------------
    def physik(self):
        jetzt = time.time()
        dt = min(0.1, jetzt - self.letzte)
        self.letzte = jetzt
        if jetzt - self.cmd_zeit > 0.3:           # Watchdog wie beim echten Board
            self.v = self.quer = self.w = 0.0
        x, y, w = self.pose
        x += (self.v * math.cos(w) - self.quer * math.sin(w)) * dt
        y += (self.v * math.sin(w) + self.quer * math.cos(w)) * dt
        w += self.w * dt
        self.pose = [x, y, (w + math.pi) % (2 * math.pi) - math.pi]
        self.gefahren += math.hypot(self.v, self.quer) * dt
        # Odometrie mit Fehlern: 3 % zu kurz gemessen, Drehung leicht verfaelscht
        ov, oq, ow = self.v * 0.97, self.quer * 0.97, self.w * 1.02 + 0.004
        ox, oy, oth = self.odom_pose
        self.odom_pose = [ox + (ov * math.cos(oth) - oq * math.sin(oth)) * dt,
                          oy + (ov * math.sin(oth) + oq * math.cos(oth)) * dt, oth + ow * dt]
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = ov, oq, ow
        self.pub_odom.publish(o)
        imu = Imu()
        imu.header.stamp = o.header.stamp
        imu.angular_velocity.z = self.w + 0.003 + float(self.rng.normal(0, 0.01))
        self.pub_imu.publish(imu)

        steht = abs(self.v) < 0.005 and abs(self.w) < 0.01 and abs(self.quer) < 0.005
        self.stand_seit = (self.stand_seit or jetzt) if steht else None
        stand = jetzt - self.stand_seit if self.stand_seit else 0.0
        s, seite = self.welt.s_von(x, y)
        vorwaerts = math.cos(w - self.welt.pose_bei(s)[2]) > 0
        # Ampel: rot, wenn der Roboter (vorwaerts) kommt; gruen nach 3 s Stand
        bis_ampel = (self.welt.ampel_s - s) % self.welt.laenge
        if vorwaerts and bis_ampel < 0.8:
            if self.welt.ampel == 'aus':
                self.welt.ampel = 'rot'
            elif self.welt.ampel == 'rot' and stand > 3.0:
                self.welt.ampel = 'gruen'
        elif bis_ampel > self.welt.laenge - 0.2 or not vorwaerts:
            self.welt.ampel = 'aus'
        # Fussgaenger geht weg, wenn der Roboter 4 s vor dem Zebrastreifen gewartet hat
        bis_zebra = (self.welt.zebra_s - s) % self.welt.laenge
        if stand > 4.0 and bis_zebra < 0.6:
            vorher = len(self.welt.zylinder)
            self.welt.zylinder = [z for z in self.welt.zylinder if z['name'] != 'fussgaenger']
            if len(self.welt.zylinder) < vorher:
                self.get_logger().info('Fussgaenger ist ueber den Zebrastreifen gegangen')
        fussgaenger = any(z['name'] == 'fussgaenger' for z in self.welt.zylinder)
        wuerfel = next((z for z in self.welt.zylinder if z['name'] == 'wuerfel'), None)
        self.pub_zustand.publish(String(data=json.dumps({
            'zeit': jetzt, 'x': round(x, 3), 'y': round(y, 3), 'w': round(self.pose[2], 3),
            'odom': [round(v, 3) for v in self.odom_pose], 's': round(s, 3), 'seite': round(seite, 3),
            'vorwaerts': vorwaerts, 'v': round(self.v, 3), 'quer': round(self.quer, 3), 'dreh': round(self.w, 3),
            'gefahren': round(self.gefahren, 3), 'ampel': self.welt.ampel, 'fussgaenger': fussgaenger,
            'wuerfel': None if wuerfel is None else [round(wuerfel['x'], 3), round(wuerfel['y'], 3)],
            'haelt': self.haelt is not None,
            'abgelegt': None if self.abgelegt is None else [round(self.abgelegt['x'], 3), round(self.abgelegt['y'], 3)],
            'arm': self.arm})))

    # ---------------- Sensoren ----------------
    def kamera(self):
        bild, _ = self.welt.kamera(self.pose, self.arm[0] - 90)
        msg = self.bridge.cv2_to_imgmsg(bild, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color'
        self.pub_bild.publish(msg)
        self.pub_info_c.publish(CameraInfo(header=msg.header, width=bild.shape[1], height=bild.shape[0]))

    def tiefe(self):
        _, d = self.welt.kamera(self.pose, self.arm[0] - 90, schritt=2)
        d = d + self.rng.normal(0, 0.003, d.shape) * (d > 0)
        msg = self.bridge.cv2_to_imgmsg((d * 1000).astype(np.uint16), '16UC1')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_depth'
        self.pub_tiefe.publish(msg)
        self.pub_info_d.publish(CameraInfo(header=msg.header, width=d.shape[1], height=d.shape[0]))

    def lidar(self):
        for i, pub in enumerate(self.pub_scan):
            r, winkel = self.welt.lidar(self.pose, rng=self.rng)
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'laser{i}'
            msg.angle_min, msg.angle_increment = float(winkel[0]), float(winkel[1] - winkel[0])
            msg.angle_max = float(winkel[-1])
            msg.range_min, msg.range_max = 0.05, 8.0
            msg.ranges = r.tolist()
            pub.publish(msg)


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
