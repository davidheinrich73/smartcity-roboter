#!/usr/bin/env python3
# Linienfolger (schwarze Linie) + Ampel (rotes Licht = Stopp)
# Kamera muss laufen:  scripts/kamera.sh
# Testmodus (faehrt NICHT): scripts/test.sh
# Fahren:                   scripts/fahren.sh
#
# Alle Einstellungen koennen beim Start mit  -p name:=wert  oder waehrend des
# Laufens mit  ros2 param set /line_follower name wert  geaendert werden.
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

import ampel  # liegt im selben Ordner


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        # ---- Allgemein ----
        self.declare_parameter('drive', False)        # False = nur anschauen, nicht fahren
        self.declare_parameter('show', True)          # Fenster anzeigen (braucht DISPLAY)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        # ---- Linie ----
        self.declare_parameter('speed', 0.08)         # m/s
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

        self.red_hits = 0
        self.red_stop = False
        self.last_image = time.time()
        self.bridge = CvBridge()
        topic = self.p('image_topic')
        self.sub = self.create_subscription(Image, topic, self.on_image, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Sicherheit: kommen keine Kamerabilder mehr, Stopp senden
        self.create_timer(0.5, self.watchdog)
        self.get_logger().info(f'Linienfolger gestartet. Warte auf Bilder von {topic} ...')

    def p(self, name):
        return self.get_parameter(name).value

    def watchdog(self):
        if self.p('drive') and time.time() - self.last_image > 1.0:
            self.pub.publish(Twist())
            self.get_logger().warn('Keine Kamerabilder -> Stopp', throttle_duration_sec=2.0)

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

    def find_red(self, img):
        cfg = {name: self.p(name) for name in ampel.STANDARD}
        return ampel.finde_rot(img, cfg)

    def on_image(self, msg):
        self.last_image = time.time()
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]

        y0, line_mask, biggest, cx, cy = self.find_line(img, h, w)

        red, red_box = None, None
        if self.p('red_check'):
            red = self.find_red(img)
            red_box = red['treffer']

        # Entprellen: erst nach mehreren roten Bildern stoppen, erst ohne Rot wieder fahren
        if red_box is not None:
            self.red_hits = min(self.red_hits + 1, 10)
        else:
            self.red_hits = max(self.red_hits - 1, 0)
        if self.red_hits >= self.p('red_frames'):
            self.red_stop = True
        elif self.red_hits == 0:
            self.red_stop = False

        twist = Twist()
        if self.red_stop:
            status = 'AMPEL ROT -> STOPP'
        elif cx is not None:
            error = (w / 2) - cx  # positiv = Linie liegt links von der Bildmitte
            turn = error * self.p('steer_gain')
            limit = self.p('max_turn')
            twist.linear.x = self.p('speed') * self.p('direction')
            twist.angular.z = max(-limit, min(limit, turn))
            status = f'Linie x={cx}  Abweichung={error:+.0f}  Lenkung={twist.angular.z:+.2f}'
        else:
            status = 'KEINE LINIE -> Stopp'

        if self.p('drive'):
            self.pub.publish(twist)
        self.get_logger().info(status, throttle_duration_sec=0.5)

        if self.p('show'):
            view = img.copy()
            cv2.line(view, (0, y0), (w, y0), (255, 0, 0), 2)               # blau: Beginn Suchbereich Linie
            cv2.line(view, (w // 2, y0), (w // 2, h), (0, 255, 255), 1)    # gelb: Bildmitte
            if cx is not None:
                cv2.drawContours(view[y0:h, :], [biggest], -1, (0, 255, 0), 2)  # gruen: erkannte Linie
                cv2.circle(view, (cx, y0 + cy), 8, (0, 0, 255), -1)              # rot: Zielpunkt
            if red is not None:
                ampel.zeichne(view, red)  # lila Suchbereich, orange verworfen, rot = Ampel
            mode = 'FAEHRT' if self.p('drive') else 'TESTMODUS (faehrt nicht)'
            cv2.putText(view, mode, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(view, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
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
