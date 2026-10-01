#!/usr/bin/env python3
# ROS-Node: erkennt Personen, Autos usw. im Kamerabild (YOLO) und meldet,
# wenn etwas IM WEG ist. FAEHRT NICHT selbst; der Linienfolger haelt dann an.
#
# Start: scripts/objekte.sh      (Kamera muss laufen)
# Modell: models/yolov8n.onnx    (siehe docs/ki.md, wie man es bekommt)
#
# Sendet:
#   /erkennung/hindernis          std_msgs/Bool    True = etwas Wichtiges im Weg
#   /erkennung/objekte            std_msgs/String  JSON-Liste aller Objekte
#   /erkennung/bild/compressed    Bild mit Kaesten (fuers Control-Panel)
import json
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from yolo import Yolo, zeichne

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


class Objekte(Node):
    def __init__(self):
        super().__init__('objekte')
        self.declare_parameter('model', os.path.join(REPO, 'models', 'yolov8n.onnx'))
        self.declare_parameter('imgsz', 320)          # wie beim Export
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('rate', 5.0)           # so oft pro Sekunde erkennen (spart Rechenzeit)
        self.declare_parameter('min_conf', 0.4)       # Mindest-Sicherheit 0..1
        # Diese Dinge fuehren zum Anhalten, wenn sie im Weg sind (Namen aus yolo.COCO)
        self.declare_parameter('stop_names', ['Person', 'Fahrrad', 'Auto', 'Motorrad', 'Bus', 'LKW',
                                              'Hund', 'Katze', 'Teddy'])
        # "Im Weg" = Kasten unten im Bild, mittig und gross genug (= nah)
        self.declare_parameter('weg_links', 0.2)      # Anteil Bildbreite
        self.declare_parameter('weg_rechts', 0.8)
        self.declare_parameter('weg_unten_min', 0.5)  # Unterkante des Kastens tiefer als 50 % Bildhoehe
        self.declare_parameter('weg_min_hoehe', 0.15)  # Kasten mindestens 15 % der Bildhoehe hoch

        modell = self.get_parameter('model').value
        if not os.path.exists(modell):
            raise SystemExit(f'Modell {modell} fehlt. Siehe docs/ki.md')
        self.yolo = Yolo(modell, self.get_parameter('imgsz').value)
        self.bridge = CvBridge()
        self.bild = None
        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self.on_image, 1)
        self.pub_block = self.create_publisher(Bool, '/erkennung/hindernis', 10)
        self.pub_info = self.create_publisher(String, '/erkennung/objekte', 10)
        self.pub_bild = self.create_publisher(CompressedImage, '/erkennung/bild/compressed', 1)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.erkenne)
        self.get_logger().info(f'Objekterkennung laeuft ({self.yolo.backend}), Modell {modell}')

    def p(self, name):
        return self.get_parameter(name).value

    def on_image(self, msg):
        self.bild = msg  # nur das neueste merken, umgewandelt wird im Timer

    def im_weg(self, o, w, h):
        x, y, bw, bh = o['box']
        mitte = (x + bw / 2) / w
        return (o['name'] in self.p('stop_names') and self.p('weg_links') <= mitte <= self.p('weg_rechts')
                and (y + bh) / h >= self.p('weg_unten_min') and bh / h >= self.p('weg_min_hoehe'))

    def erkenne(self):
        if self.bild is None:
            return
        msg, self.bild = self.bild, None
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        start = time.time()
        objekte = self.yolo.erkenne(img, self.p('min_conf'))
        dauer = time.time() - start
        for o in objekte:
            o['im_weg'] = bool(self.im_weg(o, w, h))
        block = any(o['im_weg'] for o in objekte)
        self.pub_block.publish(Bool(data=block))
        self.pub_info.publish(String(data=json.dumps(objekte)))
        if objekte:
            self.get_logger().info(', '.join(f"{o['name']}{'!' if o['im_weg'] else ''}" for o in objekte)
                                   + f'  ({dauer * 1000:.0f} ms)', throttle_duration_sec=1.0)

        view = img.copy()
        x0, x1 = int(w * self.p('weg_links')), int(w * self.p('weg_rechts'))
        cv2.rectangle(view, (x0, int(h * self.p('weg_unten_min'))), (x1, h - 1), (255, 0, 255), 1)  # lila: "Weg"
        zeichne(view, objekte)
        cv2.putText(view, f'{len(objekte)} Objekte, {dauer * 1000:.0f} ms ({self.yolo.backend})', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        ok, jpg = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            self.pub_bild.publish(CompressedImage(header=msg.header, format='jpeg', data=jpg.tobytes()))


def main():
    rclpy.init()
    node = Objekte()
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
