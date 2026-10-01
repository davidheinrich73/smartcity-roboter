#!/usr/bin/env python3
# Zeigt das Control-Panel mit ausgedachten Daten, OHNE Roboter und ohne ROS.
# Aufruf: python3 tests/panel_demo.py   dann im Browser: http://localhost:8099
# Knoepfe wie START tun hier nichts Sinnvolles (es gibt keinen Roboter).
import os, sys, math, time, threading, types
HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HIER, 'attrappen')); sys.path.insert(0, os.path.join(HIER, '..', 'panel'))
import numpy as np, cv2, panel
from http.server import ThreadingHTTPServer
class A: kamera_topic='/camera/color/image_raw'; tiefe_topic='/camera/depth/image_raw'
node = panel.PanelNode(A()); pr = panel.Prozesse()
def fake():
    while True:
        img = np.full((480,640,3),180,np.uint8); cv2.line(img,(320,480),(300,300),(0,0,0),30)
        node._set('kamera', types.SimpleNamespace(img=img))
        d = np.tile(np.linspace(200,2500,640,dtype=np.uint16),(480,1))
        node._set('tiefe', types.SimpleNamespace(img=d, encoding='16UC1'))
        for i,t in enumerate(node.scan_topics):
            r=[1.0+0.5*math.sin(k/20)+i*0.3 for k in range(360)]; r[0:10]=[0.2]*10
            node._scan(t, types.SimpleNamespace(ranges=r, angle_min=-math.pi, angle_increment=2*math.pi/360, range_min=0.05, range_max=12))
        node._set('akku', 11.8); node.zaehle('/battery'); node.zaehle('/imu/data_raw')
        for _ in range(10): node.zaehle('/imu/data_raw'); node.zaehle('/odom_raw')
        node._set('agent', True)
        node._set('lf_status', {'status':'Linie x=310  Abweichung=+10  Lenkung=+0.04'})
        node._set('ki_objekte', [{'name':'Person','sicher':0.81,'im_weg':True}])
        time.sleep(0.1)
threading.Thread(target=fake, daemon=True).start()
srv = ThreadingHTTPServer(('127.0.0.1', 8099), panel.mache_handler(node, pr)); srv.daemon_threads=True
print('Panel-Vorschau: http://localhost:8099  (Strg+C = Ende)')
srv.serve_forever()
