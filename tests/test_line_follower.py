#!/usr/bin/env python3
# Testet LiDAR-Notbremse und Stoppschild-Ablauf des Linienfolgers ohne ROS.
# Aufruf: python3 tests/test_line_follower.py
import os, sys, math, time, types
HIER = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HIER, 'attrappen')); sys.path.insert(0, os.path.join(HIER, '..', 'line_follower'))
import rclpy.node as rn
# Parameter-Unterstuetzung fuer die Attrappe
def declare(self, n, v): self._p = getattr(self, '_p', {}); self._p[n] = v
def get(self, n): return types.SimpleNamespace(value=self._p[n])
rn.Node.declare_parameter = declare; rn.Node.get_parameter = get
import line_follower as lf
n = lf.LineFollower()
def scan(front_r, other=2.0, idx_front=180):
    r = [other]*360; 
    for k in range(idx_front-5, idx_front+6): r[k] = front_r
    return types.SimpleNamespace(ranges=r, angle_min=-math.pi, angle_increment=2*math.pi/360, range_min=0.05, range_max=12.0)
ok = True
def check(name, got, want):
    global ok; good = (got is not None) == want; ok &= good
    print('OK  ' if good else 'FEHLER', name, '->', got)
check('kein LiDAR -> Stopp', n.obstacle(), True)
n.on_scan(scan(1.0), '/scan0', 0); n.on_scan(scan(1.0), '/scan1', 1)
check('frei (1 m vorne)', n.obstacle(), False)
n.on_scan(scan(0.2), '/scan0', 0)
check('Hindernis 20 cm vorne', n.obstacle(), True)
n.on_scan(scan(0.05), '/scan0', 0)
check('5 cm = eigener Roboter, ignorieren', n.obstacle(), False)
n.on_scan(scan(0.2, idx_front=0), '/scan0', 0)
check('20 cm HINTEN -> kein Stopp', n.obstacle(), False)
n._p['direction'] = -1.0; n.on_scan(scan(0.2, idx_front=0), '/scan0', 0)
check('rueckwaerts: 20 cm hinten -> Stopp', n.obstacle(), True)
n._p['direction'] = 1.0; n._p['scan_front_deg'] = [90.0, 0.0]
n.on_scan(scan(0.2, idx_front=270), '/scan0', 0)
check('scan_front_deg=90: Punkt bei +90 Grad -> Stopp', n.obstacle(), True)
n._p['obstacle_check'] = False
check('obstacle_check aus', n.obstacle(), False)
# Stoppschild-Ablauf
n._p['sign_wait'] = 0.3; n._p['sign_cooldown'] = 0.3
box = (1, 1, 50, 50)
r = [n.update_sign(box) for _ in range(3)]
check('3 Bilder Schild -> warten', True if r[-1] else None, True)
time.sleep(0.35); check('nach Wartezeit weiterfahren (Schild noch sichtbar)', True if n.update_sign(box) else None, False)
time.sleep(0.35); r = [n.update_sign(box) for _ in range(3)]
check('nach Cooldown neues Schild erkannt', True if r[-1] else None, True)
print('ALLES OK' if ok else 'FEHLER'); sys.exit(0 if ok else 1)
