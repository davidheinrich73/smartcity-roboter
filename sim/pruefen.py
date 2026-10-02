#!/usr/bin/env python3
# Startet Simulator + KI-Zentrale + Linienfolger (faehrt nur im Simulator) und prueft,
# ob sich der simulierte Roboter richtig verhaelt.
# Aufruf: scripts/simulation.sh pruefen     (setzt eine eigene ROS_DOMAIN_ID, NICHT 30)
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from std_msgs.msg import String

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(REPO, 'sim'))
from simulator import AMPEL, SCHILD, HAUS  # noqa: E402

DAUER = float(os.environ.get('SIM_DAUER', 80))
SZENARIO = os.environ.get('SIM_SZENARIO', 'normal')   # normal | einsatz


def starte(befehl, name):
    log = open(f'/tmp/sim_{name}.log', 'w')
    return subprocess.Popen(befehl, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def main():
    if os.environ.get('ROS_DOMAIN_ID', '0') == '30':
        sys.exit('ROS_DOMAIN_ID=30 ist die der Roboter! Simulation nur mit anderer Domain-ID.')
    cfg = ['--params-file', os.path.join(REPO, 'config', 'roboter.yaml')]
    prozesse = [
        starte([sys.executable, os.path.join(REPO, 'sim', 'simulator.py')], 'simulator'),
        starte([sys.executable, os.path.join(REPO, 'ki', 'zentrale.py'), '--ros-args'] + cfg, 'ki'),
        starte([sys.executable, os.path.join(REPO, 'line_follower', 'line_follower.py'), '--ros-args'] + cfg +
               ['-p', 'drive:=true', '-p', 'show:=false'], 'linienfolger'),
    ]
    rclpy.init()
    node = rclpy.create_node('sim_pruefer')
    zustaende, ereignisse, befehle = [], [], []
    node.create_subscription(String, '/sim/zustand', lambda m: zustaende.append(json.loads(m.data)), 50)
    node.create_subscription(String, '/ki/ereignis', lambda m: ereignisse.append(json.loads(m.data)), 50)
    node.create_subscription(String, '/ki/befehl', lambda m: befehle.append(json.loads(m.data)), 50)
    pub = node.create_publisher(String, '/ki/szenario', 10)
    node.create_timer(1.0, lambda: pub.publish(String(data=SZENARIO)))
    print(f'Simulation laeuft {DAUER:.0f} s, Szenario {SZENARIO} ...')
    ende = time.time() + DAUER
    try:
        while time.time() < ende:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for p in prozesse:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in prozesse:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
        node.destroy_node()
        rclpy.shutdown()

    if not zustaende:
        print('FEHLER: keine Daten vom Simulator. Logs: /tmp/sim_*.log')
        return 1
    t0 = zustaende[0]['zeit']
    print('\nEreignisse der KI:')
    for e in ereignisse:
        print(f"  {e['zeit'] - t0:6.1f} s  {e['text']}")

    fehler = 0

    def check(name, ok, info=''):
        nonlocal fehler
        fehler += not ok
        print(f"{'OK  ' if ok else 'FEHLER'} {name} {info}")

    print('\nPruefungen:')
    erste = [z for z in zustaende if z['runde'] == 0]
    gesamt = zustaende[-1]['gesamt']
    check('Roboter faehrt', gesamt > 2.0, f'({gesamt:.1f} m gefahren)')
    schnell = max(z['v'] for z in zustaende)
    erwartet = (0.17, 0.25) if SZENARIO == 'einsatz' else (0.12, 0.17)
    check('Geschwindigkeit (normal 0,15 m/s, Einsatz x1,3)', erwartet[0] <= schnell <= erwartet[1], f'(max {schnell:.2f} m/s)')
    rot_s = [z['s'] for z in erste if z['ampel'] == 'rot']
    am_schild = [z for z in erste if SCHILD[0] - 0.8 <= z['s'] <= SCHILD[1] and z['v'] == 0]
    stand_schild = (am_schild[-1]['zeit'] - am_schild[0]['zeit']) if am_schild else 0
    if SZENARIO == 'einsatz':
        check('EINSATZ: bei Rot durchgefahren', any(z['ampel'] == 'rot' and z['v'] > 0 for z in erste)
              and not any(z['ampel'] == 'rot' and z['v'] == 0 and z['s'] > 0.35 for z in erste))
        check('EINSATZ: am Stoppschild nicht gehalten', stand_schild < 0.5, f'({stand_schild:.1f} s)')
    else:
        check('bei Rot nicht ueber die Ampel gefahren', not rot_s or max(rot_s) < AMPEL[1],
              f'(max s bei Rot {max(rot_s, default=0):.2f} m)')
        check('vor der roten Ampel angehalten', any(z['ampel'] == 'rot' and z['v'] == 0 for z in erste))
        check('bei Gruen weitergefahren', any(z['s'] > AMPEL[1] for z in erste))
        check('am Stoppschild gehalten (ca. 3 s)', 2.0 <= stand_schild <= 6.0, f'({stand_schild:.1f} s)')
    abstaende = [z['hindernis'] for z in erste if z['hindernis'] is not None]
    if abstaende:
        check('nicht ins Hindernis gefahren', min(abstaende) > 0.1, f'(naechster Abstand {min(abstaende) * 100:.0f} cm)')
        check('vor dem Hindernis gehalten', any(z['hindernis'] is not None and z['v'] == 0 for z in erste))
    else:
        check('Hindernis erreicht', False, '(Simulation zu kurz?)')
    haus = [z['v'] for z in erste if HAUS[0] + 0.05 <= z['s'] <= HAUS[1] - 0.05]
    frei = [z['v'] for z in erste if 1.2 <= z['s'] <= 1.6]
    if haus and frei:
        check('am Haus (nur LiDAR) langsamer, aber nicht gestoppt',
              0 < sum(haus) / len(haus) < 0.8 * max(frei), f'(Haus {sum(haus) / len(haus):.2f} m/s, frei {max(frei):.2f} m/s)')
    else:
        check('Haus erreicht', False, '(Simulation zu kurz?)')
    abw = max(abs(z['e']) for z in zustaende)
    check('Linie nicht verloren', abw < 0.9, f'(groesste Abweichung {abw:.2f})')
    print('\nAlles in Ordnung.' if not fehler else f'\n{fehler} Pruefung(en) fehlgeschlagen. Logs: /tmp/sim_*.log')
    return 1 if fehler else 0


if __name__ == '__main__':
    sys.exit(main())
