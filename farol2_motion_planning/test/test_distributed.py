#!/usr/bin/env python3
# usage: python3 distributed_test.py
import subprocess
import sys
import time

PKG = 'farol2_motion_planning'
NODE_NS = '/farol2_motion_planning/distributed_planner_node'
DISCOVERY_TIME = 5.0    # seconds with discovery_mode on
SERVICE_TIMEOUT = 30.0  # seconds to wait for each service

# name: (e0, n0, theta0, eg, ng, thetag, v)
VEHICLES = {
    'mred0':    (0.0,   0.0, 0.0,  0.0, 200.0, 0.0,  0.05),
    'myellow0': (2.0, 200.0, 3.14, 0.0,   0.0, 3.14, 0.05),
}

TOPIC = '/farol2_motion_planning/distributed_planner_node/start_stop_distributed'
MSG = 'farol2_motion_planning/msg/DistributedCommand'

def ros2(*args):
    r = subprocess.run(['ros2', *args], capture_output=True, text=True)
    out = (r.stdout + r.stderr).strip()
    if r.returncode != 0:
        print(f"FAILED: ros2 {' '.join(args)}\n{out}")
    return r.returncode == 0, out


def wait_for_service(name):
    t0 = time.time()
    while time.time() - t0 < SERVICE_TIMEOUT:
        ok, out = ros2('service', 'list')
        if name in out.splitlines():
            return
        time.sleep(1.0)
    sys.exit(f"Timed out waiting for {name}. Are the nodes running?")


def state(name, e, n, th, v):
    return f"{{vehicle_name: '{name}', e: {e}, n: {n}, theta: {th}, v: {v}}}"

def command(start):
    names = ', '.join(f"'{n}'" for n in VEHICLES)
    ros2('topic', 'pub', '--once', '-w', str(len(VEHICLES)), TOPIC, MSG,
         f"{{vehicle_names: [{names}], start: {str(start).lower()}}}")
    
def main():
    # states, goals, discovery on
    for name, (e0, n0, th0, eg, ng, thg, v) in VEHICLES.items():
        wait_for_service(f'/{name}/set_states')
        wait_for_service(f'/{name}/set_goal')
        ros2('service', 'call', f'/{name}/set_states',
             f'{PKG}/srv/SetState', state(name, e0, n0, th0, v))
        ros2('service', 'call', f'/{name}/set_goal',
             f'{PKG}/srv/SetState', state(name, eg, ng, thg, v))
        ros2('param', 'set', f'{NODE_NS}/{name}', 'discovery_mode', 'true')
        print(f"{name}: configured, discovery on")

    # discovery off
    time.sleep(DISCOVERY_TIME)
    for name in VEHICLES:
        ros2('param', 'set', f'{NODE_NS}/{name}', 'discovery_mode', 'false')
    print("discovery off")

        # start
    command(True)
    print(f"start sent to: {','.join(VEHICLES)}")

if __name__ == '__main__':
    main()