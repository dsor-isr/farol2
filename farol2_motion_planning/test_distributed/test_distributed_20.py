#!/usr/bin/env python3
# usage: python3 distributed_test_circle.py mred0 myellow0 [mblack0 ...] [--mode opposite|circle] [--seed 1]
import argparse
import math
import random
import subprocess
import sys
import time

PKG = 'farol2_motion_planning'
NODE_NS = '/farol2_motion_planning/distributed_planner_node'
DISCOVERY_TIME = 5.0    # seconds with discovery_mode on
SERVICE_TIMEOUT = 30.0  # seconds to wait for each service

TOPIC = '/farol2_motion_planning/distributed_planner_node/start_stop_distributed'
MSG = 'farol2_motion_planning/msg/DistributedCommand'


def heading(de, dn):
    # heading measured from north (n axis), as in the original script
    return math.atan2(de, dn)


def start_pose(base, radius, angle_jitter, radius_jitter):
    # point on the circle centered at the origin, heading to the center
    a = base + random.uniform(-angle_jitter, angle_jitter)
    r = radius + random.uniform(-radius_jitter, radius_jitter)
    e, n = r * math.cos(a), r * math.sin(a)
    return e, n, heading(-e, -n)


def make_vehicles(names, radius=20.0, angle_jitter=0.15, radius_jitter=2.0, v=0.2):
    # returns name: (e0, n0, theta0, eg, ng, thetag, v)
    # start: equally spaced on the circle, heading to the center
    # goal: opposite point on the same circle, heading away from the center
    # angle_jitter [rad] and radius_jitter [m] are applied to start and goal independently
    count = len(names)
    vehicles = {}
    for i, name in enumerate(names):
        base = 2.0 * math.pi * i / count
        e0, n0, th0 = start_pose(base, radius, angle_jitter, radius_jitter)
        eg, ng, _ = start_pose(base + math.pi, radius, angle_jitter, radius_jitter)
        thg = heading(eg, ng)
        vehicles[name] = tuple(round(x, 3) for x in (e0, n0, th0, eg, ng, thg)) + (v,)
    return vehicles


def make_vehicles_goal_circle(names, radius=20.0, angle_jitter=0.15, radius_jitter=2.0, v=0.2,
                              goal_center=(60.0, 0.0), goal_radius=20.0, goal_phase=math.pi):
    # same start as make_vehicles
    # goal: equally spaced on a circle centered at goal_center, heading away from its center
    # goal_phase [rad] rotates the goal angles relative to the start angles
    count = len(names)
    vehicles = {}
    for i, name in enumerate(names):
        base = 2.0 * math.pi * i / count
        e0, n0, th0 = start_pose(base, radius, angle_jitter, radius_jitter)
        de, dn, thg = start_pose(base + goal_phase, goal_radius, angle_jitter, radius_jitter)
        thg = heading(de, dn)
        eg, ng = goal_center[0] + de, goal_center[1] + dn
        vehicles[name] = tuple(round(x, 3) for x in (e0, n0, th0, eg, ng, thg)) + (v,)
    return vehicles


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


def command(vehicles, start):
    names = ', '.join(f"'{n}'" for n in vehicles)
    ros2('topic', 'pub', '--once', '-w', str(len(vehicles)), TOPIC, MSG,
         f"{{vehicle_names: [{names}], start: {str(start).lower()}}}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('names', nargs='+')
    parser.add_argument('--mode', choices=['opposite', 'circle'], default='opposite')
    parser.add_argument('--seed', type=int, default=None)
    args = parser.parse_args()

    random.seed(args.seed)
    if args.mode == 'opposite':
        vehicles = make_vehicles(args.names)
    else:
        vehicles = make_vehicles_goal_circle(args.names)
    for name, (e0, n0, th0, eg, ng, thg, v) in vehicles.items():
        print(f"{name}: start=({e0}, {n0}, {th0}) goal=({eg}, {ng}, {thg}) v={v}")

    # states, goals, discovery on
    for name, (e0, n0, th0, eg, ng, thg, v) in vehicles.items():
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
    for name in vehicles:
        ros2('param', 'set', f'{NODE_NS}/{name}', 'discovery_mode', 'false')
    print("discovery off")

    # start
    command(vehicles, True)
    print(f"start sent to: {','.join(vehicles)}")


if __name__ == '__main__':
    main()