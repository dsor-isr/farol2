#!/usr/bin/env python3
"""Closed-loop checks of the farol2 stack against the Farol-Isaac myellow simulation.

Run with the farol2 workspace sourced, the Isaac job and `ros2 launch farol2_bringup start_farol2.launch.py
vehicle_name:=myellow vehicle_id:=0 use_sim_time:=true` already running:

    isaac_closed_loop.py [--ns myellow0] [--out DIR] TEST [TEST ...]    TEST = vel | wp | pf | all

Everything is logged to DIR/<test>.csv (10 Hz: sim ground truth + references) and summarised on stdout.
Time is the simulation time (/clock)."""
import argparse, csv, math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Vector3
from farol2_interfaces.msg import NavigationState
from farol2_planning.srv import SpawnLine, SetMode, SetConstSpeed, ResetPath
from farol2_path_following.srv import SetPF, StartPF, StopPF
from farol2_waypoint.srv import SendWpType1
from std_srvs.srv import Trigger


class Tester(Node):
    def __init__(self, ns, out):
        super().__init__("isaac_closed_loop_test", namespace=ns, parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.ns, self.out = ns, out
        self.refs = {}
        self.pub = {k: self.create_publisher(Float32, f"inner_loop/ref/{k}", 1) for k in ("surge", "yaw", "yaw_rate", "depth", "heave", "sway")}
        self.ms_pub = self.create_publisher(Int8, "mission_status", 1)
        self.nav = None
        self.pos = self.vel = self.ori = self.rate = None
        self.create_subscription(NavigationState, "nav/filter/state", lambda m: setattr(self, "nav", m), 1)
        self.create_subscription(Vector3, "sim/position", lambda m: setattr(self, "pos", m), 1)
        self.create_subscription(Vector3, "sim/body_velocity", lambda m: setattr(self, "vel", m), 1)
        self.create_subscription(Vector3, "sim/orientation", lambda m: setattr(self, "ori", m), 1)
        self.create_subscription(Vector3, "sim/orientation_rate", lambda m: setattr(self, "rate", m), 1)
        self.create_timer(0.2, self.publish_refs)
        self.rows, self.t0 = [], None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_refs(self):
        for k, v in self.refs.items():
            self.pub[k].publish(Float32(data=float(v)))

    def spin_for(self, seconds, log=None):
        end = self.now() + seconds
        next_log = 0.0
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if log is not None and self.pos is not None and self.nav is not None and self.now() >= next_log:
                next_log = self.now() + 0.1
                self.rows.append([round(self.now(), 2), self.pos.x, self.pos.y, self.pos.z, self.ori.x, self.ori.y, self.ori.z,
                                  self.vel.x, self.vel.y, self.vel.z, self.rate.z, self.nav.utm_position.northing, self.nav.utm_position.easting,
                                  self.nav.depth, self.nav.attitude.yaw, *[self.refs.get(k, float("nan")) for k in ("surge", "yaw", "yaw_rate", "depth")]])

    def wait_ready(self):
        while rclpy.ok() and (self.nav is None or self.pos is None):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.spin_for(1.0)

    def save(self, name):
        os.makedirs(self.out, exist_ok=True)
        with open(os.path.join(self.out, name + ".csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow("t north east down roll pitch yaw u v w r utm_n utm_e nav_depth nav_yaw ref_surge ref_yaw ref_yaw_rate ref_depth".split())
            w.writerows(self.rows)
        self.rows = []

    def call(self, cli_type, name, req, timeout=5.0):
        cli = self.create_client(cli_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {name} unavailable")
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def stop_all(self):
        self.call(SendWpType1, "waypoint/send_wp_standard", SendWpType1.Request(x=-2.0, y=-2.0))
        try:
            self.call(StopPF, "path_following/Stop", StopPF.Request())
        except Exception:
            pass
        self.refs = {}
        self.ms_pub.publish(Int8(data=0))

    def hold_depth(self, depth, seconds=40):
        """Dive to `depth` with the reference publisher on, no horizontal motion."""
        self.refs = {"depth": depth, "surge": 0.0}
        self.spin_for(seconds)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def test_vel(t):
    print("== vel: depth hold, surge steps, yaw step, yaw-rate step")
    d = 2.0
    t.refs = {"depth": d, "surge": 0.0}
    t.spin_for(40)
    t.rows = []
    t.refs = {"depth": d, "surge": 0.5}
    t.spin_for(40, log=True)
    u = [r[7] for r in t.rows if r[0] > t.rows[0][0] + 30]
    print(f"surge ref 0.5: u mean over last 10 s = {sum(u)/len(u):.3f} m/s, depth {t.nav.depth:.2f} (ref {d})")
    yaw0 = t.ori.z
    t.refs = {"depth": d, "surge": 0.3, "yaw": wrap(yaw0 + 45.0)}
    t.spin_for(40, log=True)
    print(f"yaw step +45 deg: yaw {yaw0:.1f} -> {t.ori.z:.1f} (ref {wrap(yaw0 + 45):.1f})")
    t.refs = {"depth": d, "surge": 0.0, "yaw_rate": 5.0}
    t.spin_for(15, log=True)
    print(f"yaw_rate ref 5 deg/s: measured {t.rate.z:.2f} deg/s")
    t.refs = {"depth": 3.0, "surge": 0.0, "yaw": wrap(t.ori.z)}
    t.spin_for(40, log=True)
    print(f"depth step 2 -> 3 m: depth {t.nav.depth:.2f}")
    t.save("vel")
    t.refs = {}


def test_wp(t):
    print("== wp: send_wp_standard to a point 20 m N and 10 m E")
    t.refs = {"depth": 2.0}
    t.spin_for(10)
    n0, e0 = t.nav.utm_position.northing, t.nav.utm_position.easting
    tgt = (round(n0 + 20.0), round(e0 + 10.0))
    r = t.call(SendWpType1, "waypoint/send_wp_standard", SendWpType1.Request(x=float(tgt[0]), y=float(tgt[1])))
    print("service:", r.success, r.message)
    best = 1e9
    end = t.now() + 150
    while t.now() < end:
        t.spin_for(1.0, log=True)
        dist = math.hypot(t.nav.utm_position.northing - tgt[0], t.nav.utm_position.easting - tgt[1])
        best = min(best, dist)
        if dist < 1.5:
            break
    print(f"waypoint: final distance {dist:.2f} m, closest {best:.2f} m, took {t.now() - (end - 150):.0f} s")
    t.save("wp")
    t.stop_all()
    t.refs = {"depth": 2.0}


def test_pf(t, algo="Breivik"):
    print(f"== pf: line path 25 m north then east, {algo}")
    t.refs = {"depth": 2.0}
    t.spin_for(5)
    n0, e0 = t.nav.utm_position.northing, t.nav.utm_position.easting
    # line starts 4 m east of the vehicle and runs 30 m north: the vehicle must converge to it
    s = [float(round(n0)), float(round(e0 + 4.0)), 0.0]
    e = [s[0] + 30.0, s[1], 0.0]
    t.call(ResetPath, "ResetPath", ResetPath.Request(reset_path=True))
    t.call(SpawnLine, "SpawnLinePath", SpawnLine.Request(start_point=s, end_point=e, ref_point=s))
    t.call(SetMode, "SetMode", SetMode.Request(closest_point_mode=True))
    t.call(SetConstSpeed, "SetConstVdVehicle", SetConstSpeed.Request(speed=0.5, default_speed=0.5))
    t.call(SetPF, f"path_following/Set{algo}", SetPF.Request())
    r = t.call(StartPF, "path_following/Start", StartPF.Request())
    print("start:", r.success)
    end = t.now() + 100
    while t.now() < end:
        t.spin_for(1.0, log=True)
        if t.nav.utm_position.northing > e[0] - 1.0:
            break
    cross = [abs(r_[12] - s[1]) for r_ in t.rows]
    last = [abs(r_[12] - s[1]) for r_ in t.rows[-50:]]
    print(f"cross-track: initial {cross[0]:.2f} m, max {max(cross):.2f} m, final(last 5 s mean) {sum(last)/len(last):.2f} m; north progress {t.nav.utm_position.northing - n0:.1f} m of 30")
    t.save(f"pf_{algo.lower()}")
    t.stop_all()
    t.refs = {"depth": 2.0}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("tests", nargs="+")
    ap.add_argument("--ns", default="myellow0")
    ap.add_argument("--out", default="/scratch/evaristo/farol/runs/myellow_closed_loop")
    ap.add_argument("--algo", default="Breivik")
    a = ap.parse_args()
    rclpy.init()
    t = Tester(a.ns, a.out)
    t.wait_ready()
    # the stack's channels only run with mission_status != 0 on some vehicles; harmless for others
    for name in (["vel", "wp", "pf"] if "all" in a.tests else a.tests):
        {"vel": test_vel, "wp": test_wp, "pf": lambda tt: test_pf(tt, a.algo)}[name](t)
    t.stop_all()
    rclpy.shutdown()
