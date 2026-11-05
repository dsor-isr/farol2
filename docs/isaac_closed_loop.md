# farol2 myellow in closed loop with Farol-Isaac


## Note: DVL mode and the surge/sway/heave loops

The inner loop (`farol2_inner_loop/src/inner_loop_node.cpp`, previously `pid.cpp`) subscribes to `nav/filter/state` and closes
- **surge** on `velocity_through_water.x`, **sway** on `.y`, **heave** on `.z`,
- depth/altitude rate terms on `velocity_over_ground.z`.

`filter_node` fills `velocity_through_water` only from a `measurement/velocity_through_water` message.

`auv_sim` publishes a DVL on **one** topic, chosen by `sensor.dvl.mode` in `sim.yaml`:
`bottom_track` -> `measurement/velocity_over_ground`, `water_track` -> `measurement/velocity_through_water`.
`myellow/default/sim.yaml` used `bottom_track`, so in `auv_sim` the nav state had `velocity_through_water = 0` (checked on a running stack:
true surge 1.00 m/s, `nav/filter/state.velocity_through_water` 0, `velocity_over_ground` 1.00). The surge PI then saw a zero measurement,
wound up, and the vehicle accelerated far past the reference (ref 0.5 m/s gave 0.9 m/s and rising).

Changes:
- `myellow/default/sim.yaml`: `dvl.mode: "water_track"`, so the simulated vehicle feeds the surge/sway/heave loops.
- The Isaac bridge (`modules/farol-isaac-cli/lib/ros_bridge.py`) publishes both `measurement/velocity_over_ground` (bottom track) and
  `measurement/velocity_through_water` (velocity relative to the water, i.e. minus the world `water:current`), both in the body frame
  with `dvl_link` as frame id.

## Run

```bash
# 1. Isaac (GPU job; ROS domain 0, the default)
cd ~/Farol-Isaac && sbatch tools/hpc/manual/run_manual.sbatch sample_scenarios/closed_loop_myellow.json 2000 --headless
# 2. farol2, once the Isaac log shows "Linked vessel 'myellow0' ... 6 thruster(s)"; same host, farol2 workspace sourced
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch farol2_bringup start_farol2.launch.py vehicle_name:=myellow vehicle_id:=0 use_sim_time:=true
# 3. tests (CSVs go to /scratch/$USER/farol/runs/myellow_closed_loop/)
python3 scripts/isaac_tests/isaac_closed_loop.py vel wp pf
```
Isaac provides the namespace `/myellow0`, `/clock`, `measurement/{imu,gnss,depth,velocity_over_ground,velocity_through_water,thruster_rpm}` and `sim/*`,
and subscribes `allocation/rpm_command` (6 entries, `allocation.yaml` frame order). No `auv_sim`, `sim_clock` or bridge node runs on the farol2 side.
The 100 x 100 m world limits the runs (the vehicle stops at +-50 m from the origin).

## What was changed on the Isaac side (branch `feature/myellow-isaac` in `~/Farol-Isaac` and its modules)
- Vessel renamed `medusa_yellow` -> `myellow` (extension `dsor.vessels.myellow`, scenarios `*_myellow.json`, namespace `/myellow0`).
- USD: 6 thrusters at the `myellow.xacro` poses (FRD -> FLU) in `allocation.yaml` order; mass 30 kg, inertia, added mass 20/30/80.088 and damping from `sim.yaml`;
  centre of mass at the hull's centre of buoyancy (auv_sim has zg = 0); `farol:datum` = (lat, lon) of the world origin.
- `buoyancy:scale = 0.86385` (new attribute in `dsor.isaac_water`): the hull mesh displaces 1.19 W, auv_sim's sphere 1.025 W. Checked with
  `tools/hpc/calibrate_trim.sbatch`: initial free-ascent acceleration 0.074 m/s^2 (7.4 N net lift on 110 kg).
- The 4 legacy thruster links of the base asset must keep `visuals` and `collisions` deactivated. When active they collided with the hull and produced a
  constant ~28 N backward force and a yaw spin.
- Bridge: GNSS (UTM zone from the datum, `lib/geo.py`), depth, DVL bottom-track and water-track velocities.

## Controller changes in `myellow/default/inner_loop.yaml`
The shipped gains (yaw kp 22, kd 0; heave +-6 N; depth channel disabled) gave limit cycles in `auv_sim` as well as in Isaac, so they were retuned for the
110 kg heave and 4.6 kg m^2 yaw plants: depth enabled (kp 20, ki 1, kd 70, +-40 N, rate limit 0.3 m/s; heave disabled), yaw kp 8 / kd 8 / +-10 N m, surge ki 5.

## Results (Isaac, flat world, no current)
| Test | Result |
|---|---|
| depth hold 2 m, step to 3 m | 2.00 m, 3.04 m, no oscillation |
| `ref/surge` 0.5 m/s | 0.499 m/s (mean of the last 10 s) |
| `ref/yaw` +45 deg | -172.9 -> -127.9 deg (ref -127.9) |
| `ref/yaw_rate` 5 deg/s | 5.4 deg/s |
| `send_wp_standard`, 20 m N + 10 m E | arrived at 1.35 m (cdist 1.5) in 72 s |
| line path, Breivik, 0.5 m/s, start 4 m off the line | cross-track 3.99 -> 0.21 m, 29.2 of 30 m progress |

Not done: other path-following algorithms, curved paths, current/waves, sensor noise, altimeter. The `farol_gym` MedusaYellow preset was updated to the farol2 values but its test-suite was not re-run.
