# FAROL 2 runbook

Procedures for the common tasks. The commands in sections 1 to 4 (launch, reference velocity, waypoint, path following) and the stop commands in section 5 were run against a live `start_vehicle_sim` session (ROS 2 Jazzy, branch `dev`, vehicles `delfim` and `magicelectric`). Sections 6 to 9 (bags, gains, configuration, real vehicle) and the `UpdateGains` example are written from the source and launch files and were **not** executed. Read [quickstart.md](quickstart.md) first for units, the `mission_status` codes and the timing gates.

Conventions used below:
- `delfim0` is the vehicle namespace for `vehicle_name:=delfim`. Replace it with your vehicle (`magicelectric0`, ...).
- Run each numbered block in a terminal where the workspace is sourced (`source install/setup.bash`) and `ROS_DOMAIN_ID` / `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` match the terminal running the launch file.
- Commands starting with `ros2 topic pub -r 5` run until you press Ctrl+C. Run them in a second terminal, or append `&` and kill them afterwards.

## 0. Build

```bash
cd <colcon_ws>            # contains src/farol2
colcon build --symlink-install
source install/setup.bash
```

Needs `libgeographiclib-dev`, `libeigen3-dev`, `ros-jazzy-xacro` and the usual ROS 2 Jazzy packages (see the root [README](../README.md)). If you use a conda/RoboStack ROS install where Eigen lives in `$CONDA_PREFIX/include/eigen3`, export `CPLUS_INCLUDE_PATH=$CONDA_PREFIX/include/eigen3` before building, or `farol2_allocation` fails with `Eigen/Dense: No such file`. The `farol` CLI (`farol build`, `farol cd`, ...) is described in [farol2_cli/README.md](../farol2_cli/README.md).

## 1. Launch a simulated vehicle

```bash
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=delfim
```

Expected: the launch log lists `sim_clock`, `auv_sim`, `filter_node`, `robot_state_publisher`, `allocation_node`, `inner_loop_node`, `open_loop`, `waypoint_node`, `path_following_node`, `path_node`, `console_parser_node`, `console_server`, `nav2console_state_node`. Then `allocation_node` prints `Thruster allocation ready with 2 TF thruster frames`.

Check it is alive:

```bash
ros2 node list                                  # /delfim0/navigation_filter, /delfim0/inner_loop, ...
ros2 topic hz /delfim0/nav/filter/state         # about 10 Hz
ros2 topic echo --once /delfim0/nav/filter/state --field utm_position
```

Notes:
- The first `ros2 node list` after launching can be incomplete. Wait a few seconds, or run `ros2 daemon stop` and retry.
- A `WARN ... Static transform '<ns>/<sensor>_link' -> '<ns>/base_link' is not available, using identity` means the vehicle's xacro lacks that sensor frame (all current xacro files define them).
- To launch a second vehicle in the same graph just run the same command again: `vehicle_id` is auto-assigned (`delfim1`).

## 2. Set a reference velocity (surge and yaw rate)

Use `delfim` for this test: its surge and yaw loops are closed and its allocation does not need `mission_status`.

Surge 1 m/s:

```bash
ros2 topic pub -r 5 /delfim0/inner_loop/ref/surge std_msgs/msg/Float32 "{data: 1.0}"
```

Watch the chain (other terminal):

```bash
ros2 topic echo --once /delfim0/allocation/thrust_x            # data: ~21.8 (N) after 12 s
ros2 topic echo --once /delfim0/allocation/rpm_command --field rpm   # [~2543, ~2543]
ros2 topic echo --once /delfim0/sim/body_velocity              # x ~0.5 .. 1.1 m/s, still accelerating
ros2 topic echo --once /delfim0/nav/filter/state --field utm_position
```

Yaw rate 5 deg/s (publish instead of, not together with, a `yaw` reference):

```bash
ros2 topic pub -r 5 /delfim0/inner_loop/ref/yaw_rate std_msgs/msg/Float32 "{data: 5.0}"
ros2 topic echo --once /delfim0/allocation/torque_z --field data      # ~1.3
ros2 topic echo --once /delfim0/allocation/rpm_command --field rpm    # differential, e.g. [450.8, -450.8]
ros2 topic echo --once /delfim0/nav/filter/state --field attitude.yaw # degrees, increasing
```

Other channels: `ref/sway`, `ref/heave`, `ref/depth`, `ref/altitude`, `ref/yaw` (deg), `ref/pitch`, `ref/roll`, `ref/pitch_rate`, `ref/roll_rate`. A channel only works if `enabled: true` in `farol2_bringup/config/<vehicle>/default/inner_loop.yaml`. For delfim: `surge`, `yaw`, `yaw_rate`.

To stop: press Ctrl+C on the `topic pub`. After `ref_timeout` (2 s) the inner loop stops publishing that channel and the wrench goes to zero (the topics stop updating).

Sim-specific notes:
- The delfim sim parameters are marked unreliable in `sim.yaml`. The yaw response is slow (about 5 degrees after 7 s at 5 deg/s demand).
- Direct `ros2 param`/service changes of controller gains: see section 7.

### Same on magicelectric (rudder + one thruster)

`magicelectric` output is gated by `mission_status` (without it rpm and rudder stay at zero) and only has the `yaw` channel enabled (plus the open-loop surge): a `ref/yaw_rate` is silently ignored on this vehicle.

```bash
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=magicelectric
ros2 topic pub -r 5 /magicelectric0/inner_loop/ref/surge std_msgs/msg/Float32 "{data: 1.5}"
ros2 topic pub -r 5 /magicelectric0/inner_loop/ref/yaw   std_msgs/msg/Float32 "{data: 90.0}"    # degrees
ros2 topic pub -r 2 /magicelectric0/mission_status std_msgs/msg/Int8 "{data: 4}"
ros2 topic echo --once /magicelectric0/allocation/rpm_command --field rpm                 # [2200.0, 2200.0]
ros2 topic echo --once /magicelectric0/allocation/control_surface_angle --field angle     # rudder, ~21 deg
ros2 topic echo --once /magicelectric0/nav/filter/state --field attitude.yaw              # approaches 90 (95 after ~18 s)
```

Notes:
- `open_loop` converts surge to rpm with `surge.gain` (2200) and saturates it to `surge.rpm_min/rpm_max` (+-2200 in the vehicle's `inner_loop.yaml`; keep them equal to `rpm_conversion` in `allocation.yaml`). Without those parameters there is no saturation (1.5 m/s would give 3300 rpm).
- `sim.yaml` for magicelectric has `fixed_rpm: true` and an initial body velocity of 1.78 m/s, so the simulated propeller ignores `rpm_command` and the vehicle already moves before any command. Set `fixed_rpm: false` in `sim.yaml` to make the simulator follow the commanded rpm.

## 3. Send a waypoint

Three services, all of type `farol2_waypoint/srv/SendWpType1` (`float32 x`, `float32 y`, `float32 yaw`):

| Service | Controller | Uses `yaw` |
|---|---|---|
| `/delfim0/waypoint/send_wp_standard` | converge to (x, y) | no |
| `/delfim0/waypoint/send_wp_loose` | same, with yaw rate limited by `type1.gains.speed_turn` for wide turns | no |
| `/delfim0/waypoint/send_wp_heading` | converge to (x, y) with a final yaw | yes |

`x` is **UTM northing** and `y` is **UTM easting**, in metres. Get the current position and send a point 40 m north-east of it:

```bash
ros2 topic echo --once /delfim0/nav/filter/state --field utm_position
# northing: 4282546.9  easting: 481587.2   (example)
ros2 service call /delfim0/waypoint/send_wp_standard farol2_waypoint/srv/SendWpType1 \
  "{x: 4282586.9, y: 481627.2, yaw: 0.0}"
# response: success=True, message='New waypoint reference: (4282587.000000,481627.250000)'
```

What happens:
- `mission_status` becomes 4 and the node publishes `ref/surge` and `ref/yaw` (standard/loose) or `ref/surge`, `ref/sway`, `ref/yaw_rate` (heading).
- The vehicle turns and moves toward the point. In the test run it passed within about 4 m of the target and then kept circling; the sim model is crude and delfim cannot always settle inside `cdist: 1.5`. Check `ref/surge` and `ref/yaw` (`ros2 topic echo --once ...`) to see what the controller is asking.
- **Precision warning:** `x` and `y` are `float32`. At UTM magnitudes (about 4.28e6) the resolution is 0.25 m, so the position echoed in the response differs slightly from what you sent.

Special values (give them as both `x` and `y`):

| Value | Effect |
|---|---|
| `-1, -1` | hold the current position |
| `-3, -3` | go to a point `delta_t` (default 5 s) ahead along the current heading |
| `-2, -2` | stop: `mission_status` to 0, timer cancelled. The response has `success: False, message: 'Stop signal sent'` (this is expected, not an error). |

```bash
ros2 service call /delfim0/waypoint/send_wp_standard farol2_waypoint/srv/SendWpType1 "{x: -2.0, y: -2.0, yaw: 0.0}"
```

Parameters (`config/<vehicle>/default/waypoint.yaml`): `node_frequency` 10, `cdist` 1.5 (arrival radius, m), `delta_t` 5, `type1.gains.{ku,ks,speed_turn}`, `type2.gains.{k1,k2,k3}`.

## 4. Path following

Sequence: reset the path, spawn a path, choose the mode and speed, choose the algorithm, start. All services are in `/delfim0/` (planning) and `/delfim0/path_following/` (algorithms and start/stop). The path coordinates are UTM `[northing, easting, z]`.

```bash
# 1. Where are we?
ros2 topic echo --once /delfim0/nav/filter/state --field utm_position     # N, E

# 2. Clear any previous path
ros2 service call /delfim0/ResetPath farol2_planning/srv/ResetPath "{reset_path: true}"

# 3. Spawn a 60 m line toward north-east (use N, E from step 1)
ros2 service call /delfim0/SpawnLinePath farol2_planning/srv/SpawnLine \
  "{start_point: [4282557.2, 481735.1, 0.0], end_point: [4282617.2, 481795.1, 0.0], ref_point: [4282557.2, 481735.1, 0.0]}"

# 4. Closest-point mode (the vehicle projects itself onto the path)
ros2 service call /delfim0/SetMode farol2_planning/srv/SetMode "{closest_point_mode: true}"

# 5. Desired speed along the path (m/s)
ros2 service call /delfim0/SetConstVdVehicle farol2_planning/srv/SetConstSpeed "{speed: 0.8, default_speed: 0.8}"

# 6. Algorithm
ros2 service call /delfim0/path_following/SetBreivik farol2_path_following/srv/SetPF "{}"

# 7. Go (mission_status becomes 6)
ros2 service call /delfim0/path_following/Start farol2_path_following/srv/StartPF "{}"
```

Each call returns `success=True`. The node log (`path_node`, `path_following_node`) shows `Adding LINE to the path`, `Path operation mode: Closest Point`, `Load vehicle speed section: 0.8`, `PF controller switched to breivik`, `Path Following is starting.`.

Monitor progress:

```bash
ros2 topic echo --once /delfim0/path_following/gamma --field data   # path parameter, grows from 0 as the vehicle progresses
ros2 topic echo --once /delfim0/path_following/debug                # PFDebug message
ros2 topic echo --once /delfim0/path_following/path_data            # PathData (point, tangent, curvature, vd, ...)
```

Stop (also resets the path): `ros2 service call /delfim0/path_following/Stop farol2_path_following/srv/StopPF "{}"`.

Other path services (all in `/delfim0/`):

| Service | Type | Request |
|---|---|---|
| `SpawnLinePath` | `farol2_planning/srv/SpawnLine` | `start_point[3]`, `end_point[3]`, `ref_point[3]` |
| `SpawnCircle2DPath` | `SpawnCircle2D` | `radius`, `center_x`, `center_y`, `z` |
| `SpawnArc2DPath` | `SpawnArc2D` | `start_point[2]`, `end_point[2]`, `center_point[2]`, `direction` (int8), `z` |
| `SpawnBernoulliPath` | `SpawnBernoulli` | `radius`, `center_x`, `center_y`, `z` |
| `SetConstVdVehicle` / `SetConstVdRabbit` | `SetConstSpeed` | `speed`, `default_speed` (vehicle speed / virtual target speed) |
| `SetMode` | `SetMode` | `closest_point_mode` (true: project on path; false: path progression comes from an external gamma, used for coordinated path following) |
| `ResetPath` | `ResetPath` | `reset_path` |

Path-following services under `/delfim0/path_following/`: `Start`, `Stop`, `UpdateGains` (`float64[] gains`), `ResetVT` (`value`), and one `Set<Algorithm>` per algorithm: `SetLapierre`, `SetAguiar`, `SetBreivik`, `SetFossen`, `SetIlos`, `SetMarcelo`, `SetPramod`, `SetRavi`, `SetRomulo`, `SetSamson`, `SetRelativeHeading` (all `farol2_path_following/srv/SetPF`).

**Important:** selecting an algorithm reads its gains from `controller_gains.<algo>.*` in the vehicle's `path_following.yaml`. If that block is missing, the node throws a parameter exception. Check the YAML contains the algorithm before calling `Set<Algo>`. The default algorithm is `default_algorithm` in the same file.

Change gains while running: `ros2 service call /delfim0/path_following/UpdateGains farol2_path_following/srv/UpdateGainsPF "{gains: [10.0]}"` (the number and order of gains is algorithm specific; see `farol2_path_following/docs/path_following.md`).

## 5. Abort everything

```bash
ros2 service call /delfim0/path_following/Stop farol2_path_following/srv/StopPF "{}"        # if following a path
ros2 service call /delfim0/waypoint/send_wp_standard farol2_waypoint/srv/SendWpType1 "{x: -2.0, y: -2.0, yaw: 0.0}"   # if following a waypoint
ros2 topic pub --once /delfim0/mission_status std_msgs/msg/Int8 "{data: 0}"
```

Then stop publishing manual references (Ctrl+C on any `topic pub`). Within 2 s the inner loop stops commanding.

## 6. Record and replay a bag

```bash
export ROSDATA_DIR=$HOME/rosbags && mkdir -p $ROSDATA_DIR           # required: the launch uses it as working directory
ros2 launch farol2_bringup bag_record.launch.py vehicle_name:=delfim vehicle_id:=0 [compression:=true]
```

This runs `ros2 bag record -a` (all topics) into `$ROSDATA_DIR/delfim0_<YYYY_MM_DD-HH_MM_SS>`. The arguments `config_package_path_share` and `config_package_path_real` are unused. With the CLI: `farol bag info|topics|play|crop <bag>`.

## 7. Change controller gains while running

`inner_loop/change_params` (`farol2_inner_loop/srv/ChangeParams`) changes the **yaw** controller only, whatever the `controller` field says:

```bash
ros2 service call /delfim0/inner_loop/change_params farol2_inner_loop/srv/ChangeParams \
  "{controller: 'yaw', kp: 1.0, ki: 0.0, kd: 0.0, lpf_wc: 0.0, tau_min: -50.0, tau_max: 50.0, w0: 0.0, xi: 0.0, mr: 0.0}"
```

For other gains edit `config/<vehicle>/default/inner_loop.yaml` and relaunch. Toggle yaw from heading to course with the `SetBool` service `/delfim0/course_instead_of_yaw` (note the name: it is not under `/inner_loop/`).

## 8. Add or switch vehicle / configuration

- Pick a vehicle with `vehicle_name:=<name>`; the YAML files come from `farol2_bringup/config/<name>/default/`. Existing vehicles: `bluerov`, `delfim`, `magicelectric`, `mblack`, `mred`, `myellow`.
- Add one by copying a folder, keep the leading `/**:` wrapper in every YAML, and add a `farol2_description/urdf/<name>.xacro` with the base link, sensors and thruster frames named in `allocation.yaml`.
- `config_to_use:=<folder>` only has an effect on `path_following` today; the other packages always read `default/`.
- The simulator model and noise live in `sim.yaml`; the nav estimator list is `filters` in `nav.yaml` (see [farol2_nav](../farol2_nav/docs/README.md)).

## 9. Real vehicle

```bash
ros2 launch farol2_bringup start_vehicle_real.launch.py vehicle_name:=magicelectric
```

Needs `farol2_drivers` in the same workspace. The file launches `farol2_drivers_bringup/launch/<vehicle_name>_drivers.launch.py` (only `magicelectric` and `towedpayload` exist) and the GNC stack with `use_sim_time:=false`. The driver on/off arguments (`ashtech`, `vn310`, `vn100`, `can_thrusters`, `can_instrumentation`, `airmar*`) are hard-coded in the file: copy it to your own bringup package and edit them. In the current file the driver include is not part of the returned launch description (commented out), so launch the drivers yourself.
