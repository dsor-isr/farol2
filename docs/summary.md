# farol2: GNC / command-and-control stack for AUVs/ASVs (ROS 2 Jazzy)

Source: `~/farol2` (DSOR-ISR, github.com/dsor-isr/farol2), **branch `dev`** (HEAD `693f523` "Fix namespaces and update documentation", tracks `origin/dev`). First read on `main` 2026-11-04, updated for `dev` the same day. We work on `dev` from now on; branch new work off it as `feature/*`/`fix/*`/`experiment/*`. Companion: `~/SUMMARY-farol2_drivers.md`. **Verified build/run results, bug list and doc changes: `~/ANALYSIS-farol2.md`** (local branches, nothing pushed: `docs/analysis` has the new docs `docs/quickstart.md` and `docs/runbook.md`; `fix/analysis-findings`, stacked on it and currently checked out, fixes the `nav.yaml` key mismatch so nav parameters now apply, open_loop rpm clamp, sensor frames in the xacro files, launch/remap/package fixes, the Eigen link, a NotchFilter dt bug and an auv_sim stddev=0 abort; the sections below describe the code before those fixes where they differ).
Port of the ROS1 Noetic FAROL stack. License MIT. Active developer: Eduardo Cunha.
Requirements: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.3, `sudo apt install libgeographiclib-dev`.
Submodule: `3rd_party/nmea_msgs` (clone with `--recurse-submodules`).
Git workflow (`WORKFLOW.md`): `main` = field-validated only; `dev` = integration; short-lived `feature/*`, `fix/*`, `experiment/*` branches off `dev`; never commit to `main`.

## 1. Build and workspace

```
mkdir -p ~/colcon_ws_sim/src && cd ~/colcon_ws_sim/src
git clone --recurse-submodules git@github.com:dsor-isr/farol2.git
# README gives a ~/.bashrc block: set_colcon_ws_function <name>, COLCON_HOME, ROSDATA_DIR,
# ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, colcon_cd, alias S='source ~/.bashrc'
set_colcon_ws_function sim; colcon_cd; colcon build --symlink-install
```
Workspace names follow `colcon_ws_<name>`. `farol2_drivers` (and `farol2_interfaces`, `farol2_allocation`, `nmea_msgs`) must be in the same workspace to build real-vehicle drivers.
`dev` adds the `farol` CLI (section 7) as an alternative to the hand-rolled bashrc helpers.

## 2. Packages

| Package | Role |
|---|---|
| `farol2_interfaces` | Custom msgs/srvs: `NavigationState`, `Attitude`, `UTM`, `GCS`, `LocalDatum`, `Depth`, `ControlSurfaceAngle`, `Thruster`, `ThrusterRPM`, `Formation`, `MultiSection`, `Section`, `StateConsole`, `ManageProcess.srv`. `ProcessState` was removed on dev; `ThrusterRPM` moved here from allocation and `PFDebug` moved to path_following. |
| `farol2_nav` | Navigation. One node `filter_node` (ROS node name `navigation_filter`): a chain of plugin filters over a shared state. `sample_and_hold` always runs first, then the optional plugins listed in `filters`: `asv_dynamics_model`, `current_estimator` (renamed from `position_current_ekf`), `yaw_rate_ekf`. Fixed-rate tick, stale-measurement timeouts. Tuning services `TuneCurrentEstimator`, `TuneYawRateEkf` and debug msgs `CurrentEstimatorDebug`, `YawRateEkfDebug`. Has a default `config/nav.yaml` example in the package. |
| `farol2_waypoint` | Outer loop. Waypoint references (`send_wp_standard` / `send_wp_loose` / `send_wp_heading` services, srv `SendWpType1`). |
| `farol2_planning` | Path generation: line, arc, circle, Bernoulli lemniscate. `PathData.msg`, `Spawn*` services, `SetMode`, `ResetPath`, const-speed. |
| `farol2_path_following` | Path-following algorithms: Lapierre, Fossen LOS, Aguiar, Breivik, Samson, ILOS, Marcelo, Romulo, Pramod, Ravi and others. Services `StartPF`/`StopPF`/`SetPF`/`UpdateGainsPF`/`ResetVT`. Node `path_following_node` (name `path_following`). On dev the service code was slimmed down (`PathFollowingServices.cpp` -550 lines) and a package-level `config/path_following.yaml` was added; the per-vehicle `placeholder` config was dropped. |
| `farol2_inner_loop` | Inner loop. PI/PID per channel: surge, sway, heave, depth, altitude, yaw, pitch, roll and the rates. Only channels with `enabled: true` are instantiated. Outputs body wrench as thrust/torque x/y/z. Node `inner_loop_node` (name `inner_loop`; was `pid`), plus `open_loop` node and `reference_generator`. |
| `farol2_allocation` | Wrench to actuators. **Rewritten on dev** as a single `allocation_node` (name `actuator_allocation`) that picks the strategy from `allocation.type` in `allocation.yaml`: `thrust` (`StaticThrusterAllocator`, pseudo-inverse of the thrust matrix) or `thrust_rudder` (`RudderAllocator`, common-mode thrusters plus rudder, for ASVs like magicelectric). `ThrusterGeometry` builds the thrust matrix from TF frames of the thrusters (`thrusters.base_frame`, `thrusters.frames`; thrust is assumed along each thruster frame's x axis, so `thrust_axis` was removed). `ThrusterRpmConverter` does force to RPM (`rpm_conversion.mode` 0 = propeller model, 1 = `coef_fwd`/`coef_bwd` polynomial). Old nodes `static_thruster_allocation`, `thruster_rudder_allocation`, `rpm_conversion`, `wrench_manager` are gone, as is `ThrusterForce.msg`. |
| `farol2_sim` | Basic simulators. `auv_sim` (6-DOF, thruster-RPM input, current model, publishes fake sensors incl. a DVL and GPS velocity over ground, with frame_ids and TF compensation) and `magic_electric_sim` (thruster plus rudder, C++ port of a Simulink model). Also `sim_clock` (publishes `/clock`; moved here from the removed `farol2_clock` package, must run with `use_sim_time:=false`). Plugged in via the same `/measurement/*` topics as real drivers. |
| `farol2_description` | URDF and `tf_static.launch.py` (needs `ros-jazzy-xacro`). |
| `farol2_console` | `console_server.py`, an HTTP server (port 7080) bridging the web console (missions, waypoint service calls) to ROS. Partly ported from ROS1. |
| `farol2_utils` | C++ library: angle helpers, low-pass filter (Butterworth/Bessel; Euler/ZOH/Tustin), notch filter, and new `tf_utils` (TF lookup helpers). Eigen3. |
| `farol2_bringup` | Per-vehicle configs and top-level launches. Python helper `get_next_available_vehicle_id()`. |
| `farol2_cli` | New on dev. The `farol` Bash CLI (build, bag tools, driver profiles, ...). No longer an `ament_python` package: it is plain scripts, see section 7. |

## 3. Running

```
ros2 launch farol2_bringup start_vehicle_sim.launch.py  vehicle_name:=delfim [vehicle_id:=] [config_to_use:=default]
ros2 launch farol2_bringup start_vehicle_real.launch.py vehicle_name:=magicelectric
```
- `start_vehicle_sim`:
  - Launches `sim_clock` (arg `farol2_clock`, default true), `farol2_sim`, then `start_farol2`.
  - The simulator is picked automatically: `magic_electric_sim` when `vehicle_name == magicelectric`, otherwise `auv_sim`.
  - `start_farol2` is called with `use_sim_time:='false'`; time comes from the `/clock` published by `sim_clock`, not from a global sim-time flag.
  - It is a template: edit it, or copy it into your own bringup package.
- `start_vehicle_real`:
  - Includes `farol2_drivers_bringup/launch/<vehicle_name>_drivers.launch.py`, then `start_farol2` with `use_sim_time:=false`.
  - The driver toggles are hardcoded in the file (`ashtech: true`, vn310/vn100/can_*/airmar*: false). Edit them per vehicle.
  - Only `magicelectric` and `towedpayload` driver launch files exist in `farol2_drivers`.
- `start_farol2.launch.py` is a hand-edited orchestrator (it replaced the old `process.yaml` / bringup process manager). It includes the launch files of nav, tf_static, allocation, inner_loop, waypoint, path_following, planning and console. Flags set there: nav `sample_and_hold=false`, `filter_node=true`; inner loop `pid=true`, `open_loop=true`. Allocation takes no flags any more: choose it with `allocation.type` in the vehicle's `allocation.yaml` (`thrust` for AUVs like delfim, `thrust_rudder` for magicelectric). `use_sim_time` is a launch arg of `start_farol2` (default false).
- Args: `vehicle_name` (default `magicelectric`), `vehicle_id` (empty means auto-pick the next free numeric id across all `/<name><N>` namespaces in the ROS graph), `config_to_use` (config folder, default `default`).
- Namespace is `/<vehicle_name><vehicle_id>`, e.g. `/magicelectric0`.
- `bag_record.launch.py` records bags (`ROSDATA_DIR`).
- Per-vehicle YAMLs: `farol2_bringup/config/<vehicle>/<config_to_use>/{addons,allocation,inner_loop,nav,path_following,planning,sim,waypoint}.yaml`.
  - Vehicles: `bluerov`, `delfim`, `magicelectric`, `mblack`, `mred`, `myellow`.
  - Add a vehicle by copying a folder.
  - Nodes read params via a `/**:` wildcard prefix. Do not remove it.
  - `sim.yaml` holds mass, inertia, damping, thruster RPM limits and initial lat/lon. The Delfim values are flagged as unreliable.
  - `allocation.yaml` now carries the thruster TF frame names, rudder limits (magicelectric: +-36 deg) and the RPM conversion parameters; `nav.yaml` for magicelectric enables `asv_dynamics_model`, `current_estimator`, `yaw_rate_ekf` and lists the `measurements` the node subscribes to.

## 4. Data flow (all topics under `/<vehicle_ns>/`)

```
drivers or sim -> /measurement/{imu,gnss,ned_utm,velocity_over_ground,velocity_through_water,
                                 depth,altimeter,control_surface_angle,thruster_rpm,...}
   (nav `measurements` list on dev: gnss, imu, depth, velocity_over_ground, velocity_through_water,
    utm_ned, altimeter, thruster_rpm, control_surface_angle)
   -> farol2_nav filter_node -> /nav/filter/state   (farol2_interfaces/NavigationState)
        (+ /nav/<filter>/state per stage when publish_all_steps; /nav/filter/reset_dead_reckoning)
/nav/filter/state -> waypoint | planning + path_following
   -> /inner_loop/ref/{surge,sway,heave,depth,altitude,yaw,pitch,roll,yaw_rate,pitch_rate,roll_rate} (Float32)
   -> inner_loop PID -> /allocation/{thrust_x,y,z,torque_x,y,z}
   -> allocation (actuator_allocation) -> /allocation/rpm_command (+ /allocation/rudder_command)
   -> drivers (can_thrusters, can_instrumentation rudder) or sim
```
- Mission state: `/mission_status`, and the legacy `Flag` int8 (0 idle, 4 waypoint, 6 path following).
- Path following: `/control/path_following/{path_data,vc,gamma,virtual_state,debug,Start,Stop,UpdateGains,ResetVT,Set<Algo>}`.
- Planning services: `/ResetPath`, `/SetMode`, `/SpawnBernoulliPath`, `/SpawnLinePath`, `/SetConstVdRabbit`, `/SetConstVdVehicle`.
- Waypoint services: `/control/waypoint/send_wp_{standard,loose,heading}`.
- Inner loop: `/inner_loop/change_params` (yaw gains), `/inner_loop/course_control` (SetBool, yaw from course vs heading), `/inner_loop/pid/<ch>/debug`.
- Sim internals: `/sim/{position,body_velocity,orientation,orientation_rate,body_acceleration,angular_acceleration}`.

**NavigationState conventions.**
- Fields (reworked on dev):
  - `global_position` (GCS) and `utm_position`.
  - `depth`, `altimeter`, `altitude_wgs84`, `altitude_local_datum`.
  - Velocities: `velocity_over_ground` (NED velocity expressed in the body frame), `speed_over_ground`, `velocity_through_water`, `speed_through_water`.
  - Current: `current_velocity_ned`, `current_speed`, `current_direction` (deg, 0 = north, clockwise).
  - `attitude` (roll, pitch, yaw in **degrees**), `course_over_ground` (deg), `angular_velocity` (**deg/s**).
  - Actuator states: `thruster_rpm[]`, `control_surface_angle[]` (deg).
- The PID converts angles with `deg2rad` internally.
- Altitude control output is inverted to NED (down-positive) force.

## 5. Safety and gating behaviour worth knowing

- The PID skips a cycle if `dt <= 0` or `dt > 2/f`.
- A PID channel runs only if its reference is newer than `2/f`. Publish references continuously.
- Nav measurements time out (`timeouts.imu/navsat/utm/rpm` in `nav.yaml`) and are dropped, not fatal.
- Adding a nav filter plugin means (see `farol2_nav/docs/README.md`):
  - Implement `BaseFilter`.
  - Add it to the `build_pipeline()` switch.
  - Add a manual remap for `<name>/state` in the nav launch file (no wildcard remaps in ROS 2).
  - Add its params to `nav.yaml`.
- Nav param tree is `plugins.<filter>.*` (e.g. `plugins.current_estimator.*`, `plugins.yaw_rate_ekf`), with `plugins.sample_and_hold.initializer_measurements` (e.g. gnss, imu) choosing which measurements initialise the state.
- `filters: [""]` (an empty string) is the trick to get a string array when no extra filter is wanted; check each vehicle's `nav.yaml` for which are active.

## 6. Caveats

- Several docs are stale ROS1 text. The waypoint docs reference `auv_msgs/NavigationStatus` and `/#vehicle#/...` topics, `farol2_interfaces/docs` mention a bringup node, and `farol2_bringup/docs` mention `config_default/` and `vehicle_launchers/`. Trust the launch files and `bringup/config` YAMLs.
- The nav README references `architecture.png`, and some docs reference images that may not exist.
- The PID docs say `pid/...` (the old package name). The package is `farol2_inner_loop`.
- `start_vehicle_sim` auto-selects `magic_electric_sim` for `magicelectric` and `auv_sim` otherwise.
- `farol2_allocation/docs/README.md` was renamed `old_README.md`; the per-mode pages (`static_thruster_allocation.md`, `thruster_rudder_allocation.md`, `rpm_conversion.md`) describe the pre-rewrite node layout. Trust `allocation_node.cpp` and `allocation.yaml`.
- `farol2_nav` docs still show `architecture.png`; the old `position_current_ekf` name may linger in places (now `current_estimator`).
- The console and the waypoint service names in `console_server.py` still carry ROS1-style code (cgi, urlopen).
- Real-vehicle driver launch args are hardcoded in `start_vehicle_real.launch.py`.
- `farol2_interfaces/msg/Thruster.msg` is flagged TODO (only used by medusa thrusters, removable once those drivers are ported).

## 7. `farol` CLI (`farol2_cli`, dev only)

Pure Bash dispatcher (`bin/farol` -> `libexec/farol-*`), no colcon build or ROS sourcing needed. Install with `farol2_cli/install.sh` (symlinks into `~/.local/bin`, `~/.local/share/farol`, bash-completion; edits `~/.bashrc` unless `--no-modify-shell-rc`). `uninstall.sh` reverts. Optional snippets in `profile.d/*.bash` (`FAROL_DISABLE_PROFILE=1` skips them).
- Build: `farol build [-jN | --cores N]`, `farol build this` (package of current dir), `farol build sim`/`path_following nav` (the `farol2_` prefix is added), raw colcon args after `--`; `farol clean`; `farol source`.
- Navigation: `farol cd <pkg>` (needs the shell integration; `planning` completes to `farol2_planning`), `farol pkg src|share|cd-src <pkg>`, `farol ws status|root`.
- Bags: `farol bag info|topics|play|crop` (`crop <bag> <start> <end> [-a | topic globs]`).
- Other: `farol bridge serial /dev/ttyACM0`, `farol kill ros` (replaces `nuke_ros2`).
- Drivers: `farol drivers profiles|set <all|magicelectric>|status|enable|disable`. Profiles (in `config/drivers.bash`) toggle managed `COLCON_IGNORE` files in `~/farol2_drivers` (recorded in `.farol_drivers_profile`); the `legacy` subtree stays ignored.
- Add a command by creating `libexec/farol-<a>-<b>` and listing it in `bin/farol`, both completions and the README.


## 8. Isaac closed loop (2026-11-05, branch `feature/myellow-isaac`, local; details `~/farol2/docs/isaac_closed_loop.md`)

- `start_farol2.launch.py` alone drives the Farol-Isaac `myellow` vessel (`vehicle_name:=myellow vehicle_id:=0 use_sim_time:=true`; Isaac provides `/clock` and the `measurement/*` topics). Helper test: `scripts/isaac_tests/isaac_closed_loop.py vel|wp|pf`.
- The surge/sway/heave loops close on `nav/filter/state.velocity_through_water`, which nav fills only from `measurement/velocity_through_water`. `auv_sim` publishes one DVL topic chosen by `sensor.dvl.mode`: `bottom_track` -> `velocity_over_ground`, `water_track` -> `velocity_through_water`. myellow's `sim.yaml` is now `water_track` (it was `bottom_track`, so surge saw 0 and wound up). Other vehicles' `sim.yaml` should be checked for the same issue.
- myellow `inner_loop.yaml` retuned (depth channel enabled instead of heave: kp 20, ki 1, kd 70, +-40 N, rate limit 0.3; yaw kp 8, kd 8, +-10 N m; surge ki 5): the shipped yaw/heave gains were limit-cycling in `auv_sim` too. `myellow.xacro` vertical thruster joint names fixed (positions were right).
