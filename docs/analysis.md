# farol2 (branch `dev`): code analysis, build/run record, documentation changes

For later sessions. Companion files: `~/SUMMARY-farol2.md` (package overview), `~/SUMMARY-farol2_drivers.md`, `~/SUMMARY-hpc-docs.md` (cluster rules).
Date of analysis: 2026-11-04. Repo `~/farol2`, base `dev` at `693f523`. Work is **local only, nothing pushed** (user instruction). Branch `docs/analysis` (commit `4cade9c`, docs only) and, stacked on it, branch `fix/analysis-findings` with the fixes of section 8 (one commit each; `git -C ~/farol2 log --oneline -8`). Sections 1 to 7 describe the state **before** the fixes; section 8 says what was fixed and which statements above turned out wrong. The current checked-out branch is `fix/analysis-findings`.

Confidence tags: **[run]** observed in the live simulation, **[src]** read from the source but not executed, **[agent]** reported by a read-only code survey and not re-checked by me.

---

## 1. How to build and run (reproducible, no sudo)

Host `hpc-dsor` (Ubuntu 24.04, no sudo, 56 cores). ROS 2 is **not** installed system-wide. A RoboStack environment left by an earlier session is used:

| Item | Path |
|---|---|
| micromamba | `/scratch/evaristo/farol/manual/mm/bin/micromamba` |
| env root / env | `MAMBA_ROOT_PREFIX=/scratch/evaristo/farol/manual/mamba`, env `jazzy` (ros2, colcon, xacro, Eigen3, GeographicLib, robot_state_publisher) |
| fallback | `/scratch/evaristo/farol/ros-jazzy.sif` (Apptainer; has colcon + Eigen, lacks GeographicLib) |
| workspace (mine) | `/scratch/evaristo/farol2_ws_dev/` with `src/farol2 -> ~/farol2` symlink; `build/ install/ log/` live there |
| older partial builds (other session, do not touch) | `/scratch/evaristo/farol/ws`, `/scratch/evaristo/farol/manual/farol2_ws` (only `farol2_interfaces`) |

Per the HPC rules, Slurm is only needed for the GPU. Building and running ROS here is done directly in the shell (user instruction).

```bash
export MAMBA_ROOT_PREFIX=/scratch/evaristo/farol/manual/mamba
eval "$(/scratch/evaristo/farol/manual/mm/bin/micromamba shell hook -s bash)"
micromamba activate jazzy
cd /scratch/evaristo/farol2_ws_dev
colcon build --symlink-install --parallel-workers 4     # 12 packages, ~3 min; (before the Eigen fix of section 8 this also needed export CPLUS_INCLUDE_PATH=$CONDA_PREFIX/include/eigen3)
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROSDATA_DIR=/scratch/evaristo/farol2_ws_dev/bags
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=delfim   # or magicelectric
```

Helper scripts used in the session (session scratchpad, may be gone): `env.sh`, `launch_delfim.sh`, `wp_test.sh`, `pf_test.sh`. Logs: `/scratch/evaristo/farol2_ws_dev/{build-direct.out,launch-delfim.log,launch-me.log}`.

Pitfalls met while running: `ros2 node list` right after launch is incomplete (run `ros2 daemon stop`, wait a few seconds); `pkill -f "ros2 launch ..."` kills your own shell if the pattern is in the command line, use the PID; stop a launch with `kill -INT <pid of ros2 launch>`. All background ROS processes were stopped at the end of the session. Not run: `colcon test`, any real-vehicle launch, `farol2_drivers`.

---

## 2. Verified behaviour [run]

### 2.1 Launch
`start_vehicle_sim ... delfim` starts 13 processes (sim_clock, auv_sim, filter_node, robot_state_publisher, allocation_node, inner_loop_node, open_loop, waypoint_node, path_following_node, path_node, console_parser_node, console_server, nav2console_state_node). Namespace `/delfim0`. `nav/filter/state` publishes at 10.00 Hz. Node names seen: `/delfim0/{navigation_filter, inner_loop, inner_loop/open_loop, actuator_allocation, waypoint, path_following, planning/paths, sim/auv_sim, robot_state_publisher, addons/*}`.

### 2.2 Reference velocity (delfim, closed loop)
| Test | Result |
|---|---|
| `ros2 topic pub -r 5 /delfim0/inner_loop/ref/surge std_msgs/msg/Float32 "{data: 1.0}"` | after ~12 s: `allocation/thrust_x` = 21.8 N, `allocation/rpm_command.rpm` = [2543.4, 2543.4], sim `body_velocity.x` 0.51 (then 1.12 later, still accelerating). |
| `ref/yaw_rate` = 5.0 (deg/s) | `torque_z` = 1.34, rpm = [450.8, -450.8], yaw increases (5.1 deg at ~7 s), measured rate lags the demand (sim params flagged unreliable). |
| stop publishing | outputs stop updating after the 2 s `ref_timeout` (topics go silent). |
| `nav/filter/state.velocity_over_ground` and `speed_*` | stay **0** although `measurement/velocity_over_ground` is published by the sim (see 4.1 bug N1). |

### 2.3 Waypoint (delfim)
`send_wp_standard` with x = N+40, y = E+40 (UTM): `success=True`, message shows the value rounded by float32 (`4282588.178 -> 4282588.0`). The vehicle turned and approached (within ~4 m), then kept circling (never settled in `cdist = 1.5` in 90 s; delfim sim dynamics are crude). `(-2,-2)` returns `success=False, message='Stop signal sent'` and stops. `mission_status` is published only on change (a late `echo --once` sees nothing).

### 2.4 Path following (delfim, Breivik)
`ResetPath` -> `SpawnLinePath` -> `SetMode(closest_point_mode=true)` -> `SetConstVdVehicle(0.8)` -> `path_following/SetBreivik` -> `path_following/Start`: all `success=True`; log lines `Adding LINE to the path`, `Path operation mode: Closest Point`, `Load vehicle speed section: 0.8`, `PF controller switched to breivik`, `Path Following is starting.`. Over 60 s the vehicle turned around onto the line and `path_following/gamma` rose 0 -> 0.14. `path_following/Stop` returned `success=True` and logged `Path reset successful` (Stop also resets the path). Convergence quality and arrival were not evaluated.

### 2.5 magicelectric
Without `mission_status` the outputs are zero (`rpm [0,0]`, no rudder). With `mission_status=4` published at 2 Hz and `ref/surge = 1.5`: `rpm_command = [3300.0, 3300.0]` (open_loop gain 2200 x 1.5, **above** the 2200 rpm limit in `allocation.yaml`, nothing clamps it). Sim `body_velocity.x` was 1.78 regardless: `sim.yaml` has `initial_state.body_velocity: [1.78,0,0]` and `fixed_rpm: true`. The rudder (`allocation/control_surface_angle`) echo returned nothing in the 4 s window; **not investigated** (open item).

### 2.6 Parameter application
`ros2 param get /delfim0/navigation_filter measurements` -> `['gnss','imu']` (code default) while `bringup/config/delfim/default/nav.yaml` lists `gnss, imu, altimeter, thruster_rpm` -> the YAML is **not** applied (bug N1). Other nodes' YAML keys match their launch names (checked by comparing `name=` in all launch files with the top-level keys in each YAML).

---

## 3. Architecture notes (as implemented on `dev`)

Pipeline and topics are in `docs/quickstart.md`. Key facts for anyone changing code:

- Everything is namespaced `/<vehicle_name><vehicle_id>`. Nodes use relative topic names (`TOPIC_*` constants) and the **launch files** remap them to absolute names; look in `launch/*.py` for the real topic of anything. There is no wildcard remap in ROS 2, hence one explicit remap per topic (e.g. a new nav filter needs a `<name>/state` remap).
- YAML parameter files are wrapped in `/**: <node_name>: ros__parameters:`; the node name must equal the launch `name=`. Many nodes use `declare_parameter<T>(name)` **without defaults** (waypoint, allocation, magic_electric_sim): a missing key throws at start-up. `inner_loop` and `path_following` use `automatically_declare_parameters_from_overrides(true)` and `get_parameter` (undeclared access throws).
- Nav: `filter_node` runs a wall timer at `node_frequency`, builds a `MeasurementSnapshot` from latest messages (one per type; flushed each tick), runs `sample_and_hold` then the `filters` list over a shared `State`, publishes `NavigationState` (+ `<filter>/state` per stage). Plugins: `asv_dynamics_model` (u,v,r model from rpm + rudder), `current_estimator`, `yaw_rate_ekf`.
- Inner loop: channels instantiated only if `enabled: true`; outputs of channels on the same axis add; reference gate `ref_timeout` 2 s; `dt` gate `2/f`.
- Allocation: single `allocation_node`; `allocation.type` = `thrust` (pseudo-inverse via TF-derived thrust matrix) or `thrust_rudder`; waits for TF of thruster frames; `mission_status` gate only in `thrust_rudder`; `open_loop: true` hands rpm to the `open_loop` node.
- Units: degrees and deg/s on every ROS interface (attitude, refs, rudder), radians inside controllers; UTM metres for positions (x = northing, y = easting).
- `mission_status` codes: 0 idle, 4 waypoint, 6 path following (replaces ROS 1 `Flag`).
- Time: stack is launched with `use_sim_time:=false`; simulated time comes from `/clock` published by `sim_clock`; simulator nodes force `use_sim_time: True`.

---

## 4. Inconsistencies and bugs (none fixed)

Priority: **H** breaks behaviour or silently ignores configuration, **M** wrong but contained, **L** hygiene.

### 4.1 Navigation and configuration
| # | Pri | Finding | Evidence |
|---|---|---|---|
| N1 | **H** | `nav.yaml` of every vehicle is keyed `filter_node:` but `farol2_nav.launch.py:77` runs the node as `navigation_filter` (C++ constructor name is `filter_node`, `filter_node.cpp:17`). None of the nav parameters are applied: `measurements` falls back to `[gnss, imu]`, so `velocity_over_ground`, `velocity_through_water`, `depth`, `altimeter`, `thruster_rpm`, `utm_ned` are never subscribed. Consequences: `NavigationState` velocities stay 0; `waypoint` (uses `velocity_over_ground`) and the `asv_dynamics_model` have no input. | [run] 2.6 |
| N2 | M | `farol2_nav.launch.py` uses `IfCondition(LaunchConfiguration('filter_node'))` without declaring that argument; works only because `start_farol2` passes `filter_node:=true`. | [agent] |
| N3 | M | `config_to_use` override blocks are commented out in nav, allocation, inner_loop, waypoint, planning, console launch files; only `path_following` loads `<config_to_use>/path_following.yaml` (and fails if that folder does not exist). `start_farol2` does not pass `config_to_use` to allocation. | [agent] |
| N4 | M | URDFs define no sensor frames (`gnss_link`, `dvl_link`, `depth_link`, `rudder_link`), which the simulators put in message headers. `StaticTransformLookup` falls back to identity with a warning on every vehicle. | [run] `delfim.xacro`, grep of all xacro |
| N5 | M | Nav timer is a wall timer but `dt` comes from `now()` (ROS time). With `/clock` speed-up != 1 the period and `dt` disagree. | [src] `filter_node.cpp:161,166-168` |
| N6 | M | `thruster_rpm` is subscribed by nav but neither simulator publishes `measurement/thruster_rpm`; `altitude` and `current_velocity` remaps exist with no subscriber. | [agent] |
| N7 | L | YAML keys never read: `current_estimator.{init_current_x,init_current_y,override_position_state,rpm_*,rho,prop_*,k_t_bp,m_u,x_u,x_uu}` in `farol2_nav/config/nav.yaml` (the docs mention `init_current_x/y`, code has `init_current_speed/direction`); code defaults differ from the YAML (`measurement_noise_pos` 1.0 vs 10000.0, `process_noise_bias` 10 vs 0.01, `measurement_window_samples` 50 vs 40). `farol2_nav/config/` is not installed. | [agent] |
| N8 | L | `magicelectric/nav.yaml` lists a different measurement set from the other vehicles; `timeouts.*` documented previously but not implemented. | [agent]/[src] |
| N9 | L | `measurement_snapshot.hpp:18` TODO: only one message per type is buffered. | [agent] |

### 4.2 Control chain
| # | Pri | Finding | Evidence |
|---|---|---|---|
| C1 | **H** | `open_loop` publishes `rpm = surge_ref * gain` (2200) with no clamp; 1.5 m/s gives 3300 rpm > `max_rpm` 2200 of `allocation.yaml`. Rate fixed at 10 Hz, timeout 0.2 s, two identical entries hard-coded. | [run] + `open_loop.cpp:79-80` |
| C2 | M | Service name mismatch: code `course_instead_of_yaw` (`inner_loop_node.hpp:67`) vs launch remap `course_control` (launch:126). The service lives at `/<ns>/course_instead_of_yaw`. | [run] service list |
| C3 | M | `use_heading_rate_as_yaw_rate` is in several `inner_loop.yaml` (and declared as a member) but never read. | [agent] |
| C4 | M | `inner_loop/change_params` edits only the yaw controller regardless of `controller`; input validation block commented out (L925-930); a negative `tau_min` cannot be set (`tau_min > 0` check). | [agent] |
| C5 | M | delfim has `yaw` and `yaw_rate` both enabled: both add to `torque_z`. Depth rate uses `velocity_over_ground.z` while surge/sway/heave use `velocity_through_water` (mixed convention). | [agent]/[run] |
| C6 | M | `RudderAllocator::compute` feeds degrees (`attitude.yaw`, `angular_velocity.z`) into trigonometry as radians; `solveDeltaFromTau`, `rudder_x_body_drag_`, `gamma` unused; `1.75` hard-coded; `K_L/K_D0/K_D1` read but not applied. Old doc divides by `||V_s||^2`, code does not. | [agent]/[src] |
| C7 | M | `thrust_rudder` mode and `open_loop` output nothing while `mission_status == 0`; nothing sets it for manual references (users must publish it). Undocumented before. | [run] |
| C8 | L | magicelectric `sim.yaml`: `fixed_rpm: true` (propeller ignores commands) and an initial body velocity of 1.78 m/s; header still `TODO: Description`. | [run] |
| C9 | L | `path_following.yaml`: `fossen` has no gains in any vehicle, `ilos` only in magicelectric; selecting an algorithm without its `controller_gains.<algo>` block hits an undeclared parameter (exception). magicelectric defines only ravi, pramod, breivik, ilos but defaults to `ravi`; code fallback is `breivik`. Package `config/path_following.yaml` is unused. | [agent]/[src] |
| C10 | L | `waypoint`: previous controller leaked (free commented out), `res->success` left false on stop, `x,y` are `float32` (0.25 m resolution at UTM values), special values -1/-2/-3 undocumented before, no null check of `wp_controller_` (safe only because the timer starts cancelled). `delta` of arrival (`cdist`) not reached in the delfim test. | [run]/[src] |
| C11 | L | `planning`: `Polinomial5.h` (typo duplicate), `Polynomial5.cpp` and `Sinusoid2D.cpp` are TODO stubs, `Bernoulli.cpp:56` TODO second derivative, `Aguiar.cpp:69,93` TODOs. | [agent] |
| C12 | L | `allocation`: `frame_prefix` read but not set in the shipped YAML; `rpm_command` topic literal rather than constant; all params without defaults. `test/ref_pub.py` hard-codes `/magicelectric0` and mixes degrees and radians (`ramp` vs `sine`). | [agent] |
| C13 | L | `inner_loop`: `PidDebug` fills `tau` from the shared accumulator, pitch/roll debug lack reference fields; `lpf_wc` "ignored". | [agent] |

### 4.3 Bringup, console, repository
| # | Pri | Finding | Evidence |
|---|---|---|---|
| B1 | M | `start_vehicle_sim.launch.py` passes `use_sim_time:'false'` while its comment and the old docs said "forced true"; the sim launch hard-codes `use_sim_time: True` and `sim_clock` is launched separately. Works, but docs/comments disagreed. | [src] |
| B2 | M | `start_vehicle_real.launch.py`: the `drivers` include is not in the returned list (commented out); driver arguments (`ashtech:true`, others false) are hard-coded for magicelectric. Only `magicelectric_drivers` and `towedpayload_drivers` exist in `farol2_drivers_bringup`. | [src] |
| B3 | M | `myellow/default/inner_loop.yaml` has no `open_loop:` block although the launch starts `open_loop`. `inner_loop.yaml`/`allocation.yaml`/`nav.yaml` structures differ between vehicles. | [agent] |
| B4 | M | `farol2_bringup/package.xml` lists only `ament_cmake_python`; no `exec_depend` on the packages that `start_farol2` includes. Undeclared deps: GeographicLib (`farol2_sim` via `pkg_check_modules`, `farol2_nav` via `find_library`), Eigen, tf2, `python3-yaml` (console). | [agent] |
| B5 | L | `bag_record.launch.py`: unused args `config_package_path_share/real`, `prefix` description is a copy-paste, `cwd` = `$ROSDATA_DIR` becomes None if unset, default `vehicle_name` is `'vehicle'`, timestamp taken at launch-description time. | [src] |
| B6 | L | `farol2_console/scripts/__innit__.py` typo; `console_server.py` ROS 1 leftovers (see section 6). | [agent] |
| B7 | L | `tf_static.launch.py` begins with `#Placeholder##!/usr/bin/env python3` (shebang lost); sim TF parent is `world` without vehicle prefix; `sim_utilis/` misspelling; `oreintation_rate_pub_` typo. | [agent] |
| B8 | L | `3rd_party/nmea_msgs` submodule is empty and unused by farol2 (belongs to `farol2_drivers`). `ManageProcess.srv` unused. `get_next_available_vehicle_id` sleeps 0.25 s, silently falls back to `'0'`. | [src] |
| B9 | env | In the RoboStack env, `farol2_allocation` fails to compile unless `CPLUS_INCLUDE_PATH=$CONDA_PREFIX/include/eigen3` is set (`Eigen/Dense: No such file`), although CMake finds Eigen3 and uses `ament_target_dependencies(... Eigen3)`. Probably specific to conda; unknown whether an apt build is affected. `farol2_waypoint`/`farol2_planning` were only aborted by that failure. | [run] |

---

## 5. Documentation work (all on branch `docs/analysis`, uncommitted)

New files: `docs/quickstart.md`, `docs/runbook.md` (launch, reference velocity incl. magicelectric, waypoint, path following, abort, bags, gains, vehicles, real vehicle; sections 1-5 were executed, 6-9 and the `UpdateGains` example are from source only), `farol2_allocation/docs/README.md`, `farol2_nav/docs/asv_dynamics_model.md`, `farol2_console/README.md`, `farol2_utils/README.md`.
Rewritten/updated: root `README.md`, `farol2_waypoint/docs/{README,waypoint,sendWpType1}.md`, `farol2_planning/docs/{paths,README}.md`, `farol2_path_following/docs/{path_following,README,implementation,StartPF,StopPF,SetPF,UpdateGainsPF,ResetVT}.md`, `farol2_inner_loop/docs/{README,inner_loop_node}.md` (`pid.md` renamed with `git mv`), `farol2_interfaces/docs/README.md`, `farol2_bringup/docs/README.md`, `farol2_nav/docs/README.md`, `farol2_allocation/docs/{rpm_conversion,static_thruster_allocation,thruster_rudder_allocation}.md`, `farol2_sim/docs/README.md`, `farol2_description/README.md`.
Still thin/unverified: `farol2_planning/docs/{implementation,PathData,Spawn*,SetMode,...}.md` and `farol2_path_following/docs/implementation.md` (theory pages, only name sweeps applied); `farol2_nav/docs/{current_estimator,yaw_rate_ekf,sample_and_hold}.md` (not checked line by line against code; `yaw_rate_ekf` code declares `m_uv`); `farol2_utils/docs/*`; no per-vehicle YAML reference (key by key) exists; `ManageProcess`, `Formation/MultiSection/Section` semantics are described only by name.

---

## 6. ROS 1 leftovers

### 6.1 Removed or updated in the docs
| File | Old (ROS 1) | New |
|---|---|---|
| `farol2_waypoint/docs/waypoint.md` | topics `/#vehicle#/Flag`, `/#vehicle#/ref/{surge,sway,yaw,yaw_rate}`; `auv_msgs/NavigationStatus`; services `/#vehicle#s/send_wp_*`; parameter paths `/#vehicle#s/outer_loops_controllers/waypoint/...` | `/<ns>/mission_status`, `/<ns>/inner_loop/ref/*`, `farol2_interfaces/NavigationState`, `/<ns>/waypoint/send_wp_*`, YAML keys `waypoint: ros__parameters: ...`; documented `turn_radius_flag`, special values |
| `farol2_waypoint/docs/README.md` | "The *Flag* Topic" | `mission_status` (Flag named as the old name) |
| `farol2_waypoint/docs/sendWpType1.md` | duplicated srv text, no units | fields with units, special values |
| `farol2_planning/docs/paths.md` | `/#vehicle#/...`, `farol_msgs/NavigationState`, `farol_msgs/StateConsole`, `SpawnLineDPath` | `/<ns>/...`, `farol2_interfaces/*`, `SpawnLinePath`; added not-implemented note |
| `farol2_planning/docs/README.md` | `SpawnLineDPath` | `SpawnLinePath` |
| `farol2_path_following/docs/path_following.md` | `/#vehicle#/...`, `//path_following/vc`, `farol_msgs/NavigationState`, `farol_msgs/mPFDebug`, `PFStart/PFStop/PFUpdateGains/PFSet*`, `/gamma`, `/ref/<controller>`, placeholder links `(...)` | real topics/services, `Start/Stop/UpdateGains/Set<Algo>`, `farol2_path_following/PFDebug`, YAML structure `controller_gains.<algo>`, per-vehicle algorithm availability, Breivik and ILOS tables |
| `farol2_path_following/docs/{implementation,README,StartPF,...}.md` | `#vehicle#`, `farol_msgs`, `PFStart/PFStop/PFUpdateGains` (sed sweep) | `<ns>`, `farol2_interfaces`, `Start/Stop/UpdateGains` |
| `farol2_inner_loop/docs/README.md` | "PID Package", package `pid`, `pid/config/pid.yaml`, `colcon build --packages-select pid`, `ros2 launch pid pid.launch.py`, `ros2 run pid pid_control` | `farol2_inner_loop`, real executables, real parameters, `open_loop` node |
| `farol2_inner_loop/docs/pid.md` -> `inner_loop_node.md` | `pid/msg/PidDebug`, `pid/srv/ChangeParams`, `topics.*` parameters, service `course_control` | `farol2_inner_loop/*`, real topics, real service name `course_instead_of_yaw` and the mismatch |
| `farol2_interfaces/docs/README.md` | title "Farol Msgs Package", `ProcessState` msg/srv, `Measurement`, "farol2_bringup node" | actual message list, `Depth`, `ControlSurfaceAngle`, `ThrusterRPM`, field/unit table; `ManageProcess` unused |
| `farol2_bringup/docs/README.md` | `config_default/`, `vehicle_launchers/`, "Always forces `use_sim_time:=true`" | `config/<vehicle>/default/`, `launch/`, the real sim/clock behaviour, node-name-to-YAML-key table |
| `farol2_nav/docs/README.md` | per-sensor `timeouts.imu/navsat/utm/rpm`, "stale measurements are invalidated by timeout", dt "from wall time", filter list without `asv_dynamics_model` | removed/corrected, added launch/config section |
| `farol2_description/README.md` | `meshes/`, `config/` folders | removed (do not exist); real contents and frame notes |
| `README.md` (root) | "catkin workspace setting" | "colcon workspace setting"; added Eigen/tf2 deps and doc index |
| `farol2_allocation/docs/*` | empty template pages | real descriptions (the pre-rewrite math page stays as `old_README.md`) |

### 6.2 Still present in code (not changed)
- `farol2_console/scripts/console_server.py`: `import cgi, cgitb, urllib` (L42), `from urllib.request import urlopen, Request` (L48), `cgi.FieldStorage` (L1403), commented `rospy`/`rosgraph` imports and calls (L20, L22, ~L222, L279-304, L358-366, L655, L1486), rostopic-style help text (L534-541, L1344), `parseXMLStateConsoleAttributesFromROS2NamingToROS1` (L838, called at L531; intentional compatibility with the web add-ons), ~53 functions with large dead blocks. `cgi` is removed in Python 3.13; the launch file suppresses its warning (`farol2_console.launch.py:107`).
- `farol2_bringup/launch/start_farol2.launch.py:1,34`: comments about the old `process.yaml`; stray "NEW PROCESS LAUNCHES GO HERE" comments.
- `farol2_inner_loop` debug topics are still named `.../inner_loop/pid/<ch>/debug` (legacy name).
- `farol2_interfaces/srv/ManageProcess.srv`, `msg/Thruster.msg` ("only used with medusa thrusters", TODO) are legacy.
- Grep for `auv_msgs|rospy|roscpp|catkin|rosparam|dynamic_reconfigure|roslaunch|rosrun` finds nothing in the C++ code and launch files of the other packages (surveyed in nav, sim, interfaces, utils, description, inner_loop, allocation, waypoint, planning, path_following).

---

## 7. Open items and suggestions for the next session
1. Fix N1 (rename the `filter_node:` key to `navigation_filter:` in all six `nav.yaml`, or launch the node as `filter_node`), then re-run the delfim test and confirm `velocity_over_ground` becomes non-zero and the `waypoint` behaviour changes. Decide with the maintainer: this changes behaviour of every vehicle.
2. Clamp `open_loop` to the allocation limits (C1) and make its rate/timeouts parameters.
3. Add sensor frames to the xacro files (N4), or have the sim publish headers in `base_link`.
4. Declare missing dependencies in `package.xml` files (B4); check the Eigen include issue on a plain apt Jazzy install (B9).
5. Investigate why `allocation/control_surface_angle` was silent on magicelectric (2.5): check `sim_rudder: false`, `thrust_rudder` gating and the topic type.
6. Verify the untested runbook sections (6-9) and the remaining theory docs; run `colcon test`.
7. Decide whether to commit the docs: `git -C ~/farol2 add -A docs farol2_*/docs farol2_*/README.md README.md && git commit` on `docs/analysis` (commit trailer per the session reminder), then open a PR to `dev` per `WORKFLOW.md`.
8. Update `~/SUMMARY-farol2.md` if bugs get fixed (it describes the pre-fix state, including the fact that nav parameters were assumed to apply).


---

## 8. Fixes applied afterwards (branch `fix/analysis-findings`, local) and corrections

Each fix was verified in the live simulation or by a build/test, as noted. Final check: clean full rebuild with no workarounds (12 packages), delfim and magicelectric launch with 0 errors/warnings, delfim: surge 1 m/s -> rpm 2436, nav `velocity_over_ground.x` 0.43, waypoint +30 m N/E moved the vehicle north, `(-2,-2)` stops.

| Commit | Fix | Verification |
|---|---|---|
| `ccba111` | **N1**: `nav.yaml` key `filter_node` -> `navigation_filter` in the six vehicles and `farol2_nav/config/nav.yaml`; delfim also gets `velocity_over_ground` in `measurements` (the sim publishes it and delfim listed only gnss/imu/altimeter/rpm). | `ros2 param get` now returns the YAML list; nav velocity follows the sim (0.27 -> 0.33 m/s vs sim 0.40). Note: this changes the behaviour of every vehicle (the EKF plugins now actually run where configured, e.g. magicelectric `asv_dynamics_model` + `current_estimator`). |
| `11477de` | **C1**: `open_loop` gets `surge.rpm_min/rpm_max` (magicelectric +-2200), `surge.num_thrusters`, `node_frequency`, `ref_timeout`; defaults keep the old behaviour (no clamp, 2 thrusters, 10 Hz, 0.2 s). | magicelectric `rpm_command = [2200, 2200]` for a 1.5 m/s reference. |
| `4457854` | **N4**: placeholder zero-offset sensor frames (`gnss_link`, `dvl_link`, `depth_link`, `imu_link`, magicelectric also `rudder_link`) in bluerov, delfim, magicelectric, mblack, mred. Only `myellow` had real ones. | Launch logs have no `Static transform ... using identity` warning any more. The real mounting offsets are still unknown. A `TODO:` colon in an XML comment broke the YAML parsing of `robot_description`, so keep colons out of xacro comments. |
| `10363ca` | **N2** `filter_node` launch argument declared; **C2** remap fixed so the service is `/<ns>/inner_loop/course_control`; `start_farol2` passes `inner_loop:=true` (it passed `pid`, an argument that did not exist); `farol2_sim` declares `geographiclib`; `__innit__.py` -> `__init__.py`; misleading `use_sim_time forced` comment. | `course_control` SetBool answers both values. |
| `ec53311` | **B9**: `farol2_allocation`, `farol2_waypoint`, `farol2_planning` link `Eigen3::Eigen`; no `CPLUS_INCLUDE_PATH` workaround needed. Also reverts the bringup `exec_depend`s that the previous commit added. | Clean rebuild OK. **Lesson**: `farol2_allocation` depends on `farol2_bringup` (its launch reads the config from there), so `farol2_bringup` must not declare the GNC packages as dependencies (colcon: "Unable to order packages topologically"). This implicit cycle (launch files reading `farol2_bringup` config without declaring it) is why B4 is only partly solved. |
| `7c45cea` | **New bug found by `colcon test`**: `NotchFilter::step(x, dt)` accepted `dt == 0` (fs = inf, NaN coefficients). Now rejects `dt <= 0` or non-finite. | `test_notch_filter` 5/5, `test_low_pass_filter` 3/3, `test_moving_average_filter` 5/5. |
| `a67bfe2` | **New bug found by the clean rebuild**: `auv_sim` built `std::normal_distribution(mean, 0)` for the ocean-current noise (`environment.current.sigma = 0`). With the conda toolchain's standard-library assertions (no `Release` build type) `auv_sim` aborted at start (exit -6); with Release flags it silently ran with undefined behaviour. Stddev is now clamped to 1e-12. | delfim launches with 0 errors in a default (non-Release) build. |

### 8.1 Corrections to sections 2 to 6
- **magicelectric rudder silence (2.5) was my test error, not a bug**: magicelectric enables only `yaw` (plus open-loop surge); `ref/yaw_rate` is a disabled channel and is ignored. With `ref/yaw = 90` and `mission_status = 4`: `torque_z` 2.7, rudder 21 deg, heading 95 deg after about 18 s. The runbook example was fixed.
- **N4 / B7 were partly wrong**: `myellow.xacro` did define sensor frames, and `tf_static.launch.py` does *not* start with a broken shebang (the survey agent was wrong). Section 6.1 and `farol2_description/README.md` were corrected on the fix branch.
- `start_farol2` passed `pid: 'true'`, which is not an argument of the inner-loop launch (it is `inner_loop`); the docs that mentioned `pid=true` were corrected.
- **UpdateGains** returns `success=False` for Breivik (it takes no gains); `ResetVT` and `change_params` work; `bag_record` works (mcap files in `$ROSDATA_DIR`, metadata only if stopped with one Ctrl+C).
- The orphan problem: if `start_vehicle_sim` aborts after starting `sim_clock` (e.g. launch exception), `sim_clock` keeps running and the next launch gets `/clock` from two publishers (`Detected jump back in time`, `TF_OLD_DATA`) and the next vehicle id (`delfim1`). Kill leftovers before relaunching (`ps -eo pid,comm | grep -E "sim_clock|auv_sim|magic_electric"`).
- **Orphan `console_server`**: `farol2_console/scripts/console_server.py` (a Python process) is not terminated when the launch is stopped with SIGINT; it survived several sim runs and keeps its HTTP port (7080). Check with `ps -eo pid,args | grep console_server` and kill it before relaunching (my `LEFTOVER` check only looked for C++ node names and missed it; it was killed at the end of the drivers session).
- Tooling traps: `pgrep -f`/`pkill -f` with a pattern that occurs in your own command line kills your shell (exit 144); use PID files (`sim.sh` in the session scratchpad did this: `setsid` + pidfile + `kill -INT -<pgid>`). `ros2 topic echo --once` needs >= 8-10 s timeouts after `ros2 daemon stop`.

### 8.2 Test status (`colcon test`, 12 packages)
Functional unit tests: only `farol2_utils` has them (filters), all pass after `7c45cea`. The other 1700+ "failures" are **style linters** (`ament_uncrustify`, `cpplint`, `flake8`/`pep257`, `xmllint`, `copyright`, `lint_cmake`) on the existing code; `cppcheck` is skipped. They were not addressed (reformatting the code base is a separate decision). There are no tests for nav, allocation, inner_loop, waypoint, planning or path_following logic.

### 8.3 Still open
- Wide behaviour questions for the maintainer: N3 (`config_to_use` is only honoured by path_following), N5 (wall timer vs ROS-time `dt`), N6 (nothing publishes `measurement/thruster_rpm` in the sims, so `asv_dynamics_model` uses rpm=0), N7 (nav YAML keys not read), C3-C6 and C9-C13, B2, B3, B5, B7 (sim TF parent `world` unprefixed), B8.
- `delfim` yaw response in the sim is slow and the waypoint controller does not settle within `cdist` (sim parameters flagged unreliable).
- Real vehicles and `farol2_drivers` were never launched. Runbook sections 8 (adding a vehicle) and 9 (real vehicle) are source-derived only.
- Nothing is pushed. To publish: `git -C ~/farol2 push -u origin docs/analysis fix/analysis-findings` and open PRs against `dev` (do this only when the user asks).


## 9. Closed loop with Farol-Isaac (2026-11-05, branch `feature/myellow-isaac`)

| ID | Finding | Status |
|---|---|---|
| I1 | The surge/sway/heave PIDs read `nav_state.velocity_through_water`, filled only from `measurement/velocity_through_water`. `auv_sim` with `dvl.mode: bottom_track` publishes only `velocity_over_ground`, so the nav value stays 0 (checked live: true u 1.00 m/s, nav VTW 0). Surge then winds up (ref 0.5 -> 0.9 m/s and rising). This is the cause of the "still accelerating" observation in 2.2. Same code in `pid.cpp` on `main`. | myellow `sim.yaml` set to `water_track`; other vehicles not checked |
| I2 | myellow shipped gains limit-cycle in `auv_sim` and Isaac: yaw kp 22 / kd 0 (period ~10 s, r +-40 deg/s), depth channel disabled and heave limited to +-6 N although the net lift is 7.4 N. | retuned, see `docs/isaac_closed_loop.md` |
| I3 | `myellow.xacro` vertical joints were named opposite to their child links (positions OK). | fixed |
| I4 | `auv_sim` buoyancy for myellow: sphere of 1 kg/l, B = 1.025 W, so the vehicle is 7.4 N buoyant uncontrolled (rises at ~0.4 m/s). | noted, reproduced in Isaac with `buoyancy:scale` |

Verified in Isaac (flat world, no current): depth hold 2.00 m / step 3.04 m, surge 0.499 for 0.5 m/s, yaw +45 deg step settles on the reference, yaw rate 5.4 for 5 deg/s, `send_wp_standard` 20 m N + 10 m E reached within 1.35 m in 72 s, Breivik line following cross-track 3.99 -> 0.21 m over 30 m. Not run: other path-following algorithms, curved paths, current, waves, noise, altimeter; `colcon test` not re-run after the config changes.
