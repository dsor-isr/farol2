# FAROL 2: gaps found in the code

Reviewed on 2026-10-15 against `dsor-isr/farol2` branch **`dev`** at commit `693f523` ("Fix namespaces and update documentation", 2026-07-30), which is 273 commits ahead of the commit pinned by `dsor-isr/dsor_simulation-ros2` and well ahead of `main`. Paths below are relative to the `farol2` repository root and line numbers refer to that commit. Gaps 38 to 41 also use [`dsor-isr/farol2_drivers`](https://github.com/dsor-isr/farol2_drivers) at `f210008` (2026-07-23) and [`dsor-isr/magicelectric-simulation-estimation`](https://github.com/dsor-isr/magicelectric-simulation-estimation) at `53a4f34` (2026-07-09).

How each item was checked:

- **Built**: the whole workspace was built with ROS 2 Jazzy (RoboStack build on Ubuntu 24.04, `colcon build --symlink-install`).
- **Ran**: `ros2 launch farol2_bringup start_vehicle_sim.launch.py` was run for `mred` and `magicelectric` and the live graph was queried with `ros2 param`, `ros2 topic` and `ros2 service`.
- **Read**: found by reading the code; not exercised at runtime.

Severity: **High** means a vehicle behaves wrongly or a feature is dead in normal use. **Medium** means wrong in some configurations or a trap for the next developer. **Low** means cleanup.

## Summary

| # | Sev. | Area | Gap | Evidence |
|---|---|---|---|---|
| 1 | High | nav | Every vehicle's `nav.yaml` is silently ignored | Ran |
| 2 | High | bringup / time | Simulation and stack run on different clocks | Read |
| 3 | High | inner_loop / allocation | Two nodes publish to the same `rpm_command` topic | Ran |
| 4 | High | nav | GNSS velocity (NED) is treated as a body-frame velocity | Read |
| 5 | High | nav / inner_loop | Surge loop has no feedback on most vehicles | Read |
| 6 | High | simulation repo | `dsor_simulation-ros2` launches packages that no longer exist | Read |
| 7 | Medium | bringup | `config_to_use` overrides are disabled in 8 of 9 launch files | Read |
| 8 | Medium | path_following | Selecting ILOS crashes the node on 5 of 6 vehicles | Read |
| 9 | Medium | path_following | 3-D path following uses the altimeter as the vehicle's z | Read |
| 10 | Medium | path_following / nav | `reset_dead_reckoning` service has no server | Ran |
| 11 | Medium | inner_loop | `course_control` remap misses the real service name | Ran |
| 12 | Medium | inner_loop | `change_params` always edits the yaw controller | Read |
| 13 | Medium | inner_loop | `use_heading_rate_as_yaw_rate` is never read | Read |
| 14 | Medium | nav | `course_over_ground` is never computed | Read |
| 15 | Medium | sim | `auv_sim` aborts when current sigma is 0 under libstdc++ assertions | Ran |
| 16 | Medium | sim | AUV simulator publishes no altimeter, so altitude control cannot be tested | Read |
| 17 | Medium | console | Undeclared Boost dependency breaks the build outside stock Ubuntu | Built |
| 18 | Medium | console | `console_parser` vehicle id is hard-coded to 3 for every vehicle | Read |
| 19 | Medium | sim / multi-vehicle | Each vehicle starts its own `/clock` publisher | Read |
| 20 | Medium | utils | A unit test fails on `dev` (`NotchFilter::step` accepts `dt = 0`) | Built |
| 21 | Low | nav | Launch file uses an undeclared `filter_node` argument | Read |
| 22 | Low | nav | Example `nav.yaml` documents parameters the code does not read | Read |
| 23 | Low | planning | `Sinusoid2D` and `Polynomial5` are stubs; duplicate `Polinomial5.h` | Built |
| 24 | Low | description | Only `myellow` defines sensor frames | Read |
| 25 | Low | interfaces | `ManageProcess` is dead; `Thruster` is used only by the half-ported Seabotix drivers | Read |
| 26 | Low | build | Stale `farol2_allocation` dependency in `farol2_inner_loop` | Read |
| 27 | Low | tests | Only `farol2_utils` has unit tests; no launch or integration tests | Read |
| 28 | Medium | bringup / drivers | Real-vehicle launch never starts the drivers, and drivers exist only for magicelectric and the towed payload | Read |
| 29 | Low | external | The MATLAB/Simulink work lives in a separate repository that the farol2 docs never mention | Read |
| 30 | Medium | waypoint | Waypoint service carries UTM coordinates as `float32` (about 0.5 m resolution) | Read |
| 31 | Low | waypoint | Each waypoint request leaks the previous controller object | Read |
| 32 | Medium | console | `console_server` and `console_parser` read missions from different folders | Read |
| 33 | Low | console | Console HTML pages are not in the repository; their path is hard-coded | Read |
| 34 | Medium | inner_loop | Delta-form PID uses the wrong derivative for quadratic drag feed-forward | Read |
| 35 | High | allocation | Rudder allocation mixes degrees and radians and ignores flow speed | Read |
| 36 | High | waypoint | Standard waypoint always commands 0.3 m/s surge, so the vehicle never stops | Read |
| 37 | Low | description | `mred` horizontal and `myellow` vertical thrusters sit on the wrong side in the URDF (unconfirmed on hardware) | Read |
| 38 | High | allocation / drivers | magicelectric rudder topics on `dev` no longer match `farol2_drivers`, so the real rudder gets no commands | Read |
| 39 | High | drivers | Medusa thruster drivers are ROS 1 leftovers that cannot receive `rpm_command` | Read |
| 40 | Medium | drivers / nav | Driver `frame_id`s do not match the URDF links, so lever-arm compensation is off on real vehicles | Read |
| 41 | Medium | MATLAB | The Simulink ROS 2 model is hard-wired to `/magicelectric0` and partly out of step with `dev` | Read |

## Details

### 1. Every vehicle's `nav.yaml` is silently ignored (High)

`farol2_nav/launch/farol2_nav.launch.py:77` names the node `navigation_filter`, but every `farol2_bringup/config/<vehicle>/default/nav.yaml` (for example `mred/default/nav.yaml:5`) keys its parameters under `filter_node:`. ROS 2 matches parameter files by node name, so none of them apply. The rename came in the latest `dev` commit (`693f523`), which changed the node name and namespaces but not the YAML files.

Runtime check (magicelectric, whose YAML asks for `asv_dynamics_model` and `current_estimator`):

```
$ ros2 param get /magicelectric0/navigation_filter filters
String values are: []
```

So the navigation filter runs with code defaults on every vehicle: `sample_and_hold` only, `gnss` and `imu` only, and no plugin parameters. The same pattern should be checked for every launch file touched by that commit; the others (`actuator_allocation`, `inner_loop`, `path_following`, `waypoint`, `paths`) do match their YAML keys.

Fix: rename the YAML key to `navigation_filter:` in all six vehicles and in `farol2_nav/config/nav.yaml`, or revert the launch `name=`.

### 2. Simulation and stack run on different clocks (High)

`farol2_bringup/launch/start_vehicle_sim.launch.py:49` says the stack is included "with use_sim_time forced", and `farol2_bringup/docs/README.md` says the sim launcher "always forces `use_sim_time:=true`", but line 59 passes `'use_sim_time': 'false'`. Meanwhile `farol2_sim/launch/farol2_sim.launch.py:71` runs the simulators with `use_sim_time: True`, driven by `sim_clock`, which starts at t = 0.

The result is two time bases: measurements are stamped in sim time (seconds since 0), while the GNC nodes run on wall time. Any `speedup` other than 1.0 in `sim.yaml` makes the simulated vehicle and the controllers drift apart, and timestamps from the two halves cannot be compared in bags. In addition, `farol2_nav/src/filter_node.cpp:161` and `farol2_allocation/src/allocation_node.cpp:201,206` use `create_wall_timer`, so even with `use_sim_time:=true` those loops would tick on wall time while computing `dt` from the ROS clock.

Fix: pass `use_sim_time: 'true'` in the sim launcher and replace the wall timers with `create_timer`.

### 3. Two nodes publish to the same `rpm_command` topic (High)

`start_farol2.launch.py:91` always launches `open_loop` (and passes `pid`, an argument the inner-loop launch file does not declare; it declares `inner_loop`). Both `open_loop` and `actuator_allocation` publish to `/<vehicle>/allocation/rpm_command`:

```
$ ros2 topic info /magicelectric0/allocation/rpm_command
Publisher count: 2
```

On `mred`, `inner_loop.yaml` enables `open_loop.surge` with gain 2000 (copied from magicelectric). Whenever a surge reference is present, `open_loop` publishes a 2-element RPM vector (`farol2_inner_loop/src/open_loop.cpp`, "this is just for magicelectric") at 10 Hz, interleaved with the 4-element vector from allocation. With `mission_status` = 0 it publishes zeros. The simulator then sees alternating, differently sized commands.

Fix: launch `open_loop` only for vehicles that need it (make it a per-vehicle flag), and disable `open_loop.surge` in `mred`.

### 4. GNSS velocity (NED) is treated as a body-frame velocity (High)

Both simulators publish `measurement/velocity_over_ground` from GNSS as an **inertial NED** vector with `frame_id = <ns>/gnss_link` (`farol2_sim/src/auv_sim.cpp:417-424`, `magic_electric_sim.cpp:490-495`). `sample_and_hold` treats every `velocity_over_ground` message as a velocity in the sensor frame and only applies the static base-to-sensor rotation (`farol2_nav/src/filters/sample_and_hold.cpp:154`). `NavigationState.velocity_over_ground` is documented as "expressed in the body" (`farol2_interfaces/msg/NavigationState.msg`).

So whenever `velocity_over_ground` is enabled from GNSS, the body velocity in the nav state is actually the NED velocity, which is only correct when heading north. The DVL bottom-track path (body frame) is handled correctly. The message type (`Vector3Stamped`) carries no way to tell the two apart.

Fix: either publish GNSS velocity in a dedicated topic (or a NED frame id) and rotate it with `rotation_bn.transpose()`, or have the sim publish body-frame velocity.

### 5. Surge loop has no feedback on most vehicles (High)

The surge PID uses `nav_state.velocity_through_water.x` (`farol2_inner_loop/src/inner_loop_node.cpp`, SURGE case). That field is only filled if the nav filter subscribes to `velocity_through_water` or runs `asv_dynamics_model`. With gap 1 (defaults `gnss`, `imu` only) it stays at 0 on every vehicle, and even with the YAML applied it stays at 0 on `mred`, `mblack` and `delfim`, whose `nav.yaml` does not list `velocity_through_water`. The surge integrator then winds up to `tau_max` and the vehicle runs at full surge force whenever a surge reference is active.

Fix: once gap 1 is fixed, enable `velocity_through_water` (DVL water track or sim) or a model-based estimate per vehicle, or fall back to `velocity_over_ground.x` when no water-relative velocity is available.

### 6. `dsor_simulation-ros2` launches packages that no longer exist (High)

`dsor_simulation-ros2/simulation_bringup/launch/start_vehicle.launch.py` starts `farol_bringup/farol_bringup_node` with a `process.yaml`, and its `config_personal/` files use the old `actuation`, `filter_handler` and `#vehicle#` parameter layout. The submodule is pinned to farol2 commit `284097a` (2025-07-24), which still had `farol_bringup`, `farol_actuation`, `farol_nav` and `farol_sim`. Current `farol2` (`main` and `dev`) has none of these; it uses `farol2_bringup/start_vehicle_sim.launch.py` and per-vehicle YAML under `farol2_bringup/config/`. Bumping the submodule will break the simulation repo. Its README also clones into `~/colcon_ws_magicelectric/src` after creating `~/colcon_ws_sim/src`.

Fix: rewrite `simulation_bringup` as a thin wrapper over `farol2_bringup/start_vehicle_sim.launch.py` and bump the submodule.

### 7. `config_to_use` overrides are disabled in 8 of 9 launch files (Medium)

Every package launch file accepts `config_to_use`, but the override YAML entry is commented out in allocation, console, inner_loop, nav, planning, sim and waypoint, and allocation never receives the argument from `start_farol2.launch.py`. Only `farol2_path_following` and `sim_clock.launch.py` load `config/<vehicle>/<config_to_use>/...`. Passing `config_to_use:=placeholder` (the only non-default folder, under `magicelectric`) therefore changes path-following gains and nothing else. Commit `68ae64f` notes this was a deliberate stop-gap after removing `personal_bringup`.

### 8. Selecting ILOS crashes the node on 5 of 6 vehicles (Medium)

`createControllerIlos` (`farol2_path_following/src/ros/PathFollowingNode.cpp`) calls `get_parameter("controller_gains.ilos.delta")`, which throws if the parameter is undeclared. Only `magicelectric/default/path_following.yaml` defines `ilos`. The package's own `farol2_path_following/config/path_following.yaml` has the defaults, but no launch file loads it. Calling `/<vehicle>/path_following/SetIlos` on bluerov, delfim, mblack, mred or myellow terminates the node.

### 9. 3-D path following uses the altimeter as the vehicle's z (Medium)

`PathFollowingNode::vehicleStateCallback` sets `eta1 = [northing, easting, altimeter]` (`PathFollowingNode.cpp:395-397`). For an AUV following a 3-D path, the z coordinate should be depth (NED). Algorithms that use `eta1(2)` (Aguiar and Marcelo with `kz`) will regulate the wrong quantity.

### 10. `reset_dead_reckoning` service has no server (Medium)

At the end of a path, path following calls `/<vehicle>/nav/filter/reset_dead_reckoning` (`farol2_path_following/launch/farol2_path_following.launch.py:114`). No node in the stack provides this service; the ROS 1 `filter_handler` did. The request is sent asynchronously and silently lost.

### 11. `course_control` remap misses the real service name (Medium)

The inner-loop launch remaps `course_control` (`farol2_inner_loop/launch/farol2_inner_loop.launch.py:126`) but the node creates `course_instead_of_yaw`. At runtime the service is at `/<vehicle>/course_instead_of_yaw`, not `/<vehicle>/inner_loop/course_control`.

### 12. `change_params` always edits the yaw controller (Medium)

`InnerLoopNode::changeParamsCallback` validates `request->controller` but then always writes to `controller_yaw_` (`inner_loop_node.cpp:940`), and fails if yaw is disabled even when another controller was requested. The `w0/xi/mr` branch hard-codes a third-order pole-placement formula. `lpf_wc` in the request is ignored.

### 13. `use_heading_rate_as_yaw_rate` is never read (Medium)

The parameter is set in `inner_loop.yaml` (for example `mred/default/inner_loop.yaml:10`) and has a member in `inner_loop_node.hpp:283`, but no code reads or uses it. Yaw always uses `angular_velocity.z`.

### 14. `course_over_ground` is never computed (Medium)

`State::course_over_ground` is published in `NavigationState`, but no filter writes it, so it is always 0. Setting `course_instead_of_yaw: true` therefore makes the yaw controller track a constant 0 deg course.

### 15. `auv_sim` aborts when current sigma is 0 under libstdc++ assertions (Medium)

`farol2_sim/src/auv_sim.cpp:595` builds `std::normal_distribution(mean, sigma)` with `sigma` from `environment.current.sigma`, which is `[0, 0, 0]` in every AUV `sim.yaml`. A zero standard deviation violates the distribution's precondition. With assertion-enabled libstdc++ (as in the RoboStack/conda toolchain used here) `auv_sim` dies on start-up:

```
[auv_sim-2] ... Assertion '_M_stddev > _RealType(0)' failed.
[ERROR] [auv_sim-2]: process has died
```

Stock Ubuntu release builds happen to return the mean, which hides the issue. Fix: skip sampling when sigma is 0.

### 16. AUV simulator publishes no altimeter (Medium)

Neither simulator publishes `measurement/altimeter`, so `NavigationState.altimeter` is always 0 in simulation and the `altitude` controller (and gap 9) cannot be exercised.

### 17. Undeclared Boost dependency in `farol2_console` (Medium)

`farol2_console/include/console_parser.h:7` includes `boost/algorithm/string.hpp`, but Boost is in neither `package.xml` nor `CMakeLists.txt`. It builds on Ubuntu only because ROS pulls Boost headers in transitively; `rosdep` and other environments fail:

```
console_parser.h:7:10: fatal error: boost/algorithm/string.hpp: No such file or directory
```

### 18. `console_parser` vehicle id is hard-coded to 3 (Medium)

Every `addons.yaml` sets `console_parser.vehicle_id: 3`, and because the YAML is loaded after the launch-supplied `vehicle_id`, it wins. All vehicles therefore identify as 3 when parsing formation missions.

### 19. Each vehicle starts its own `/clock` (Medium)

`start_vehicle_sim.launch.py` launches `sim_clock` per vehicle, and `sim_clock` publishes the global `/clock`. Two simulated vehicles give two `/clock` publishers that started at different times. This matters once gap 2 is fixed. The launch file has a `farol2_clock` argument (default `true`), so the workaround is `farol2_clock:=false` for every vehicle after the first; the default should probably be to start the clock only when none is running.

### 20. A unit test fails on `dev` (Medium)

`colcon test --packages-select farol2_utils`: 16 tests, 2 failures. `NotchFilterTest.RejectsInvalidParametersAndUsage` expects `step(x, 0.0)` to throw, but `notch_filter.cpp:56` checks `fs > 0` after `fs = 1/dt`, which is `+inf` for `dt = 0`. The check should be `dt > 0`.

### 21. Undeclared `filter_node` launch argument (Low)

`farol2_nav.launch.py:79` uses `LaunchConfiguration('filter_node')` without declaring it, so launching `farol2_nav.launch.py` on its own fails. It works only when included from `start_farol2.launch.py`, which passes it. `sample_and_hold` is also passed but unused.

### 22. Example `nav.yaml` documents parameters the code does not read (Low)

`farol2_nav/config/nav.yaml` lists `current_estimator.override_position_state`, `init_current_x/y`, `rpm_*`, `rho`, `prop_*`, `k_t_bp`, `m_u`, `x_u`, `x_uu`; the filter reads only the noise terms and `init_current_speed/direction`. `farol2_nav/docs/README.md` documents `timeouts.*` and stale-measurement timeouts that do not exist.

### 23. Planning stubs (Low)

`Sinusoid2D.cpp` is entirely `//TODO` (returns zeros) and `Polynomial5` has no derivatives, but both are compiled into the library. `include/algorithms/path_sections/Polinomial5.h` is a misspelled duplicate of `Polynomial5.h`. `Line` accepts a `ref_point` in `SpawnLine.srv` and stores it, but `Line::eq_pd` never adds it, so line coordinates must be absolute.

### 24. Only `myellow` defines sensor frames (Low)

The simulators stamp measurements with `gnss_link`, `dvl_link`, `depth_link` and `rudder_link`. Only `myellow.xacro` defines sensor links, so for the other vehicles `sample_and_hold` warns once per frame and falls back to identity (`farol2_utils/src/tf_utils.cpp`). Lever-arm compensation is off for those vehicles.

### 25. Dead and half-used interfaces (Low)

`farol2_interfaces/srv/ManageProcess.srv` belonged to the removed `farol2_bringup` process manager and is unused. `Thruster.msg` is marked TODO in its own comment and nothing in `farol2` publishes it, but the Seabotix thruster drivers in `farol2_drivers` subscribe to it ([#39](#39-medusa-thruster-drivers-cannot-receive-rpm_command-high)), so it should be removed together with those drivers' port, not before. `farol2_interfaces/docs/README.md` still describes `ProcessState` and `Measurement`, which no longer exist.

### 26. Stale build dependency (Low)

`farol2_inner_loop/CMakeLists.txt:14` still requires `farol2_allocation`, which no longer exports `BodyWrenchRequest`; the only use is commented out.

### 27. Test coverage (Low)

Only `farol2_utils` has unit tests (low-pass, moving average, notch). There are no tests for allocation, PID, nav filters, path sections or path-following laws, and no launch tests. All packages enable `ament_lint_auto`, which will report many style failures if run.

### 28. Real-vehicle bringup is incomplete (Medium)

`start_vehicle_real.launch.py` builds an include of `farol2_drivers_bringup/<vehicle>_drivers.launch.py` but leaves it out of the returned actions, so a real-vehicle launch starts the GNC stack with no drivers. `farol2_drivers_bringup` (in `dsor-isr/farol2_drivers`) only has `magicelectric_drivers.launch.py` and `towedpayload_drivers.launch.py`; there is a `config/myellow/drivers.yaml` but no launch file for it, and nothing for mred, mblack, bluerov or delfim, although `farol2_cli/config/drivers.bash` has a bluerov profile. The DVL and leak drivers are excluded with `COLCON_IGNORE`.

### 29. MATLAB/Simulink work is undocumented in FAROL 2 (Low)

There are no `.m`, `.slx`, `.mdl` or `.mlx` files in `farol2` or `dsor_simulation-ros2`. The MATLAB work is in `dsor-isr/magicelectric-simulation-estimation`: system-identification scripts for magicelectric's surge and yaw-rate models, the trial data as `.mat` files, and Simulink models including `simulation/simulation/magicelectric_ros2_simulation.slx`, which talks to FAROL 2 over ROS 2 ([#41](#41-the-simulink-ros-2-model-is-hard-wired-to-magicelectric0-medium)). The identified parameters in `load_ros2_simulation.m` match `farol2_bringup/config/magicelectric/default/sim.yaml`, so `magic_electric_sim` is a faithful C++ port. Neither `farol2`'s README nor its docs point to that repository, and there is no MATLAB interface for the other vehicles.

### 30. Waypoint coordinates are `float32` (Medium)

`farol2_waypoint/srv/SendWpType1.srv` declares `float32 x, y, yaw`, and the waypoint is an absolute UTM northing/easting. A `float32` has a 24-bit mantissa, so a northing around 4 290 000 m is quantized to 0.5 m and an easting around 490 000 m to about 0.03 m. The console sends waypoints through the same service. Fix: use `float64`.

### 31. Waypoint controller leak (Low)

`Waypoint::createWaypoint` (`farol2_waypoint/src/waypoint.cpp:150-156`) replaces `wp_controller_` with a `new` object and the `free` is commented out, so every waypoint request leaks the previous controller. Use `std::unique_ptr`.

### 32. Console server and parser use different mission folders (Medium)

`console_server.py:1520` declares `addons.console_parser.path_folder` (default `~/paths_from_console`) on the `console_server` node, but `addons.yaml` sets `path_folder: "../paths_from_console"` under the `console_parser` node. The server therefore lists and writes missions in `~/paths_from_console` while the parser reads `../paths_from_console` relative to its working directory.

### 33. Console pages are not shipped (Low)

`console_server.py:1521` defaults `pages_folder` to `/home/farol/farol-sw/farol_common/http_server/pages/` (its own comment says it "should point to the package's own directory"), and no HTML is in `farol2_console`. Requests for `index.html` on port 7080 fail unless that folder exists on the machine. The web console front end lives outside the repositories reachable here.

### 34. Delta-form PID quadratic feed-forward (Medium)

In the delta (velocity-form) PID, every term is differentiated before integration. `ControllerPID::callControllerImpl` (`farol2_inner_loop/src/controller_pid.cpp`) adds `kffv_sq * state_rate_dot * |state_rate_dot|` as the derivative of the quadratic drag term. The derivative of `kffv_sq * v|v|` is `2 * kffv_sq * |v| * v_dot`, so the current term has the wrong shape and units. The feed-forward also acts on the measured rate rather than the reference rate. All shipped configs set `kffv_sq: 0`, so this only bites when someone enables it. In the non-delta form the integrator also only accumulates while `|tau| < tau_max/2`, ignoring `tau_min`.

### 35. Rudder allocation mixes degrees and radians and ignores flow speed (High)

`RudderAllocator::compute` (`farol2_allocation/src/rudder_allocator.cpp`) affects every `thrust_rudder` vehicle (magicelectric):

- `nav_state.attitude.yaw` and `nav_state.angular_velocity.z` are in degrees and deg/s (`NavigationState.msg`), but they go straight into `std::cos`, `std::sin` and `wrapToPi` and are added to a sideslip angle in radians. The inflow direction `gamma`, the lift/drag projection and the rudder drag are therefore wrong whenever the vehicle is not heading 0 deg.
- The deflection is `tau_r / (K_s · l · 1.75)`: a hard-coded 1.75 replaces the `|V_s|²` term of the model documented in `old_README.md` (`δ = τ_r / (K_s l |V_s|²)`), so the same torque request gives the same rudder angle at any speed. `solveDeltaFromTau`, which implements the lift/drag-aware inversion, is never called.
- The computed rudder drag `rudder_x_body_drag` is stored in `allocation_node.cpp:290` but never added to the surge force, unlike the documented `f = T⁺(τ_x + X_d)`.

### 36. Standard waypoint never stops the vehicle (High)

`WpStandard::calculateRef` (`farol2_waypoint/src/wp_standard.cpp:59`) computes `u_ref` and then calls `setSurgeOut(0.3)` with the comment "FIX for magic electric open loop". The computed value is discarded, on `main` and `dev`, for every vehicle. The surge reference is 0.3 m/s even inside `cdist`, so `send_wp_standard` never brings the vehicle to rest. This also affects the "hold" requests: `-1, -1` (hold here) and the `-3, -3` hold that `path_following` sends at the end of every path both use `WpStandard`, so the vehicle keeps moving after a mission. A related nit: `WpLoose::calculateRef` prints `max_ref_rate` to stdout on every tick (`wp_loose.cpp:13`). Fix: restore `setSurgeOut(u_ref)` and handle magicelectric's open loop in its own config (for example a minimum surge parameter).

### 37. Some thrusters are on the wrong side in the URDF (Low, unconfirmed)

Every URDF uses the marine body frame (x forward, y starboard, z down): `portside` links have negative y in `mblack`, `delfim`, `magicelectric`, `bluerov` and most of `myellow`. Two sets break this:

- `mred.xacro`: `thruster_portside_horizontal_link` is at y = +0.140 and `thruster_starboard_horizontal_link` at y = −0.140, while mred's own vertical thrusters follow the convention.
- `myellow.xacro`: `thruster_portside_vertical_link` is at y = +0.120 and `thruster_starboard_vertical_link` at y = −0.120.

`farol2_allocation` and `auv_sim` both read these poses from TF, so the simulation is self-consistent. On hardware the RPM vector is ordered by the `frames` list, so the swap would reverse yaw torque on mred and roll torque on myellow unless the drivers compensate. `farol2_drivers` cannot settle it: it has no mred configuration, and the Medusa thruster drivers do not yet connect to `rpm_command` at all ([#39](#39-medusa-thruster-drivers-cannot-receive-rpm_command-high)). For myellow, `allocation.yaml` lists `thruster_starboard_vertical_link` before `thruster_portside_vertical_link`, which may be a deliberate compensation. Fix: swap the y signs (and the myellow order), or rename the links, once someone checks on the vehicle.

### 38. magicelectric rudder topics no longer match the drivers (High)

On `main`, `farol2_allocation` published the rudder command as `allocation/rudder_command`. On `dev` it publishes `farol2_interfaces/ControlSurfaceAngle` on `allocation/control_surface_angle` (`farol2_allocation/launch/farol2_allocation.launch.py:91`). `farol2_drivers_bringup/launch/magicelectric_drivers.launch.py` still remaps `can_instrumentation`'s `std_msgs/Float32` input `rudder_angle_ref` to `/<ns>/allocation/rudder_command`, which nothing publishes on `dev`. The real rudder therefore receives no commands. The measured angle has the same problem in reverse: the driver publishes `std_msgs/Float32` on `measurement/rudder_angle`, while `farol2_nav` expects `ControlSurfaceAngle` on `measurement/control_surface_angle`, so `asv_dynamics_model` runs with no rudder input. Fix: either publish both forms from allocation for a transition, or update `can_instrumentation` to the new message and topic.

### 39. Medusa thruster drivers cannot receive `rpm_command` (High)

`seabotix_duo` and `seabotix_solo` (`farol2_drivers/thrusters/`) are ROS 2 packages, but their launch files (`seabotix_myellow.launch`, `seabotix_duo.launch`) are ROS 1 XML with `<rosparam>`, which `ros2 launch` does not run. The nodes subscribe to `farol2_interfaces/Thruster` on `thrusters/commands` (`seabotix_duo.hpp:15`), while `farol2_allocation` publishes `ThrusterRPM` on `/<ns>/allocation/rpm_command`. The myellow configs also drive 4 thrusters (ids 0 to 3) while `myellow`'s `allocation.yaml` and URDF have 6. As it stands, no Medusa-class vehicle can be driven on hardware from `dev`. `can_thrusters` (magicelectric) does subscribe to `ThrusterRPM`, and requires exactly 2 entries (`can_thrusters.cpp:298`). That works today only because magicelectric runs with `allocation.thrusters.open_loop: true`, so the two-entry `open_loop` message is the only RPM command; switching magicelectric to closed-loop allocation, which lists one thruster frame, would make the driver reject every command.

### 40. Driver frame ids do not match the URDF (Medium)

`farol2_drivers_bringup/config/{magicelectric,myellow}/drivers.yaml` stamp the IMU as `ahrs`, the GNSS as `gnss` and the DVL as `dvl_link`, with no `<ns>/` prefix. The URDFs name these links `imu_link`, `gnss_link` and `dvl_link`, published as `<ns>/imu_link` and so on. `sample_and_hold` looks up `<ns>/base_link ← header.frame_id`, finds nothing and falls back to identity, so lever-arm and mounting-rotation compensation are off on every real vehicle, including myellow, which has the sensor links. Fix: use `<ns>/imu_link`-style ids in the drivers (or have them prepend `frame_prefix`).

### 41. The Simulink ROS 2 model is hard-wired to `/magicelectric0` (Medium)

`magicelectric-simulation-estimation/simulation/simulation/magicelectric_ros2_simulation.slx` subscribes to `allocation/rpm_command` and `allocation/control_surface_angle`, and publishes `measurement/{imu, gnss, velocity_over_ground, velocity_through_water, thruster_rpm, control_surface_angle}`, matching `magic_electric_sim`. Problems found in the model file:

- Every topic is the absolute `/magicelectric0/...`, so it only works for vehicle id 0, while `start_vehicle_sim` picks the first free id.
- It also subscribes to `measurement/rpm` and `measurement/rudder_angle`, which only the real drivers publish, and publishes `allocation/tau_u_real` and `allocation/tau_r_real`, which nothing reads.
- One publisher uses the relative name `magicelectric0/measurement/control_surface_angle`.
- `magicelectric_simulation_mpc.slx` is a byte-identical copy of `magicelectric_ros2_simulation.slx`.
- No `/clock` publisher block appears in the saved model, although its README relies on "the ROS2 clock inside your simulink" for speed-up, so how simulated time reaches the stack is unclear (inferred from the model XML; not run).

Generating the FAROL 2 messages for MATLAB (`ros2genmsg`) needs MATLAB R2025b with Python 3.10 and GCC 12, as its README explains.

