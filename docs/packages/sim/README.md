# farol2_sim

Vehicle simulators and the simulation clock. The simulators take the actuator commands that `farol2_allocation` publishes and produce the same `measurement/*` topics that the real drivers would, so the rest of the stack runs unchanged.

| Node | Executable | Used for |
|---|---|---|
| `auv_sim` | `auv_sim` | Every vehicle except magicelectric. 6-DOF rigid body with diagonal added mass and linear and quadratic damping, driven by thruster RPM |
| `magic_electric_sim` | `magic_electric_sim` | magicelectric only. 3-DOF surface vessel with one propeller and a rudder (C++ port of a Simulink model) |
| `sim_clock` | `sim_clock` | Publishes `/clock` |

For magicelectric, the original Simulink model can replace `magic_electric_sim` over ROS 2; see [MATLAB and Simulink](../../matlab.md).

`farol2_bringup/start_vehicle_sim.launch.py` picks the simulator from `vehicle_name` and includes `sim_clock.launch.py` and `farol2_sim.launch.py`. The simulators run in `/<ns>/sim` with `use_sim_time: true`.

## sim_clock

Publishes `rosgraph_msgs/Clock` on `/clock` (reliable, transient local). Simulated time advances by `1/node_frequency` per tick, and the tick runs on a wall timer every `1/(node_frequency · speedup)` s, so `speedup: 2.0` runs the simulation twice as fast as real time.

| Parameter | Type | Meaning |
|---|---|---|
| `node_frequency` | int | Clock steps per simulated second (falls back to 10 if ≤ 0) |
| `speedup` | double | Simulated / real time ratio (falls back to 1.0 if ≤ 0) |

The node itself uses wall time. Every vehicle launch starts its own `sim_clock`, so a multi-vehicle simulation has several `/clock` publishers ([GAPS #19](../../GAPS.md#19-each-vehicle-starts-its-own-clock-medium)). The navigation, allocation and other nodes are launched with `use_sim_time: false` ([GAPS #2](../../GAPS.md#2-simulation-and-stack-run-on-different-clocks-high)).

## Topics (both simulators)

Names are relative to `/<ns>`.

| Direction | Topic | Type | Notes |
|---|---|---|---|
| sub | `allocation/rpm_command` | `farol2_interfaces/ThrusterRPM` | magicelectric uses only `rpm[0]` |
| sub | `allocation/control_surface_angle` | `farol2_interfaces/ControlSurfaceAngle` (deg) | magicelectric, when `sim_rudder: false` |
| sub | `sim/rudder_cmd` | `std_msgs/Float32` (−1…1) | magicelectric, when `sim_rudder: true`; nothing in the stack publishes it |
| pub | `measurement/imu` | `sensor_msgs/Imu` | `sensor.imu` |
| pub | `measurement/gnss` | `sensor_msgs/NavSatFix` | `sensor.gnss` |
| pub | `measurement/ned_utm` | `geometry_msgs/Vector3Stamped` | magicelectric only (commented out in `auv_sim`) |
| pub | `measurement/velocity_over_ground` | `geometry_msgs/Vector3Stamped` | GNSS velocity in **NED** (`sensor.gnss_velocity_over_ground`), or DVL bottom track |
| pub | `measurement/velocity_through_water` | `geometry_msgs/Vector3Stamped` | DVL water track (`auv_sim`), or the hull model (magicelectric) |
| pub | `measurement/depth` | `farol2_interfaces/Depth` | `sensor.depth_sensor` |
| pub | `measurement/control_surface_angle` | `farol2_interfaces/ControlSurfaceAngle` | magicelectric |
| pub | `joint_states` | `sensor_msgs/JointState` | magicelectric (rudder joint for the URDF) |
| pub | `sim/{position, body_velocity, orientation, orientation_rate, body_acceleration, angular_acceleration}` | `geometry_msgs/Vector3` | Ground truth |

No simulator publishes `measurement/altimeter` ([GAPS #16](../../GAPS.md#16-auv-simulator-publishes-no-altimeter-medium)). The GNSS velocity is in NED while `farol2_nav` treats it as body frame ([GAPS #4](../../GAPS.md#4-gnss-velocity-ned-is-treated-as-a-body-frame-velocity-high)).

## auv_sim parameters

| Group | Keys | Notes |
|---|---|---|
| top level | `node_frequency` (int), `frame_prefix` | Integration and publication rate |
| `initial_state` | `position` [lat, lon, depth], `body_velocity`, `orientation`, `orientation_rate` | Geodetic start; UTM zone is derived from it |
| `environment` | `fluid_density` (kg/l), `current.{mean, sigma, minimum, maximum}` [3] | Random current. `sigma: 0` aborts under libstdc++ assertions ([GAPS #15](../../GAPS.md#15-auv_sim-aborts-when-current-sigma-is-0-under-libstdc-assertions-medium)) |
| `vehicle` | `mass`, `zg`, `vehicle_density`, `inertia_tensor` [3], `linear_damping_tensor` [6], `quadratic_damping_tensor` [6], `added_mass_tensor` [6] | Diagonal terms |
| `vehicle.actuators` | `frames` + `base_frame` **or** `allocation_matrix` (flat) | With `frames`, thruster poses come from TF, like `farol2_allocation` |
| `vehicle.actuators` | `lump_param_positive`, `lump_param_negative` [3], `min_max_thruster_input` [2], `pole`, `delay`, `period` | Thrust curve `a·|x|² + b·|x| + c`, RPM limits, first-order pole and delay. `period` is ignored (the node period is used) |
| `sensor` | `gnss`, `gnss_velocity_over_ground`, `depth_sensor`, `imu`, `dvl_activate` (bool) | Which sensors publish |
| `sensor.dvl` | `mode` (`bottom_track` / `water_track`), `output_frame` (`body` / other = inertial) | Any other `mode` throws |
| `sensor.noise` | `activate`, then `bias` and `variance` [3] for `position`, `orientation`, `body_velocity`, `fluid_velocity`, `orientation_rate` | Gaussian noise on outputs |

## magic_electric_sim parameters

| Group | Keys |
|---|---|
| top level | `node_frequency`, `frame_prefix` |
| `initial_state` | `position`, `body_velocity`, `orientation`, `orientation_rate` |
| `environment` | `fluid_density`, `current` [speed, direction deg] |
| `vehicle` | `mass`, `zg`, hull `m_u`, `X_u`, `X_uu`, `m_uv`, `Y_v`, `Y_vv`, `m_r`, `N_r`, `N_rr`, rudder `K_L`, `K_D0`, `K_D1` |
| `vehicle.allocation.rudder` | `sim_rudder` (simulate actuator dynamics from `rudder_cmd`), `max_angle_deg`, `min_angle_deg`, `ang_vel` (deg/s) |
| `vehicle.allocation.propulsion` | `fixed_rpm`, `fixed_rpm_value`, `max_rpm`, `min_rpm`, `K_T_BP`, `prop_pitch`, `D`, `l_`, `deadzone_pos`, `deadzone_neg` |
| `sensor` | Same switches and noise blocks as `auv_sim`, without DVL and fluid velocity |

The shipped magicelectric config has `fixed_rpm: true` (660 RPM), so the simulated propeller ignores RPM commands.

## Known gaps

[#2](../../GAPS.md#2-simulation-and-stack-run-on-different-clocks-high), [#4](../../GAPS.md#4-gnss-velocity-ned-is-treated-as-a-body-frame-velocity-high), [#15](../../GAPS.md#15-auv_sim-aborts-when-current-sigma-is-0-under-libstdc-assertions-medium), [#16](../../GAPS.md#16-auv-simulator-publishes-no-altimeter-medium), [#19](../../GAPS.md#19-each-vehicle-starts-its-own-clock-medium).
