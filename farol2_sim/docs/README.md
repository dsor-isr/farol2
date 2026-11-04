# Vehicle Simulator Package

## Description

This package provides simulators for DSOR vehicles. The simulators receive actuator commands and output realistic simulated states to the sensor interfaces. Key features include hydrodynamic current modeling and vehicle-specific allocation simulation.

## Nodes

**auv_sim**
- Receives thruster RPM commands and outputs the corresponding simulated vehicle state
- Designed for AUVs but compatible with any thruster-based maritime vehicle

**magic_electric_sim**
- Receives thruster RPM commands and rudder control inputs (commands or reference setpoints)
- Optionally simulates rudder behavior based on user configuration in the YAML file
- C++ implementation of the Simulink-based simulator

**sim_clock**
- Publishes the simulation `/clock`
- Launched separately from the simulator nodes because it must run with `use_sim_time: false`

## Launch
`launch/farol2_sim.launch.py` (args `vehicle_name`, `vehicle_id`, `config_to_use`, `auv_sim`, `magic_electric_sim`) starts one simulator under `/<ns>/sim`; `launch/sim_clock.launch.py` starts `sim_clock`. `start_vehicle_sim.launch.py` in `farol2_bringup` starts both plus the GNC stack and picks `magic_electric_sim` only for `vehicle_name == magicelectric`. The simulator nodes run with `use_sim_time: true` (hard-coded in the launch file).

## Topics (relative to `/<ns>/`)
| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `allocation/rpm_command` | `ThrusterRPM` | `magic_electric_sim` uses `rpm[0]`; `auv_sim` uses one entry per thruster. |
| in | `allocation/control_surface_angle` | `ControlSurfaceAngle` | `magic_electric_sim` when `sim_rudder: false` (rudder angle taken directly from the controller). |
| in | `sim/rudder_cmd` | `std_msgs/Float32` | `magic_electric_sim` when `sim_rudder: true`. |
| out | `measurement/imu`, `gnss`, `ned_utm`, `velocity_over_ground`, `velocity_through_water`, `depth` | sensor msgs / `farol2_interfaces` | Frame ids `<ns>/gnss_link`, `dvl_link`, `depth_link`, `base_link` (see the note about missing URDF sensor frames in `farol2_description`). |
| out | `measurement/control_surface_angle` | `ControlSurfaceAngle` | `magic_electric_sim` when `sim_rudder: true`. |
| out | `sim/{position,body_velocity,orientation,orientation_rate,body_acceleration,angular_acceleration}` | `geometry_msgs/Vector3` | Debug. Orientations in degrees. |
| out | `/clock` (`sim_clock`) | `rosgraph_msgs/Clock` | |
| out | `/tf` | `world -> <ns>/base_link` | Parent frame `world` has no prefix. |

`measurement/thruster_rpm` (used by `farol2_nav`'s `asv_dynamics_model`) is not published by either simulator.

## Parameters (`farol2_bringup/config/<vehicle>/default/sim.yaml`)
Keys `sim_clock` (`node_frequency`, `speedup`) and `auv_sim` or `magic_electric_sim`:
- `initial_state.{position [lat, lon, depth], body_velocity, orientation, orientation_rate}`. Note `magicelectric` starts with `body_velocity: [1.78, 0, 0]`, so it already moves before any command.
- `environment.{fluid_density, current [speed, direction_deg]}`.
- `vehicle.*`: mass, hydrodynamic coefficients (`m_u`, `X_u`, `X_uu`, `m_uv`, `N_r`, `N_rr`, `Y_v`, `Y_vv`, `m_r`, rudder `K_L`, `K_D0`, `K_D1`) and `allocation`: `rudder.{sim_rudder, max_angle_deg, min_angle_deg, ang_vel}` and `propulsion.{fixed_rpm, fixed_rpm_value, max_rpm, min_rpm, K_T_BP, prop_pitch, D, l_, deadzone_pos, deadzone_neg}`. With `fixed_rpm: true` (the magicelectric default) the simulated propeller ignores `rpm_command`.
- `sensor.{gnss, gnss_velocity_over_ground, dvl_activate, dvl.{mode, output_frame}, noise.*}` (the AUV file also lists the DVL).
- Parameters are declared without defaults in `magic_electric_sim`; a missing key stops the node at start-up. The delfim values are flagged as unreliable in the file.
