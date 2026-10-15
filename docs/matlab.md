# MATLAB and Simulink

FAROL 2 itself has no MATLAB code. The MATLAB work for the stack lives in [`dsor-isr/magicelectric-simulation-estimation`](https://github.com/dsor-isr/magicelectric-simulation-estimation) (checked at `53a4f34`, 2026-07-09) and covers one vehicle, magicelectric.

## What the repository contains

| Folder | Contents |
|---|---|
| `raw_data/` | ROS bags from the 2025 identification trials, converted to `.mat` |
| `estimation/surge/`, `estimation/yaw_rate/` | Scripts that build identification data and fit the surge and yaw-rate models (published reports in `html/`) |
| `estimation/rudder_subsystem/` | Rudder actuator analysis |
| `generated_data/` | Identification-ready `.mat` data |
| `simulation/simulation/` | Simulink models: `magicelectric_model.slx`, `magicelectric_simulation.slx`, `magicelectric_open_loop.slx`, `magicelectric_ros2_simulation.slx`, and `load_ros2_simulation.m` (parameters) |
| `simulation/simulation/.hinf_control/` | H∞ yaw-controller experiments |

The parameters in `load_ros2_simulation.m` (`m_u = 3100`, `X_uu = −96.227`, `N_r = −14.334`, `N_rr = −260.675`, `m_r = 88.479`, `K_L`, `K_D0`, `K_D1`, `K_T_BP`) are the same as `farol2_bringup/config/magicelectric/default/sim.yaml`. `farol2_sim`'s `magic_electric_sim` is the C++ port of this model, so it is the usual simulator; the Simulink model is the reference and the place to change the dynamics.

## Running FAROL 2 against the Simulink model

`magicelectric_ros2_simulation.slx` replaces `magic_electric_sim`: it reads the actuator commands and publishes the sensor topics over ROS 2.

| Direction (Simulink) | Topic | Type |
|---|---|---|
| sub | `/magicelectric0/allocation/rpm_command` | `farol2_interfaces/ThrusterRPM` |
| sub | `/magicelectric0/allocation/control_surface_angle` | `farol2_interfaces/ControlSurfaceAngle` |
| sub | `/magicelectric0/measurement/rpm` | `farol2_interfaces/ThrusterRPM` (real driver topic) |
| sub | `/magicelectric0/measurement/rudder_angle` | `std_msgs/Float32` (real driver topic) |
| pub | `/magicelectric0/measurement/imu` | `sensor_msgs/Imu` |
| pub | `/magicelectric0/measurement/gnss` | `sensor_msgs/NavSatFix` |
| pub | `/magicelectric0/measurement/velocity_over_ground`, `velocity_through_water` | `geometry_msgs/Vector3Stamped` |
| pub | `/magicelectric0/measurement/thruster_rpm` | `farol2_interfaces/ThrusterRPM` |
| pub | `magicelectric0/measurement/control_surface_angle` | `farol2_interfaces/ControlSurfaceAngle` |
| pub | `/magicelectric0/allocation/tau_u_real`, `tau_r_real` | `std_msgs/Float32` (debug) |

Setup, from the repository's `simulation/simulation/README.md`:

1. MATLAB R2025b with ROS Toolbox, configured with **Python 3.10** and **GCC 12**.
2. Make a workspace of symlinks to the FAROL 2 packages that define messages (`farol2_interfaces`, `farol2_planning`, `farol2_inner_loop`, `farol2_path_following`, `farol2_waypoint`, `farol2_nav`, and `farol2_drivers` for its messages) and run `ros2genmsg("<workspace>")` in MATLAB. Add packages one at a time if the machine runs out of memory.
3. Run `load_ros2_simulation.m`, start the Simulink model, then launch the FAROL 2 stack **without** the C++ simulator, for example `start_farol2.launch.py vehicle_name:=magicelectric vehicle_id:=0`.

The model only works as vehicle id 0, listens to two driver topics, and has no visible `/clock` publisher ([GAPS #41](GAPS.md#41-the-simulink-ros-2-model-is-hard-wired-to-magicelectric0-medium)).

## Other vehicles

There are no MATLAB models or MATLAB interfaces for mred, mblack, myellow, bluerov or delfim. `farol2_sim/include/sim_utilis/AUV.hpp` describes itself as "a Matlab-like simulation of an AUV in C++", but no MATLAB source for it was found. The path-following algorithms come from a survey whose MATLAB toolbox is the external `hungrepo/path-following-Matlab`.
