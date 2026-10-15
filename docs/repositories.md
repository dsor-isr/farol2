# Repositories and branches

## Repositories that make up the stack

| Repository | Role | State on 2026-10-15 |
|---|---|---|
| [`dsor-isr/farol2`](https://github.com/dsor-isr/farol2) | ROS 2 Jazzy GNC stack (this documentation) | Active. `dev` at `693f523` (2026-07-30) |
| [`dsor-isr/dsor_simulation-ros2`](https://github.com/dsor-isr/dsor_simulation-ros2) | Simulation bringup; `farol2` as a submodule | Stale: last commit 2025-07-24, submodule pinned 273 commits behind `dev`, launches removed packages ([GAPS #6](GAPS.md#6-dsor_simulation-ros2-launches-packages-that-no-longer-exist-high)) |
| [`dsor-isr/farol2_drivers`](https://github.com/dsor-isr/farol2_drivers) (with `farol2_drivers_bringup`) | Hardware drivers: VectorNav VN-310/VN-100, CAN thrusters and instrumentation, Airmar 200WX/DX900, Ashtech GNSS, serial brushed-motor wrapper, BlueROV interface | `main` at `f210008` (2026-07-23). Launch files only for magicelectric and the towed payload; Medusa thruster drivers are half-ported from ROS 1 ([GAPS #28](GAPS.md#28-real-vehicle-bringup-is-incomplete-medium), [#38](GAPS.md#38-magicelectric-rudder-topics-no-longer-match-the-drivers-high), [#39](GAPS.md#39-medusa-thruster-drivers-cannot-receive-rpm_command-high)) |
| [`dsor-isr/magicelectric-simulation-estimation`](https://github.com/dsor-isr/magicelectric-simulation-estimation) | MATLAB identification scripts and Simulink models for magicelectric, including a ROS 2 simulation | `main` at `53a4f34` (2026-07-09). See [MATLAB and Simulink](matlab.md) |
| [`dsor-isr/farol`](https://github.com/dsor-isr/farol) | ROS 1 Noetic predecessor, with a full mkdocs site | Reference for algorithms and the original docs |
| [`ros-drivers/nmea_msgs`](https://github.com/ros-drivers/nmea_msgs) | Submodule at `farol2/3rd_party/nmea_msgs` | Third party |

Other DSOR repositories (`dsor_simulation`, `farol_gazebo`, `bluerov`, `slocum_glider`, `mission_planner`, `cais`, `dsor_utils`, and the archived `medusa_*`) are ROS 1-era and are not used by FAROL 2.

## MATLAB

The MATLAB and Simulink work is in `magicelectric-simulation-estimation` and covers magicelectric only: system identification and a Simulink model that can stand in for `magic_electric_sim` over ROS 2. Details, topics and setup are on [MATLAB and Simulink](matlab.md). There is no MATLAB code in `farol2` or `dsor_simulation-ros2`.

## Branches in `farol2`

| Branch | Purpose (from `WORKFLOW.md`) | Notes |
|---|---|---|
| `main` | Field-validated code | Last updated 2026-05-28 (`75b4d9f`). An ancestor of `dev`, 170 files behind it |
| `dev` | Latest integrated code | Adds `farol2_cli`, the filter-pipeline navigation, TF-based allocation, `sim_clock`, DVL simulation, depth messages |
| `feature/cpf_control` | Cooperative path following (`farol2_cpf_controller`, `CPFGamma.msg`) | Marked untested in its commits |
| `feature/motion_planning` | Bézier motion planner (`farol2_motion_planning`), merges `cpf_control` | Work in progress; adds `farol2_clock`, which `dev` later folded into `farol2_sim` |

## Main differences between `main` and `dev`

- `farol2_allocation`: separate `wrench_manager`, `thruster_allocation` and `rpm_conversion` nodes on `main`; one `allocation_node` that reads thruster poses from TF on `dev`.
- `farol2_nav`: `position_current_ekf` on `main`; `current_estimator`, `asv_dynamics_model` and debug messages on `dev`.
- `farol2_inner_loop`: `pid` node on `main`; `inner_loop_node` on `dev`.
- New on `dev`: `farol2_cli`, `farol2_sim/sim_clock`, `farol2_interfaces/Depth`, `ControlSurfaceAngle`, `ThrusterRPM`, `farol2_utils/tf_utils`.
- Namespaces: `dev` flattened `/<ns>/nav/…`, `/<ns>/control/…` node namespaces into `/<ns>` in its last commit, which caused [GAPS #1](GAPS.md#1-every-vehicles-navyaml-is-silently-ignored-high).
