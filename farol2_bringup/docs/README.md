# Farol Bringup Package

## Description

`farol2_bringup` is a support package that stores:

* default configuration files under `config_default/`
* reusable launch files under `launch/`
* reference vehicle launcher templates under `vehicle_launchers/`

It does not provide a runtime bringup process-manager node anymore.

## Vehicle Launchers

Two top-level launch files are provided under `farol2_bringup/vehicle_launchers/` to bring up the full stack:

* **`start_vehicle_sim.launch.py`** — Launches the vehicle simulation (sensor sim, vehicle dynamics, etc.) together with the full Farol stack. **Always forces `use_sim_time:=true`**, so all nodes use the simulated clock.

* **`start_vehicle_real.launch.py`** — Launches the real-hardware stack (drivers + Farol stack), i.e. with `use_sim_time:=false`.

## Nodes

None.