# Farol Bringup Package

## Description

The *farol_bringup* ROS node is responsible for handling a list of pre-defined processes, taking care of starting, stopping and restarting them (via a ROS service) and providing information about their status (via a ROS topic).

Each of these processes is assigned a specific launch file, which in turn runs a series of other ROS nodes. These specific launch files are usually under a common theme (e.g. control, navigation, etc.) and are all available under `farol_bringup/launch/`.

Moreover, the *farol_bringup* node also creates temporary copies of the `ros.yaml` configuration files, both default (under `farol_bringup/config_default/`) and personal (under `[personal_bringup]/config_personal/`), substituting the `#vehicle#` keyword for the launched vehicle's namespace (the vehicle name followed by an ID). These copies are saved under `[personal_bringup]/config_personal/.ros_tmp/` while the stack is running, for easy access for other ROS nodes.

## Vehicle Launchers

Two top-level launch files are provided under `farol_bringup/vehicle_launchers/` to bring up the full stack:

* **`start_vehicle_sim.launch.py`** — Launches the vehicle simulation (sensor sim, vehicle dynamics, etc.) together with the full Farol stack. **Always forces `use_sim_time:=true`**, so all nodes use the simulated clock.

* **`start_vehicle.launch.py`** — Launches everything (full Farol stack, drivers, etc.) but **without** simulation, i.e. `use_sim_time` is **false**. Intended for real hardware deployments.

## Nodes

* [farol_bringup](farol_bringup.md)