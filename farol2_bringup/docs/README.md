# farol2_bringup

## Description
`farol2_bringup` stores the per-vehicle configuration and the top-level launch files. It has no runtime process-manager node (the ROS 1 `process.yaml` bringup was replaced by hand-edited launch files).

```text
farol2_bringup/
  config/<vehicle>/<config_name>/    # vehicles: bluerov delfim magicelectric mblack mred myellow; config_name: default
    addons.yaml allocation.yaml inner_loop.yaml nav.yaml path_following.yaml planning.yaml sim.yaml waypoint.yaml
  launch/
    start_vehicle_sim.launch.py      # sim_clock + simulator + GNC stack
    start_vehicle_real.launch.py     # drivers + GNC stack
    start_farol2.launch.py           # the GNC stack only (nav, tf_static, allocation, inner_loop, waypoint, path_following, planning, console)
    bag_record.launch.py             # ros2 bag record -a into $ROSDATA_DIR
  farol2_bringup/__init__.py         # get_next_available_vehicle_id()
```

## Launch files
| File | What it does |
|---|---|
| `start_vehicle_sim.launch.py` | Starts `sim_clock` (arg `farol2_clock`, default true), `farol2_sim` (`magic_electric_sim` for `vehicle_name == magicelectric`, otherwise `auv_sim`), then `start_farol2`. Simulated time comes from the `/clock` published by `sim_clock`; the stack is launched with `use_sim_time:=false`. |
| `start_vehicle_real.launch.py` | Includes `farol2_drivers_bringup/<vehicle_name>_drivers.launch.py` (not part of the returned description at the moment) and `start_farol2` with `use_sim_time:=false`. Driver toggles are hard-coded in the file: copy it into your own bringup package and edit. |
| `start_farol2.launch.py` | The GNC stack. Passes `vehicle_name`, `vehicle_id`, `use_sim_time`, `config_to_use` to each package launch file. Nav: `sample_and_hold=false`, `filter_node=true`. Inner loop: `pid=true`, `open_loop=true`. |
| `bag_record.launch.py` | Args: `vehicle_name`, `vehicle_id`, `compression`, `prefix`. Needs `ROSDATA_DIR` in the environment. |

Common arguments: `vehicle_name` (default `magicelectric`), `vehicle_id` (empty = next free id, found by looking at the ROS graph), `config_to_use` (config folder, default `default`; only `path_following` currently loads an override from it).

The vehicle namespace is `/<vehicle_name><vehicle_id>` (for example `/delfim0`).

## Configuration files
Each YAML is wrapped in `/**:` followed by the **ROS node name** and `ros__parameters:`. The node name must match the name in the launch file:

| File | Node key(s) |
|---|---|
| `nav.yaml` | `filter_node` (see warning) |
| `allocation.yaml` | `actuator_allocation` |
| `inner_loop.yaml` | `inner_loop`, `open_loop` |
| `waypoint.yaml` | `waypoint` |
| `path_following.yaml` | `path_following` |
| `planning.yaml` | `paths` |
| `sim.yaml` | `sim_clock`, `auv_sim` or `magic_electric_sim` |
| `addons.yaml` | `console_parser`, `console_server` |

**Warning:** `farol2_nav` is launched with node name `navigation_filter`, but `nav.yaml` is keyed `filter_node`, so none of its values reach the node (a live `ros2 param get /delfim0/navigation_filter measurements` returns the code defaults `['gnss','imu']`, not the YAML list). Rename the key to `navigation_filter` to apply the YAML.

Add a vehicle by copying a folder and adapting the YAML files, plus a `farol2_description/urdf/<vehicle>.xacro`. See [docs/runbook.md](../../docs/runbook.md) for the common procedures.
