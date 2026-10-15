# farol2_bringup

Launch files and per-vehicle configuration. There is no bringup node: the ROS 1 process manager (`farol_bringup` with `process.yaml`) was replaced by plain launch files.

The repository's `docs/README.md` refers to `config_default/` and `vehicle_launchers/` folders and says the simulation launch forces `use_sim_time:=true`; on `dev` the folders are `config/` and `launch/`, and the stack is launched with `use_sim_time: false` ([GAPS #2](../../GAPS.md#2-simulation-and-stack-run-on-different-clocks-high)).

## Launch files

| File | What it starts |
|---|---|
| `start_vehicle_sim.launch.py` | `sim_clock` (unless `farol2_clock:=false`), the simulator for the vehicle, then `start_farol2.launch.py` |
| `start_vehicle_real.launch.py` | `start_farol2.launch.py`; the drivers include (`farol2_drivers_bringup/<vehicle>_drivers.launch.py`) is built but left out of the returned actions |
| `start_farol2.launch.py` | The GNC stack: nav, tf_static, allocation, inner_loop (+ open_loop), waypoint, path_following, planning, console. Edit this file to add or remove packages |
| `bag_record.launch.py` | `ros2 bag record -a -o <prefix>_<timestamp>`, optionally compressed |

The full include tree is in [architecture.md](../../architecture.md#launch-structure).

### Arguments

| Argument | Files | Default | Meaning |
|---|---|---|---|
| `vehicle_name` | all | `magicelectric` (`vehicle` in `bag_record`) | Selects `config/<vehicle_name>/`, the URDF and the simulator |
| `vehicle_id` | all | empty (sim, real), `0` (`start_farol2`, `bag_record`) | Namespace suffix. When empty, the first id not used by any `/<name><digits>` namespace in the graph is picked, so ids are shared across vehicle types |
| `config_to_use` | sim, real, `start_farol2` | `default` | Override folder; ignored by most packages ([GAPS #7](../../GAPS.md#7-config_to_use-overrides-are-disabled-in-8-of-9-launch-files-medium)) |
| `use_sim_time` | `start_farol2` | `false` | Forced to `false` by the sim and real launch files |
| `farol2_clock` | sim | `true` | Start `sim_clock`; set `false` for the second and later simulated vehicles ([GAPS #19](../../GAPS.md#19-each-vehicle-starts-its-own-clock-medium)) |
| `compression`, `prefix` | `bag_record` | `false`, `<vehicle_name><vehicle_id>` | Bag compression and output name prefix |

```bash
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=mred
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=mblack farol2_clock:=false   # second vehicle → /mblack1
ros2 launch farol2_bringup bag_record.launch.py vehicle_name:=mred vehicle_id:=0
```

## Configuration

`config/<vehicle>/default/` holds one YAML per package (`nav`, `allocation`, `inner_loop`, `waypoint`, `path_following`, `planning`, `addons`, `sim`). Six vehicles are configured: `mred`, `mblack`, `myellow`, `bluerov`, `delfim`, `magicelectric` (plus `magicelectric/placeholder/` as an override example). File-by-file details, the YAML key rules and how to add a vehicle are in [configuration.md](../../configuration.md).

## Python helper

`farol2_bringup.get_next_available_vehicle_id(vehicle_name)` creates a temporary `rclpy` node, waits 0.25 s for discovery and returns the lowest free id. It returns `0` if `rclpy` is unavailable or discovery fails, so two vehicles started at the same moment can get the same id.

## Known gaps

[#2](../../GAPS.md#2-simulation-and-stack-run-on-different-clocks-high), [#3](../../GAPS.md#3-two-nodes-publish-to-the-same-rpm_command-topic-high), [#7](../../GAPS.md#7-config_to_use-overrides-are-disabled-in-8-of-9-launch-files-medium), [#19](../../GAPS.md#19-each-vehicle-starts-its-own-clock-medium), [#28](../../GAPS.md#28-real-vehicle-bringup-is-incomplete-medium), [#38](../../GAPS.md#38-magicelectric-rudder-topics-no-longer-match-the-drivers-high), [#39](../../GAPS.md#39-medusa-thruster-drivers-cannot-receive-rpm_command-high).
