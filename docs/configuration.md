# Configuration and vehicles

## Where parameters live

All runtime parameters live in `farol2_bringup/config/<vehicle>/<config>/`. Each package launch file loads its own file from there:

| File | Loaded by | Node(s) (YAML key) |
|---|---|---|
| `nav.yaml` | `farol2_nav.launch.py` | `filter_node` (but the node is named `navigation_filter`, see [GAPS #1](GAPS.md#1-every-vehicles-navyaml-is-silently-ignored-high)) |
| `allocation.yaml` | `farol2_allocation.launch.py` | `actuator_allocation` |
| `inner_loop.yaml` | `farol2_inner_loop.launch.py` | `inner_loop`, `open_loop` |
| `waypoint.yaml` | `farol2_waypoint.launch.py` | `waypoint` |
| `path_following.yaml` | `farol2_path_following.launch.py` | `path_following` |
| `planning.yaml` | `farol2_planning.launch.py` | `paths` |
| `addons.yaml` | `farol2_console.launch.py` | `console_parser`, `console_server` |
| `sim.yaml` | `farol2_sim.launch.py`, `sim_clock.launch.py` | `auv_sim` or `magic_electric_sim`, `sim_clock` |

Every file uses the wildcard form so that it matches regardless of namespace:

```yaml
/**:
  <node_name>:
    ros__parameters:
      ...
```

The `<node_name>` must equal the `name=` given in the launch file, not the executable name.

Launch files also inject `vehicle_name`, `vehicle_id`, `use_sim_time` and, where TF is used, `frame_prefix = <ns>/`. The YAML is loaded after these, so a YAML value with the same name wins (this is why every vehicle's `console_parser.vehicle_id` is 3, [GAPS #18](GAPS.md#18-console_parser-vehicle-id-is-hard-coded-to-3-medium)).

### Overrides with `config_to_use`

The intended scheme is `default/` for the baseline and another folder (selected with `config_to_use:=<folder>`) layered on top. On `dev` only `path_following.yaml` and `sim_clock` honour it; the override entry is commented out everywhere else ([GAPS #7](GAPS.md#7-config_to_use-overrides-are-disabled-in-8-of-9-launch-files-medium)). The only override folder in the repository is `magicelectric/placeholder/`.

Package-local example configs (`farol2_nav/config/nav.yaml`, `farol2_path_following/config/path_following.yaml`) are reference material only; no launch file loads them.

## Vehicles

| Vehicle | Type (inferred from config) | Allocation | Thrusters (TF frames) | Simulator | `open_loop.surge` | Nav measurements in YAML |
|---|---|---|---|---|---|---|
| `mred` | Medusa class | `thrust` | 2 horizontal + 2 vertical | `auv_sim` | enabled (likely a copy error) | gnss, imu |
| `mblack` | Medusa class, surface thrusters only | `thrust` | portside, starboard | `auv_sim` | disabled | gnss, imu |
| `myellow` | Medusa class | `thrust` | 4 horizontal + 2 vertical | `auv_sim` | (no `open_loop` block) | gnss, imu, DVL, depth, altimeter, rpm |
| `bluerov` | BlueROV2 | `thrust` | 4 horizontal + 4 vertical | `auv_sim` | disabled | gnss, imu, DVL, depth, altimeter, rpm |
| `delfim` | DELFIM catamaran | `thrust` | portside, starboard | `auv_sim` | disabled | gnss, imu, altimeter, rpm |
| `magicelectric` | Surface vessel with rudder | `thrust_rudder` | middle thruster + middle rudder | `magic_electric_sim` | enabled | gnss, imu, DVL, rudder, rpm; filters `asv_dynamics_model`, `current_estimator` |

`start_vehicle_sim.launch.py` picks `magic_electric_sim` when `vehicle_name == magicelectric` and `auv_sim` otherwise. The README mentions SLOCUM gliders, but no glider configuration exists in FAROL 2 (the ROS 1 glider work is in `dsor-isr/slocum_glider`).

## Adding a vehicle

1. **Description**: add `farol2_description/urdf/<vehicle>.xacro` with `base_link`, one link per thruster (x axis along thrust) and rudder, and the sensor links the drivers stamp (`gnss_link`, `imu_link`, `dvl_link`, `depth_link`, `altimeter_link`). Missing sensor links make navigation fall back to identity transforms.
2. **Config**: copy `farol2_bringup/config/<closest>/default/` to `farol2_bringup/config/<vehicle>/default/` and edit:
   - `allocation.yaml`: `allocation.type`, `allocation.thrusters.frames` in the same order as the RPM vector the drivers expect, `rpm_conversion.coef_fwd/bwd` (force = c0·RPM² + c1·RPM + c2), RPM limits; for `thrust_rudder`, the rudder limits, `cm_distance`, `model.K_*` and propeller model.
   - `inner_loop.yaml`: enable only the axes the vehicle can actuate; set `tau_min/max` to what allocation can deliver.
   - `nav.yaml`: list the measurements the drivers publish and the filters to run.
   - `path_following.yaml`: include gains for **every** algorithm, including `ilos`, otherwise selecting it crashes the node.
   - `sim.yaml`: hydrodynamic model and actuator frames matching the URDF.
3. **Drivers**: for real hardware, add `<vehicle>_drivers.launch.py` to `farol2_drivers_bringup` and a profile in `farol2_cli/config/drivers.bash`.
4. **Launch**: `ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=<vehicle>`.

## Parameter reference

Per-node parameters are listed in each package page under [packages/](packages/README.md).
