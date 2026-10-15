# farol2_description

Vehicle geometry as URDF (xacro), published to TF. `farol2_allocation`, `auv_sim` and `farol2_nav` read thruster and sensor poses from these frames, so the URDF is part of the control configuration, not just visualisation.

The repository's own README mentions `meshes/` and `config/` folders; neither exists. There are no meshes, so the models have frames only.

## Launch

`tf_static.launch.py` runs `robot_state_publisher` in `/<ns>` with `robot_description` from `urdf/<vehicle_name>.xacro` and `frame_prefix: <ns>/`. Fixed joints go to `/tf_static` as `<ns>/base_link → <ns>/<link>`.

| Argument | Default |
|---|---|
| `vehicle_name` | `vehicle` |
| `vehicle_id` | `0` |
| `use_sim_time` | `false` |

`farol2_bringup` includes it for every vehicle. magicelectric's rudder joints are `continuous`, so their transforms come from `joint_states`, which `magic_electric_sim` publishes; there is no `joint_state_publisher` for the real vehicle.

## Conventions

- `base_link` uses the marine body frame: x forward, y starboard, z down.
- A thruster pushes along its link's +x axis. Vertical thrusters are pitched by −π/2 so +x points down.
- Sensor links are named `gnss_link`, `imu_link`, `dvl_link`, `depth_link` and `altimeter_link`, matching the `frame_id` that `auv_sim` sets. `sample_and_hold` uses them for lever-arm compensation; a missing link is treated as identity.

## Vehicles

| File | Links besides `base_link` | Notes |
|---|---|---|
| `mred.xacro` | 2 horizontal + 2 vertical thrusters | Horizontal port/starboard y signs look swapped ([GAPS #37](../../GAPS.md#37-some-thrusters-are-on-the-wrong-side-in-the-urdf-low-unconfirmed)) |
| `myellow.xacro` | 4 horizontal (±45°) + 2 vertical thrusters, `dvl_link`, `gnss_link`, `imu_link`, `depth_link`, `altimeter_link` | The only model with sensor frames ([GAPS #24](../../GAPS.md#24-only-myellow-defines-sensor-frames-low)); vertical y signs look swapped ([GAPS #37](../../GAPS.md#37-some-thrusters-are-on-the-wrong-side-in-the-urdf-low-unconfirmed)) |
| `mblack.xacro` | 2 thrusters | |
| `delfim.xacro` | 2 thrusters (±2 m) | |
| `bluerov.xacro` | 4 vectored horizontal + 4 vertical thrusters | |
| `magicelectric.xacro` | `thruster_{middle,left,right}_link`, `rudder_{middle,left,right}_link` (continuous) | Allocation uses only the middle thruster and rudder |

## Adding a vehicle

1. Create `urdf/<vehicle_name>.xacro` with `base_link`, one link per thruster (x along thrust) and one per sensor.
2. List the thruster links, in RPM order, in `farol2_bringup/config/<vehicle_name>/default/allocation.yaml` (`allocation.thrusters.frames`) and, for the simulator, `sim.yaml` (`vehicle.actuators.frames`).
3. Check with `ros2 run tf2_ros tf2_echo <ns>/base_link <ns>/<link>`.

See [configuration.md](../../configuration.md#adding-a-vehicle) for the rest.
