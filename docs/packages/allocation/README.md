# farol2_allocation

Control allocation: turns the body wrench requested by the inner loops into thruster RPM and, for thruster-rudder vehicles, a rudder angle. One node, `actuator_allocation` (executable `allocation_node`), built from three libraries: `static_thruster_allocator`, `rudder_allocator` and `thruster_rpm_converter`.

On `main` these were three separate nodes (`wrench_manager`, `thruster_allocation`, `rpm_conversion`); the old theory page is preserved as [old_README.md](old_README.md).

## How it works

1. Each wrench component arrives on its own `std_msgs/Float32` topic. On every tick, components older than two periods are treated as 0, so a channel that stops publishing cannot leave a stale command active. If no component is fresh, nothing is published.
2. **Thruster geometry comes from TF.** At start-up the node retries every 500 ms until it can look up `<ns>/base_link ← <ns>/<thruster frame>` for every frame in `allocation.thrusters.frames`. Each thruster pushes along its own frame's x axis; position gives the moment arm. This builds the 6×r thrust configuration matrix `T`, and the node uses its pseudo-inverse: `f = T⁺ τ`.
3. Forces are converted to RPM per thruster and clipped to `[min_rpm, max_rpm]` independently (saturation is not redistributed among thrusters).

### `thrust` mode

All six wrench components are allocated to the thrusters. Force-to-RPM uses the static calibration `F = c0·RPM² + c1·RPM + c2`, with `coef_fwd` for positive and `coef_bwd` for negative force.

### `thrust_rudder` mode

- Only the surge force goes to the thrusters (common mode); yaw torque goes to the rudder.
- Rudder angle is `tau_z / (K_s · cm_distance · 1.75)`, clipped to the limits. The documented model also divides by the inflow speed squared, but the code does not ([GAPS #35](../../GAPS.md#35-rudder-allocation-mixes-degrees-and-radians-and-ignores-flow-speed-high)).
- With `rpm_conversion.mode: 0`, force-to-RPM uses the propeller model `F = ρ D⁴ K_T (n² − n·u/P)` with `u = velocity_through_water.x`; otherwise the static curve.
- Nothing is published while `mission_status` is 0.
- With `allocation.thrusters.open_loop: true`, only the rudder command is published; RPM is left to `open_loop` in `farol2_inner_loop`.

## Topics

| Direction | Topic | Type | Mode |
|---|---|---|---|
| sub | `allocation/{thrust,torque}_{x,y,z}` | `std_msgs/Float32` (N, N·m) | both |
| sub | `nav/filter/state` | `farol2_interfaces/NavigationState` | `thrust_rudder` |
| sub | `mission_status` | `std_msgs/Int8` | `thrust_rudder` |
| pub | `allocation/rpm_command` | `farol2_interfaces/ThrusterRPM` (order = `frames` order) | both |
| pub | `allocation/control_surface_angle` | `farol2_interfaces/ControlSurfaceAngle` (deg) | `thrust_rudder` |

## Parameters

| Parameter | Type | Mode | Meaning |
|---|---|---|---|
| `node_frequency` | double | both | Allocation rate (Hz) |
| `allocation.type` | string | both | `thrust` (alias `static_thruster_allocation`) or `thrust_rudder` (aliases `thruster_rudder`, `thruster_rudder_allocation`) |
| `allocation.thrusters.base_frame` | string | both | Usually `base_link`; `frame_prefix` is prepended |
| `allocation.thrusters.frames` | string[] | both | Thruster TF frames, in RPM-vector order |
| `allocation.thrusters.open_loop` | bool | `thrust_rudder` | Do not publish RPM |
| `rpm_conversion.coef_fwd`, `coef_bwd` | double[3] | both | Static curve `[c0, c1, c2]` |
| `rpm_conversion.max_rpm`, `min_rpm` | double | both | RPM limits |
| `rpm_conversion.mode` | int | `thrust_rudder` | 0 = propeller model, other = static curve |
| `rpm_conversion.rho`, `K_T_BP`, `prop_pitch`, `D` | double | `thrust_rudder` | Water density, bollard-pull thrust coefficient, pitch (m), diameter (m) |
| `allocation.rudder.limits.min`, `max` | double (deg) | `thrust_rudder` | Rudder stops |
| `allocation.rudder.cm_distance` | double (m) | `thrust_rudder` | Rudder to centre of mass |
| `allocation.model.K_s`, `K_L`, `K_D0`, `K_D1` | double | `thrust_rudder` | Rudder torque, lift and drag coefficients |

All parameters are required (declared without defaults), so a missing key stops the node at start-up.

## Known gaps

[#2](../../GAPS.md#2-simulation-and-stack-run-on-different-clocks-high) (wall timers), [#3](../../GAPS.md#3-two-nodes-publish-to-the-same-rpm_command-topic-high), [#35](../../GAPS.md#35-rudder-allocation-mixes-degrees-and-radians-and-ignores-flow-speed-high), [#38](../../GAPS.md#38-magicelectric-rudder-topics-no-longer-match-the-drivers-high) (the real rudder driver still listens on `allocation/rudder_command`). The per-node pages `rpm_conversion.md`, `static_thruster_allocation.md` and `thruster_rudder_allocation.md` in the repository are empty stubs for nodes that no longer exist and can be deleted.
