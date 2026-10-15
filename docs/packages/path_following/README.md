# farol2_path_following

Path-following outer loop. One node, `path_following` (executable `path_following_node`), hosts one of eleven path-following laws at a time. It reads the vehicle state and the path data served by `farol2_planning` and publishes references for `farol2_inner_loop`.

The algorithms and their theory are described in the survey *A review of path following control strategies for autonomous robotic vehicles: theory, simulations, and experiments* ([arXiv:2204.07319](https://arxiv.org/abs/2204.07319)). The class design is explained in [implementation.md](implementation.md). Original authors: Marcelo Jacinto, João Quintas, João Cruz, Hung Tuan.

## Algorithms

| Service suffix | Class | References published | Path mode | Gains key |
|---|---|---|---|---|
| `SetAguiar` | `Aguiar` | surge, yaw_rate, gamma | virtual target | `aguiar` |
| `SetBreivik` (default) | `Breivik` | surge, yaw, gamma | virtual target | `breivik` |
| `SetFossen` | `Fossen` (LOS) | surge, yaw | closest point | none |
| `SetIlos` | `ILOS` (integral LOS) | surge, yaw, gamma | closest point | `ilos` |
| `SetLapierre` | `Lapierre` | surge, yaw_rate, gamma | virtual target | `lapierre` |
| `SetMarcelo` | `Marcelo` | surge, yaw_rate, gamma, current x/y | virtual target | `marcelo` |
| `SetPramod` | `Pramod` | surge, yaw, gamma | closest point | `pramod` |
| `SetRavi` | `Ravi` | surge, yaw, gamma | closest point | `ravi` |
| `SetRelativeHeading` | `RelativeHeading` | surge, sway, yaw, gamma | virtual target | `relative_heading` |
| `SetRomulo` | `Romulo` | surge, sway, gamma | virtual target | `romulo` |
| `SetSamson` | `Samson` | surge, yaw_rate | closest point | `samson` |

"Virtual target" algorithms drive the path parameter γ themselves and publish it on `path_following/gamma`; `paths` answers with the path data at that γ. "Closest point" algorithms switch `paths` to closest-point mode on start.

## Running a mission

1. Build the path with the `farol2_planning` spawn services and set a speed with `SetConstVdVehicle` or `SetConstVdRabbit`.
2. Select an algorithm with `path_following/Set<Name>` (refused while running).
3. Call `path_following/Start`. The node publishes `mission_status = 6` and starts its timer.
4. At the end of the path it asks `waypoint` to hold a point ahead (`send_wp_standard` with `-3, -3`) and calls `nav/filter/reset_dead_reckoning` (no server exists, [GAPS #10](../../GAPS.md#10-reset_dead_reckoning-service-has-no-server-medium)).
5. It stops when `Stop` is called or `mission_status` changes away from 6, and then resets the path.

## Topics

| Direction | Topic | Type |
|---|---|---|
| sub | `nav/filter/state` | `farol2_interfaces/NavigationState` |
| sub | `path_following/path_data` | `farol2_planning/PathData` |
| sub | `path_following/vc` | `std_msgs/Float32` (cooperative-PF speed correction; producer only on `feature/cpf_control`) |
| sub / pub | `mission_status` | `std_msgs/Int8` |
| pub | `inner_loop/ref/{surge,sway,yaw,yaw_rate}` | `std_msgs/Float32` |
| pub | `path_following/gamma` | `std_msgs/Float32` |
| pub | `current/x`, `current/y` | `std_msgs/Float32` (Marcelo) |
| pub | `path_following/debug` | `farol2_path_following/PFDebug` |

The node uses `eta1 = [northing, easting, altimeter]` and the body-frame `velocity_over_ground` from the nav state ([GAPS #9](../../GAPS.md#9-3-d-path-following-uses-the-altimeter-as-the-vehicles-z-medium)).

## Services

| Service | Type | Purpose |
|---|---|---|
| `path_following/Start` | `StartPF` (empty request) | Start the selected algorithm |
| `path_following/Stop` | `StopPF` (empty request) | Stop and reset the path |
| `path_following/UpdateGains` | `UpdateGainsPF` (`float64[] gains`) | Replace the current algorithm's gains, in the YAML order |
| `path_following/ResetVT` | `ResetVT` (`float64 value`) | Move the virtual target to γ = `value` |
| `path_following/Set<Name>` | `SetPF` (empty request) | Select an algorithm (see table) |

Clients used: `waypoint/send_wp_standard`, `nav/filter/reset_dead_reckoning`, `ResetPath`, `SetMode`.

## Parameters

| Parameter | Type | Meaning |
|---|---|---|
| `node_frequency` | double | Control rate (Hz) |
| `default_algorithm` | string, default `breivik` | Algorithm selected at start-up (falls back to Breivik if invalid) |
| `controller_gains.<key>.*` | see below | Gains per algorithm. **Order matters**: `UpdateGains` maps the array positionally |

| Key | Gains (example values from `mred`) |
|---|---|
| `lapierre` | `k1` 1.0, `k2` 0.3, `k3` 0.3, `theta` 0.8, `k_delta` 1.0 |
| `marcelo` | `delta` -1.0, `kx` 0.5, `ky` 0.5, `kz` 0.5, `k_pos` 2.0, `k_currents` 0.2, `rd` [0,0,1], `d` [0,0,0] |
| `aguiar` | `delta` -1.0, `kx` 0.5, `ky` 0.5, `kz` 0.5, `k_pos` 0.5, `k_currents` 0.5 |
| `romulo` | `ke` [0.05,0,0,0.05], `kz` 0.2 |
| `pramod` | `kp` 0.02, `ki` 0.0005 |
| `ravi` | `e_turn` 45.0 (m), `min_corridor` 45.0 (m), `xi` 1.0, `w0_min` 0.08, `epsilon_current` 0.2 |
| `samson` | `k1` 1.0, `k2` 0.3, `k3` 0.3, `theta` 0.8, `k_delta` 1.0 |
| `relative_heading` | `kx` 0.5, `ky` 0.1, `kz` 0.2, `yaw_offset` 0.785 (rad), `p_sat` [1,1] |
| `breivik` | `delta_h` 10.0 (m) |
| `ilos` | `delta` 10.0 (m lookahead), `ki` 0.05 (only in `magicelectric` and the package example, [GAPS #8](../../GAPS.md#8-selecting-ilos-crashes-the-node-on-5-of-6-vehicles-medium)) |

Parameters are auto-declared from the YAML, so a gain block missing from the vehicle's YAML makes the corresponding `Set<Name>` throw.

## Adding an algorithm

1. Derive from `PathFollowing` (`include/algorithms/PathFollowing.h`) and implement `callPFController`, `publish`, `start`, `stop`, `reset`, `setPFGains` (and `resetVirtualTarget` if it uses one).
2. Add a `createController<Name>()` in `PathFollowingNode.cpp` that creates its publishers (store them in `publishers_`) and reads its gains.
3. Register it in `switchController()` and add a `Set<Name>` service constant, server and launch remap.
4. Add its gains to **every** vehicle's `path_following.yaml`.

## Known gaps

[#8](../../GAPS.md#8-selecting-ilos-crashes-the-node-on-5-of-6-vehicles-medium), [#9](../../GAPS.md#9-3-d-path-following-uses-the-altimeter-as-the-vehicles-z-medium), [#10](../../GAPS.md#10-reset_dead_reckoning-service-has-no-server-medium).
