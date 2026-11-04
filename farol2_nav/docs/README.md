# farol2_nav

## Overview

`farol2_nav` implements navigation estimation as a **single filter pipeline inside one ROS 2 node** (`filter_node`), instead of running one monolithic estimator.

At each timer tick:

1. A measurement snapshot is assembled from asynchronous topics.
2. The pipeline starts from a clean internal state.
3. `sample_and_hold` (pass-through) fills state channels directly from sensors.
4. Additional filters (for example `asv_dynamics_model`, `current_estimator`, `yaw_rate_ekf`) refine selected parts of that state.
5. The final fused `NavigationState` is published.

This architecture favors composition: each filter can focus on one sub-problem and still cooperate through a shared state.

![farol2_nav architecture](architecture.png)

## Why this implementation style

- **Composable**: filters are stacked in `filters` order and can be combined.
- **Incremental**: adding a new filter does not require rewriting existing estimators.
- **Traceable**: optional stage outputs (`<filter_name>/state`) allow per-stage inspection.
- **Robust to async sensors**: input topics are buffered and consumed at fixed-rate ticks.
- **Fail-soft**: each tick only sees the measurements that arrived since the previous tick; missing measurements do not crash the pipeline.

## Runtime model

### Input buffering

Subscribers update a `MeasurementSnapshot` cache asynchronously. Each message type has its own freshness timestamp.

### Tick-driven processing

The timer is a wall-clock timer running at `node_frequency`; `dt` is the difference between consecutive `now()` stamps, i.e. node time (`/clock` when `use_sim_time` is true). On each tick:

- filters run sequentially over the same `State` object,
- the final state is always published,
- consumed measurements are flushed (next tick only sees newly arrived data).

### Filter chain

`sample_and_hold` is always first and implicit.

Then each key in `filters` instantiates one plugin:

- `asv_dynamics_model`
- `current_estimator`
- `yaw_rate_ekf`

Unknown keys are skipped with a warning.

## Main topics

### Subscribed (short names before remap)

- `imu`
- `gnss`
- `ned_utm`
- `velocity_over_ground`
- `velocity_through_water`
- `depth`
- `altimeter`
- `control_surface_angle`
- `thruster_rpm`

### Published

- `state` (final output)
- `<filter_name>/state` (intermediate outputs when `publish_all_steps=true`, excluding final stage)
- filter-specific debug topics (for example in `yaw_rate_ekf`)

## Core parameters

- `node_frequency` (Hz): pipeline tick rate.
- `publish_all_steps` (bool): publish intermediate stage states.
- `filters` (string[]): ordered plugins after `sample_and_hold`.
- `measurements` (string[]): which measurement subscriptions are created (`gnss`, `imu`, `depth`, `velocity_over_ground`, `velocity_through_water`, `utm_ned`, `altimeter`, `thruster_rpm`, `control_surface_angle`; default `["gnss", "imu"]`).

There is no per-sensor `timeouts.*` parameter in the current code: the freshness handling is the per-tick flush of the measurement snapshot.

Plugin parameters live under:

- `plugins.sample_and_hold.*`
- `plugins.asv_dynamics_model.*`
- `plugins.current_estimator.*`
- `plugins.yaw_rate_ekf.*`

See dedicated filter pages below for details.

## Per-filter documentation

- [sample_and_hold](sample_and_hold.md)
- [asv_dynamics_model](asv_dynamics_model.md)
- [current_estimator](current_estimator.md)
- [yaw_rate_ekf](yaw_rate_ekf.md)

## Adding a new filter plugin

1. Create a class implementing `BaseFilter` (`name`, `configure`, `compute`).
2. Add the implementation to `src/filters` and include header in `filter_node.cpp`.
3. Instantiate it in `build_pipeline()` using a new key in the `filters` switch.
4. Add remapping for `<your_filter>/state` in launch (ROS 2 remapping has no wildcard support for this pattern).
5. Add plugin parameters to vehicle `nav.yaml`.

The key idea is to modify only the extension points, not the existing filter logic.

## Launch and configuration
- Launch: `launch/farol2_nav.launch.py` (args `vehicle_name`, `vehicle_id`, `use_sim_time`, `config_to_use`, `sample_and_hold`, `filter_node`). The ROS node name is `navigation_filter`, namespace `/<ns>`.
- Subscriptions are remapped to `/<ns>/measurement/<name>`; the output is `/<ns>/nav/filter/state`.
- Per-vehicle parameters are in `farol2_bringup/config/<vehicle>/default/nav.yaml`. **They are keyed `filter_node:` while the node runs as `navigation_filter`, so they are currently not applied** (the node runs on the code defaults). Rename the key to apply them. `config/nav.yaml` inside this package is an example only and is not installed.
- Without a `velocity_over_ground` entry in `measurements`, `NavigationState.velocity_over_ground` stays zero even if the simulator publishes it.
- Services/debug: `current_estimator/tune` (`TuneCurrentEstimator`), the yaw-EKF tune service (`TuneYawRateEkf`), `current_estimator/debug` (`CurrentEstimatorDebug`), `yaw_rate_ekf/debug` (`YawRateEkfDebug`), `asv_dynamics_model/{uvr,tau}`.
