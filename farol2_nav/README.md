# farol2_nav

`farol2_nav` provides a modular navigation-estimation pipeline in ROS 2.

The package runs one node (`filter_node`) with chained filters over a shared state:

1. `sample_and_hold` maps asynchronous sensor data into state.
2. Optional refinement filters (for example `position_current_ekf`, `yaw_rate_ekf`) run sequentially.
3. Final fused `NavigationState` is published.

This design allows small specialized filters to be stacked, compared, and extended without turning the navigation stack into one monolithic estimator.

![farol2_nav architecture](docs/architecture.png)

## Documentation

- Implementation-focused overview: [docs/README.md](docs/README.md)
- Filter theory pages:
  - [docs/sample_and_hold.md](docs/sample_and_hold.md)
  - [docs/position_current_ekf.md](docs/position_current_ekf.md)
  - [docs/yaw_rate_ekf.md](docs/yaw_rate_ekf.md)

## Where to configure

Typical parameters are in vehicle `nav.yaml` files under `farol2_bringup/config/<vehicle>/default/nav.yaml`.
