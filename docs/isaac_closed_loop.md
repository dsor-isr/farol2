# farol2 myellow in closed loop with Farol-Isaac

(Work in progress: commands, results and pitfalls are completed at the end of the session.)

## Note: DVL mode and the surge/sway/heave loops

The inner loop (`farol2_inner_loop/src/inner_loop_node.cpp`, previously `pid.cpp`) subscribes to `nav/filter/state` and closes
- **surge** on `velocity_through_water.x`, **sway** on `.y`, **heave** on `.z`,
- depth/altitude rate terms on `velocity_over_ground.z`.

`filter_node` fills `velocity_through_water` only from a `measurement/velocity_through_water` message.

`auv_sim` publishes a DVL on **one** topic, chosen by `sensor.dvl.mode` in `sim.yaml`:
`bottom_track` -> `measurement/velocity_over_ground`, `water_track` -> `measurement/velocity_through_water`.
`myellow/default/sim.yaml` used `bottom_track`, so in `auv_sim` the nav state had `velocity_through_water = 0` (checked on a running stack:
true surge 1.00 m/s, `nav/filter/state.velocity_through_water` 0, `velocity_over_ground` 1.00). The surge PI then saw a zero measurement,
wound up, and the vehicle accelerated far past the reference (ref 0.5 m/s gave 0.9 m/s and rising).

Changes:
- `myellow/default/sim.yaml`: `dvl.mode: "water_track"`, so the simulated vehicle feeds the surge/sway/heave loops.
- The Isaac bridge (`modules/farol-isaac-cli/lib/ros_bridge.py`) publishes both `measurement/velocity_over_ground` (bottom track) and
  `measurement/velocity_through_water` (velocity relative to the water, i.e. minus the world `water:current`), both in the body frame
  with `dvl_link` as frame id.
