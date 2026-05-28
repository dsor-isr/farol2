# farol2_clock

`sim_clock` publishes the simulation `/clock` used by Farol2 simulation launches.

The node advances simulated time by `1 / node_frequency` seconds on each tick and runs the wall timer at `node_frequency * speedup`. The launch file passes the selected vehicle `sim.yaml` directly to the node, matching the configured sim-node frequency.

## Topics

- Publishes `/clock` (`rosgraph_msgs/msg/Clock`)

## Parameters

- `node_frequency` (`double`): simulated clock tick rate in Hz, read from `sim.yaml`.
- `speedup` (`double`): wall-time speed multiplier, read from `sim.yaml`.

## Launch

```bash
ros2 launch farol2_clock farol2_clock.launch.py vehicle_name:=magicelectric vehicle_id:=0
```

In normal simulation use, `farol2_bringup/launch/start_vehicle_sim.launch.py` starts this node automatically before launching the Farol stack.
