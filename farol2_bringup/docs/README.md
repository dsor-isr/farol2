# Farol bringup

Configuration files live in `config/<vehicle>/<profile>/`; shared launch files
live in `launch/`. There is no runtime process-manager node.

## Simulation

Start one world in a dedicated DDS domain, then join it with vehicles from other
terminals. Reuse the same domain for vehicles in one world; choose a different
domain for another independent instance.

```bash
ros2 launch farol2_bringup start_simple_world.launch.py ros_domain_id:=41
```

```bash
ros2 launch farol2_bringup start_vehicle_sim.launch.py \
  vehicle_name:=magicelectric vehicle_id:=0 ros_domain_id:=41
```

The world owns `/clock`; vehicle bringup never starts a clock. Every simulated
subsystem uses `use_sim_time=true`. Vehicle startup requires fresh, advancing
clock messages in the selected domain within `clock_timeout` wall seconds
(default 5). Without a ready clock, launch fails before the vehicle stack starts.

`ros_domain_id` defaults to 0 on world and vehicle launches when omitted,
regardless of the shell environment. Pass it explicitly to use another domain. ROS CLI tools and external nodes must use the same domain to join.
Automatic vehicle ID discovery queries that domain; explicit IDs remain useful
for deterministic fleet configurations.

World clock and environmental parameters belong in `config/simple_world.yaml`.
Select the same `world_config` file on the world and vehicle launchers; each
`vehicle_sim` currently loads the shared environment directly. Vehicle initial
positions are local NED coordinates relative to the world geographic origin. `sim_clock.launch.py` and the per-vehicle
`farol2_clock` switch have been replaced by `start_simple_world.launch.py`.

See [simulation documentation](../../farol2_sim/docs/README.md) for multi-world
examples, supported checks and remaining limitations.

## Real hardware

`start_vehicle_real.launch.py` starts the real stack with `use_sim_time=false`.
It does not require a world or simulation clock.
