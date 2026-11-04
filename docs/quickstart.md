# FAROL 2 quickstart and conventions

This page is the shortest path from a built workspace to a moving simulated vehicle. For step-by-step procedures see [runbook.md](runbook.md).

## Architecture in one picture

```text
drivers or farol2_sim
   |  /<ns>/measurement/{imu,gnss,ned_utm,velocity_over_ground,velocity_through_water,depth,altimeter,thruster_rpm,control_surface_angle}
   v
farol2_nav  (node: navigation_filter)  ->  /<ns>/nav/filter/state   (farol2_interfaces/NavigationState, 10 Hz)
   |
   +--> farol2_waypoint        (node: waypoint)        \
   +--> farol2_planning        (node: planning/paths)    >--> /<ns>/inner_loop/ref/<channel>   (std_msgs/Float32)
   +--> farol2_path_following  (node: path_following)  /
                                   |
                                   v
farol2_inner_loop (node: inner_loop, open_loop)  ->  /<ns>/allocation/{thrust_x,thrust_y,thrust_z,torque_x,torque_y,torque_z}  (Float32)
                                   |
                                   v
farol2_allocation (node: actuator_allocation)    ->  /<ns>/allocation/rpm_command (ThrusterRPM) and /<ns>/allocation/control_surface_angle (ControlSurfaceAngle)
                                   |
                                   v
drivers or farol2_sim
```

`<ns>` is `<vehicle_name><vehicle_id>`, for example `delfim0` or `magicelectric0`. Every topic and service below is under `/<ns>/`.

## Units and frames

| Quantity | Unit | Where |
|---|---|---|
| Position | UTM metres, `northing` (x) and `easting` (y) | `NavigationState.utm_position`, waypoint and path services |
| Lat/lon, attitude, course, angular velocity | degrees, deg/s | `NavigationState` |
| Linear references (surge, sway, heave, depth, altitude) | m/s, m | `inner_loop/ref/*` |
| Angular references (yaw, pitch, roll and their rates) | degrees, deg/s | `inner_loop/ref/*` (converted to radians inside `inner_loop`) |
| Rudder / control surface angle | degrees | `ControlSurfaceAngle`, `allocation.yaml` limits |
| Thruster command | rpm | `ThrusterRPM.rpm[]` |
| Wrench from inner loop | N and N.m | `allocation/thrust_*`, `allocation/torque_*` |

NED convention: x north, y east, z down. `velocity_over_ground` and `velocity_through_water` are expressed in the body frame.

## The `mission_status` topic

`/<ns>/mission_status` (`std_msgs/Int8`) is the vehicle mode. It replaces the ROS 1 `Flag` topic.

| Value | Meaning | Set by |
|---|---|---|
| 0 | idle | waypoint stop (`x=-2,y=-2`), `path_following/Stop` |
| 4 | following a waypoint | `waypoint` when a waypoint is accepted |
| 6 | following a path | `path_following` when `Start` is called |

Why it matters:
- `waypoint` cancels its timer if the status becomes anything other than 4.
- `farol2_allocation` in `thrust_rudder` mode (magicelectric) and the `open_loop` node output **nothing** while the status is 0. To command a magicelectric by hand you must publish `mission_status` yourself (see the runbook).
- The `thrust` allocation (delfim and other AUVs with static thrusters) does not look at it.

## Timing gates (why "nothing moves")

- `inner_loop` runs a channel only if its reference is **fresher than `ref_timeout`** (default 2 s). Publish references continuously, at 5 Hz or more.
- `inner_loop` skips a cycle if `dt <= 0` or `dt > 2/node_frequency`.
- `inner_loop` needs at least one `NavigationState` before it outputs anything.
- `allocation` zeroes any wrench axis older than `2/node_frequency`, and does not start until every thruster frame is available in TF (`tf_static` from `farol2_description`).
- Only channels with `enabled: true` in the vehicle's `inner_loop.yaml` exist. If you publish to a disabled channel, nothing happens.
- `yaw` and `yaw_rate` both add into `torque_z`. Drive one of them, not both.

## Launch commands

```bash
# Simulation: sim_clock + simulator + full GNC stack
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=delfim
ros2 launch farol2_bringup start_vehicle_sim.launch.py vehicle_name:=magicelectric

# Real vehicle (needs the farol2_drivers repository in the same workspace)
ros2 launch farol2_bringup start_vehicle_real.launch.py vehicle_name:=magicelectric
```

`vehicle_id` is picked automatically (first free number) unless you pass it. The simulator is chosen from the vehicle name: `magicelectric` uses `magic_electric_sim`, any other vehicle uses `auv_sim`.
