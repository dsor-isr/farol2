# farol2_allocation

Converts the body wrench requested by the inner loop into actuator commands. One node, `allocation_node` (ROS node name `actuator_allocation`, namespace `/<vehicle_name><vehicle_id>`), with the strategy selected by `allocation.type`. The mathematics (pseudo-inverse allocation, rudder model, force to rpm) is in [old_README.md](old_README.md), the implementation notes are below.

| `allocation.type` | Class | Vehicles | Page |
|---|---|---|---|
| `thrust` | `StaticThrusterAllocator` | delfim, bluerov, mblack, mred, myellow | [static_thruster_allocation](static_thruster_allocation.md) |
| `thrust_rudder` | `RudderAllocator` | magicelectric | [thruster_rudder_allocation](thruster_rudder_allocation.md) |

Shared pieces: `ThrusterGeometry` (thrust matrix from TF) and `ThrusterRpmConverter` ([rpm_conversion](rpm_conversion.md)).

## Topics (relative to `/<ns>/`, remapped in `launch/farol2_allocation.launch.py`)
| Direction | Topic | Type |
|---|---|---|
| in | `allocation/thrust_x`, `thrust_y`, `thrust_z`, `torque_x`, `torque_y`, `torque_z` | `std_msgs/Float32` (N, N.m) from `inner_loop` |
| in | `nav/filter/state` | `NavigationState` (only in `thrust_rudder`) |
| in | `mission_status` | `std_msgs/Int8` (only in `thrust_rudder`) |
| out | `allocation/rpm_command` | `farol2_interfaces/ThrusterRPM` (rpm, one entry per thruster) |
| out | `allocation/control_surface_angle` | `farol2_interfaces/ControlSurfaceAngle` (deg), only in `thrust_rudder` |

## Behaviour to know
- **TF requirement.** On start the node looks up `<base_frame> -> <thruster frame>` for every entry of `allocation.thrusters.frames` (retrying every 500 ms, with a `Waiting for transform` warning). Nothing is published until all of them exist, so `farol2_description`'s `tf_static` launch must be running. The thrust matrix is built from those transforms and assumes each thruster pushes along the **x axis of its frame** (the old `thrust_axis` parameter was removed).
- **Freshness.** Each wrench axis is zeroed if its last message is older than `2 / node_frequency`.
- **`mission_status` gate.** In `thrust_rudder` mode nothing is produced while `mission_status == 0`. The `thrust` mode ignores it.
- **`allocation.thrusters.open_loop: true`** (magicelectric) suppresses this node's rpm output, because the `open_loop` node of `farol2_inner_loop` publishes `allocation/rpm_command` instead.
- **Frame prefix.** `frame_prefix` (default empty) is prepended to frame names. The launch file sets the TF prefix to `<ns>/`, which is why the log says `base frame 'delfim0/base_link'`.

## Parameters (`allocation.yaml`, `/**: actuator_allocation: ros__parameters:`)
| Parameter | Used by | Meaning |
|---|---|---|
| `node_frequency` | both | Output rate and freshness window. |
| `allocation.type` | both | `thrust` or `thrust_rudder`. |
| `allocation.thrusters.base_frame`, `allocation.thrusters.frames` | both | TF frames defining the thrust matrix. |
| `allocation.thrusters.open_loop` | `thrust_rudder` | See above. |
| `allocation.rudder.limits.{min,max}` | `thrust_rudder` | Rudder saturation, degrees. |
| `allocation.rudder.cm_distance` | `thrust_rudder` | Rudder arm `l` (m). |
| `allocation.model.{K_s,K_L,K_D0,K_D1}` | `thrust_rudder` | Rudder model constants. |
| `rpm_conversion.*` | both | See [rpm_conversion](rpm_conversion.md). |

All of these are declared **without defaults**: a missing key stops the node at start-up.
